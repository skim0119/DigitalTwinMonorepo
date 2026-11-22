#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace py = pybind11;

using Motor = std::tuple<int, int, double, double, double, long, long>;
using Status = std::tuple<double, long, double>;

class Gpio {
    int fd = -1;
    volatile uint32_t* registers = nullptr;
    std::vector<std::pair<int, uint32_t>> modes;

    void mode(int pin, uint32_t value) {
        auto& select = registers[pin / 10];
        const unsigned shift = (pin % 10) * 3;
        select = (select & ~(7u << shift)) | (value << shift);
    }

public:
    Gpio(const std::vector<Motor>& motors, bool simulate, unsigned enabled_mask) {
        if (simulate) return;
        std::string board;
        std::ifstream model("/proc/device-tree/model");
        if (!std::getline(model, board) || board.find("Raspberry Pi 4 Model B") != 0)
            throw std::runtime_error("Live GPIO requires a Raspberry Pi 4 Model B");
        modes.reserve(motors.size() * 2);
        fd = open("/dev/gpiomem", O_RDWR | O_CLOEXEC);
        if (fd < 0) throw std::runtime_error("Cannot open /dev/gpiomem");
        if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
            close(fd);
            throw std::runtime_error("Another native controller owns the motor GPIO");
        }
        void* mapping = mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (mapping == MAP_FAILED) {
            close(fd);
            throw std::runtime_error("Cannot map /dev/gpiomem");
        }
        registers = static_cast<volatile uint32_t*>(mapping);
        for (size_t i = 0; i < motors.size(); ++i) {
            if (!(enabled_mask & (1u << i))) continue;
            const auto& motor = motors[i];
            for (int pin : {std::get<0>(motor), std::get<1>(motor)}) {
                modes.emplace_back(pin, (registers[pin / 10] >> ((pin % 10) * 3)) & 7u);
                write(pin, false);
                mode(pin, 1);
            }
        }
    }

    ~Gpio() {
        if (!registers) return;
        for (auto [pin, previous] : modes) {
            write(pin, false);
            mode(pin, previous);
        }
        munmap(const_cast<uint32_t*>(registers), 4096);
        close(fd);
    }

    void write(int pin, bool high) {
        if (registers) registers[(high ? 0x1c : 0x28) / 4] = 1u << pin;
    }
};

struct Reading {
    Clock::time_point stamp{};
    double force = 0;
};

struct Channel {
    std::mutex mutex;
    std::condition_variable wake;
    std::thread thread;
    Reading reading;
    Clock::time_point last_usable{};
    int samples_to_recover = 0;
    double target = 0, force = 0, speed = 0;
    std::array<double, 6> terms{}; // P, I, D, feedforward, requested and filtered step rates.
    double target_rate = 0, kff = 0;
    std::array<double, 3> gains{};
    bool force_mode = true;
    bool reset_pid = false;
    double manual_rate = 0;
    long steps = 0, tightening_steps = 0;
};

class Controller {
    static constexpr auto sample_age = 1s;
    static constexpr auto pause_age = 250ms;
    static constexpr auto edge_guard = 10us;
    const std::vector<Motor> motors;
    const double max_force, max_rate, deadband, alpha, max_acceleration, output_tau;
    const int selected;
    const int excluded;
    const bool manual;
    const long tightening_limit;
    std::array<Channel, 6> channels;
    std::unique_ptr<Gpio> gpio;
    std::thread acquisition;
    int receiver = -1;
    std::atomic<bool> stopping{false};
    std::atomic<const char*> fault{nullptr};
    char tension_fault[160]{};
    std::mutex lifecycle;
    // Protected by each channel's mutex. A snapshot/command locks all channels.
    Clock::time_point expires{};

    bool enabled(size_t i) const {
        return i != static_cast<size_t>(excluded) &&
               (selected < 0 || i == static_cast<size_t>(selected));
    }

    void fail(const char* message) {
        const char* expected = nullptr;
        fault.compare_exchange_strong(expected, message);
        stopping = true;
        for (auto& channel : channels) channel.wake.notify_one();
    }

    void receive() {
        constexpr char request[] = "hx711-fast";
        while (!stopping) {
            char message[512];
            pollfd socket_poll{receiver, POLLIN, 0};
            if (send(receiver, request, sizeof(request) - 1, 0) >= 0 &&
                poll(&socket_poll, 1, 25) > 0) {
                auto count = recv(receiver, message, sizeof(message) - 1, MSG_DONTWAIT);
                if (count > 0) {
                    message[count] = '\0';
                    std::array<Reading, 6> next{};
                    const char* row = message;
                    const auto now = Clock::now();
                    bool valid = true;
                    for (auto& reading : next) {
                        long long nanos = 0;
                        int used = 0;
                        if (std::sscanf(row, "%lld %lf%n", &nanos, &reading.force, &used) != 2 ||
                            row[used] != '\n' || nanos < 0 || !std::isfinite(reading.force)) {
                            valid = false;
                            break;
                        }
                        reading.stamp = Clock::time_point(std::chrono::duration_cast<Clock::duration>(
                            std::chrono::nanoseconds(nanos)));
                        if (reading.stamp > now) {
                            valid = false;
                            break;
                        }
                        row += used + 1;
                    }
                    if (valid && *row == '\0') {
                        for (size_t i = 0; i < channels.size(); ++i) {
                            if (!enabled(i)) continue;
                            auto& channel = channels[i];
                            const auto reading = next[i];
                            std::lock_guard lock(channel.mutex);
                            if (reading.stamp == channel.reading.stamp) continue;
                            if (reading.stamp != Clock::time_point{} && reading.force >= max_force) {
                                std::snprintf(tension_fault, sizeof(tension_fault),
                                              "Measured tension reached the configured limit on motor %zu "
                                              "(tendon_%zu): unfiltered %.9g kgf, limit %.9g kgf",
                                              i, i + 1, reading.force, max_force);
                                fail(tension_fault);
                                return;  // Keep the published fault message immutable.
                            }
                            if (reading.stamp == Clock::time_point{}) {
                                channel.samples_to_recover = 2;
                            } else if (channel.samples_to_recover == 0 || --channel.samples_to_recover == 0) {
                                channel.last_usable = reading.stamp;
                            }
                            channel.reading = reading;
                            channel.wake.notify_one();
                        }
                    }
                }
            }
            std::this_thread::sleep_for(5ms);
        }
    }

    void run(size_t i) {
        const auto [step_pin, dir_pin, kp, ki, kd, low, high] = motors[i];
        auto& channel = channels[i];
        Clock::time_point sample{}, last_step{}, dir_ready{}, rate_update = Clock::now();
        double integral = 0, filtered = 0, output = 0, force_rate = 0, applied_rate = 0;
        int active_direction = 0, pin_direction = 0;
        bool sensor_paused = false;
        std::unique_lock lock(channel.mutex);
        while (!stopping) {
            const auto now = Clock::now();
            const double rate_dt = std::chrono::duration<double>(now - rate_update).count();
            rate_update = now;
            const auto reading = channel.reading;
            if (expires != Clock::time_point{} && now >= expires && (!manual || channel.manual_rate != 0)) {
                fail("Motion command expired");
                break;
            }
            if (channel.reset_pid) {
                output = 0;
                integral = 0;
                active_direction = 0;
                applied_rate = force_rate = 0;
                channel.terms = {};
                channel.reset_pid = false;
            }
            if (!manual && expires != Clock::time_point{}) {
                if (now - channel.last_usable >= sample_age) {
                    fail("HX711 reading missing or stale");
                    break;
                }
                if (channel.samples_to_recover || now - channel.last_usable >= pause_age) {
                    sensor_paused = true;
                    channel.speed = 0;
                    applied_rate = force_rate = 0;
                    channel.terms = {};
                    channel.wake.wait_until(lock, std::min(expires, channel.last_usable + sample_age));
                    continue;
                }
            }
            const bool fresh = reading.stamp != Clock::time_point{} && now - reading.stamp < pause_age;
            const bool new_sample = fresh && reading.stamp != sample;
            if (new_sample) {
                const double dt = sensor_paused || sample == Clock::time_point{} ? 0 :
                    std::chrono::duration<double>(reading.stamp - sample).count();
                sensor_paused = false;
                const double previous_measured = std::max(0.0, filtered);
                filtered = dt <= 0 ? reading.force : filtered + alpha * (reading.force - filtered);
                sample = reading.stamp;
                channel.force = filtered;
                if (channel.force_mode && expires != Clock::time_point{}) {
                    force_rate = alpha * channel.target_rate + (1 - alpha) * force_rate;
                    const double measured = std::max(0.0, filtered);
                    const double error = channel.target - measured;
                    auto& terms = channel.terms;
                    terms[0] = terms[1] = terms[2] = 0;
                    if (active_direction && active_direction * error <= deadband / 2) {
                        active_direction = 0;
                        integral = 0;
                        output = 0;
                    } else {
                        const bool continuing = active_direction != 0;
                        if (!continuing && std::abs(error) > deadband) {
                            active_direction = error > 0 ? 1 : -1;
                        }
                        if (active_direction) {
                            if (measured == 0) integral = std::max(0.0, integral);
                            const double candidate = integral + error * dt;
                            const double derivative = continuing && dt > 0 ? (previous_measured - measured) / dt : 0;
                            const auto [kp, ki, kd] = channel.gains;
                            terms[0] = kp * error;
                            terms[1] = ki * candidate;
                            terms[2] = kd * derivative;
                            const double proposed = terms[0] + terms[1] + terms[2];
                            if (!std::isfinite(proposed)) {
                                fail("Non-finite PID output");
                                break;
                            }
                            if (std::abs(proposed + channel.kff * force_rate) <= max_rate) integral = candidate;
                            output = proposed * active_direction > 0 ? proposed : 0;
                        }
                    }
                    terms[3] = channel.kff * force_rate;
                    const double requested = output + terms[3];
                    if (!std::isfinite(requested)) {
                        fail("Non-finite PID output");
                        break;
                    }
                    terms[4] = std::clamp(requested, -max_rate, max_rate);
                    const double weight = output_tau > 0 ? -std::expm1(-dt / output_tau) : 1;
                    terms[5] += weight * (terms[4] - terms[5]);
                }
            }
            if (expires == Clock::time_point{} || (manual && channel.manual_rate == 0)) {
                channel.wake.wait_for(lock, 5ms);
                continue;
            }
            const double remaining = channel.target - channel.steps;
            double speed = manual ? channel.manual_rate : channel.force_mode ? output :
                std::clamp(remaining / 0.065, -max_rate, max_rate);
            auto next = manual ? expires : std::min(expires, channel.last_usable + pause_age);
            if (!manual && channel.force_mode) {
                speed = channel.terms[5];
                if (!std::isfinite(speed)) {
                    fail("Non-finite PID output");
                    break;
                }
                speed = std::clamp(speed, -max_rate, max_rate);
                if (max_acceleration > 0) {
                    const double goal = applied_rate * speed < 0 ? 0 : speed;
                    const double delta = max_acceleration * std::min(rate_dt, 0.005);
                    applied_rate += std::clamp(goal - applied_rate, -delta, delta);
                    if (applied_rate != speed) next = std::min(next, now + 5ms);
                    speed = applied_rate;
                }
            }
            const int direction = speed > 0 ? 1 : -1;
            channel.speed = 0;
            if (std::abs(speed) >= 0.3 &&
                (channel.force_mode || std::abs(remaining) >= 1) &&
                (direction > 0 ? channel.steps < high : channel.steps > low)) {
                auto step_time = last_step + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(1 / std::abs(speed)));
                step_time = std::max(step_time, dir_ready);
                channel.speed = speed;
                if (now >= step_time) {
                    if (pin_direction != direction) {
                        gpio->write(dir_pin, direction > 0);
                        pin_direction = direction;
                        dir_ready = Clock::now() + edge_guard;
                        continue;
                    }
                    if (stopping || Clock::now() >= next) continue;
                    gpio->write(step_pin, true);
                    last_step = Clock::now();
                    while (Clock::now() - last_step < edge_guard) {}
                    gpio->write(step_pin, false);
                    channel.steps += direction;
                    if (manual && channel.steps == channel.target) {
                        channel.manual_rate = 0;
                        channel.speed = 0;
                    }
                    if (direction > 0 && ++channel.tightening_steps == tightening_limit) {
                        fail("Tightening STEP limit reached");
                        break;
                    }
                    continue;
                }
                next = std::min(next, step_time);
            }
            channel.wake.wait_until(lock, next);
        }
        channel.speed = 0;
        gpio->write(step_pin, false);
    }

    auto lock_channels() {
        return std::scoped_lock(channels[0].mutex, channels[1].mutex, channels[2].mutex,
                                channels[3].mutex, channels[4].mutex, channels[5].mutex);
    }

public:
    Controller(std::vector<Motor> config, int port, double force, double rate,
               double band, double smoothing, bool simulate, int motor, long max_tightening_steps, bool manual_motion = false,
               int skip_motor = -1, double acceleration = 0, std::array<double, 6> feedforward = {}, double output_filter_ms = 0)
        : motors(std::move(config)), max_force(force), max_rate(rate),
          deadband(band / 1000), alpha(smoothing), max_acceleration(acceleration), output_tau(output_filter_ms / 1000),
          selected(motor), excluded(skip_motor), manual(manual_motion), tightening_limit(max_tightening_steps) {
        if (motors.size() != channels.size() || port < 1 || port > 65535 ||
            !std::isfinite(force) || force <= 0 || !std::isfinite(rate) || rate <= 0 ||
            !std::isfinite(band) || band < 0 || !(alpha > 0 && alpha <= 1) ||
            !std::isfinite(output_filter_ms) || output_filter_ms < 0 || !std::isfinite(max_acceleration) || max_acceleration < 0 ||
            selected < -1 || selected >= 6 || excluded < -1 || excluded >= 6 ||
            (selected >= 0 && excluded >= 0) || tightening_limit < 0)
            throw std::invalid_argument("Invalid controller configuration");
        uint32_t used = 0;
        unsigned enabled_mask = 0;
        for (size_t i = 0; i < motors.size(); ++i) {
            const auto [step, dir, kp, ki, kd, low, high] = motors[i];
            if (enabled(i)) enabled_mask |= 1u << i;
            channels[i].gains = {kp, ki, kd};
            channels[i].kff = feedforward[i];
            for (int pin : {step, dir}) {
                if (pin < 0 || pin > 27 || (used & (1u << pin)))
                    throw std::invalid_argument("Invalid or duplicated motor pin");
                used |= 1u << pin;
            }
            for (double gain : {kp, ki, kd, feedforward[i]})
                if (!std::isfinite(gain) || gain < 0) throw std::invalid_argument("Invalid PID gain");
            if (low > 0 || high < 0 || low >= high)
                throw std::invalid_argument("Invalid step bounds");
        }
        gpio = std::make_unique<Gpio>(motors, simulate, enabled_mask);
        try {
            if (!manual) {
                receiver = socket(AF_INET, SOCK_DGRAM, 0);
                if (receiver < 0) throw std::runtime_error("Cannot open HX711 socket");
                sockaddr_in address{};
                address.sin_family = AF_INET;
                address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                address.sin_port = htons(port);
                if (connect(receiver, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
                    throw std::runtime_error("Cannot connect HX711 socket");
                acquisition = std::thread(&Controller::receive, this);
            }
            for (size_t i = 0; i < channels.size(); ++i)
                if (enabled(i)) channels[i].thread = std::thread(&Controller::run, this, i);
        } catch (...) {
            stop();
            throw;
        }
    }

    ~Controller() { stop(); }

    void stop() {
        std::lock_guard guard(lifecycle);
        {
            auto locks = lock_channels();
            stopping = true;
            for (auto& channel : channels) channel.wake.notify_one();
        }
        for (auto& channel : channels)
            if (channel.thread.joinable()) channel.thread.join();
        if (acquisition.joinable()) acquisition.join();
        if (receiver >= 0) {
            close(receiver);
            receiver = -1;
        }
        gpio.reset();
    }

    bool ready() {
        auto locks = lock_channels();
        if (const char* error = fault) throw std::runtime_error(error);
        if (stopping) return false;
        if (manual) return true;
        const auto now = Clock::now();
        for (size_t i = 0; i < channels.size(); ++i) {
            const auto stamp = channels[i].reading.stamp;
            if (enabled(i) && (channels[i].samples_to_recover || stamp == Clock::time_point{} ||
                               now - stamp >= pause_age))
                return false;
        }
        return true;
    }

    void command(const std::array<double, 6>& targets, bool force_mode, double ttl,
                 const std::optional<std::array<double, 3>>& gains,
                 const std::optional<std::array<double, 6>>& target_rates) {
        if (manual) throw std::logic_error("Use manual_command in manual mode");
        if (gains) {
            if (selected < 0 || !force_mode)
                throw std::invalid_argument("Live gains require one motor in force mode");
            for (double gain : *gains)
                if (!std::isfinite(gain) || gain < 0) throw std::invalid_argument("Invalid PID gain");
        }
        if (!(ttl > 0 && ttl <= 0.25)) throw std::invalid_argument("Invalid command lifetime");
        if (target_rates && !force_mode)
            throw std::invalid_argument("Target rates require force mode");
        for (size_t i = 0; i < targets.size(); ++i) {
            if (!enabled(i)) continue;
            const auto target = targets[i];
            if (target_rates && !std::isfinite((*target_rates)[i]))
                throw std::invalid_argument("Invalid force target rate");
            if (!std::isfinite(target) ||
                (force_mode ? (target < 0 || target >= max_force) :
                 (target < std::get<5>(motors[i]) || target > std::get<6>(motors[i]) || target != std::round(target))))
                throw std::invalid_argument("Target outside configured limits");
        }
        auto locks = lock_channels();
        if (const char* error = fault) throw std::runtime_error(error);
        if (stopping) throw std::runtime_error("Controller has stopped");
        const auto now = Clock::now();
        if (expires != Clock::time_point{} && now >= expires) {
            fail("Motion command expired");
            throw std::runtime_error("Motion command expired");
        }
        for (size_t i = 0; i < channels.size(); ++i) {
            const auto& channel = channels[i];
            const auto age = now - channel.last_usable;
            if (enabled(i) && (channel.last_usable == Clock::time_point{} || age >= sample_age ||
                               (expires == Clock::time_point{} &&
                                (channel.samples_to_recover || age >= pause_age))))
                throw std::runtime_error("HX711 reading missing or stale");
        }
        expires = now + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(ttl));
        for (size_t i = 0; i < channels.size(); ++i) {
            if (!enabled(i)) continue;
            auto& channel = channels[i];
            channel.reset_pid |= channel.force_mode != force_mode || (gains && channel.gains != *gains);
            if (gains) channel.gains = *gains;
            channel.target = targets[i];
            channel.target_rate = target_rates ? (*target_rates)[i] : 0;
            channel.force_mode = force_mode;
            channel.wake.notify_one();
        }
    }

    long manual_command(size_t motor, double rate, std::optional<long> target) {
        if (!manual || motor >= channels.size() || !enabled(motor) ||
            !std::isfinite(rate) || std::abs(rate) > max_rate)
            throw std::invalid_argument("Invalid manual motor command");
        auto locks = lock_channels();
        if (const char* error = fault) throw std::runtime_error(error);
        if (stopping) throw std::runtime_error("Controller has stopped");
        const auto now = Clock::now();
        auto& channel = channels[motor];
        const auto low = std::get<5>(motors[motor]);
        const auto high = std::get<6>(motors[motor]);
        const auto end = target.value_or(rate > 0 ? high : low);
        if (end < low || end > high)
            throw std::invalid_argument("Manual step target outside configured limits");
        for (auto& item : channels) {
            if (item.manual_rate != 0 && now >= expires) {
                fail("Motion command expired");
                throw std::runtime_error("Motion command expired");
            }
        }
        for (auto& item : channels) {
            item.manual_rate = 0;
            item.speed = 0;
            item.wake.notify_one();
        }
        channel.target = end;
        if ((rate > 0 && end > channel.steps) || (rate < 0 && end < channel.steps))
            channel.manual_rate = rate;
        expires = now + 200ms;
        return channel.steps;
    }

    long tightening_steps(size_t motor) {
        auto& channel = channels.at(motor);
        std::lock_guard lock(channel.mutex);
        return channel.tightening_steps;
    }

    std::array<std::array<double, 8>, 6> pid_status() {
        auto locks = lock_channels();
        std::array<std::array<double, 8>, 6> result{};
        for (size_t i = 0; i < channels.size(); ++i) {
            result[i][0] = channels[i].force;
            result[i][1] = channels[i].speed;
            std::copy(channels[i].terms.begin(), channels[i].terms.end(), result[i].begin() + 2);
        }
        return result;
    }

    std::array<Status, 6> status() {
        auto locks = lock_channels();
        std::array<Status, 6> result{};
        for (size_t i = 0; i < channels.size(); ++i)
            result[i] = {channels[i].force, channels[i].steps, channels[i].speed};
        return result;
    }
};

PYBIND11_MODULE(_native, module) {
    py::class_<Controller>(module, "Controller")
        .def(py::init<std::vector<Motor>, int, double, double, double, double, bool, int, long, bool, int,
                      double, std::array<double, 6>, double>(),
             py::arg("motors"), py::arg("port"), py::arg("max_force"), py::arg("max_rate"),
             py::arg("deadband_g"), py::arg("filter_alpha"), py::arg("simulate") = false,
             py::arg("motor") = -1, py::arg("max_tightening_steps") = 0,
             py::arg("manual_motion") = false, py::arg("skip_motor") = -1,
             py::arg("max_acceleration") = 0, py::arg("feedforward") = std::array<double, 6>{},
             py::arg("output_filter_ms") = 0)
        .def("ready", &Controller::ready, py::call_guard<py::gil_scoped_release>())
        .def("command", &Controller::command, py::arg("targets"), py::arg("force_mode"),
             py::arg("ttl"), py::arg("gains") = py::none(), py::arg("target_rates") = py::none(),
             py::call_guard<py::gil_scoped_release>())
        .def("manual_command", &Controller::manual_command, py::arg("motor"), py::arg("rate"),
             py::arg("target") = py::none(), py::call_guard<py::gil_scoped_release>())
        .def("tightening_steps", &Controller::tightening_steps, py::call_guard<py::gil_scoped_release>())
        .def("pid_status", &Controller::pid_status, py::call_guard<py::gil_scoped_release>())
        .def("status", &Controller::status, py::call_guard<py::gil_scoped_release>())
        .def("stop", &Controller::stop, py::call_guard<py::gil_scoped_release>());
}
