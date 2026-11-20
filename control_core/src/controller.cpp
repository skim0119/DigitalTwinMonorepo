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
    Gpio(const std::vector<Motor>& motors, bool simulate, int selected) {
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
            if (selected >= 0 && i != static_cast<size_t>(selected)) continue;
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
    double target = 0, force = 0, speed = 0;
    bool force_mode = true;
    bool target_changed = false;
    long steps = 0, tightening_steps = 0;
};

class Controller {
    static constexpr auto sample_age = 1s;
    static constexpr auto edge_guard = 10us;
    const std::vector<Motor> motors;
    const double max_force, max_rate, deadband, alpha;
    const int selected;
    const long tightening_limit;
    std::array<Channel, 6> channels;
    std::unique_ptr<Gpio> gpio;
    std::thread acquisition;
    int receiver = -1;
    std::atomic<bool> stopping{false};
    std::atomic<const char*> fault{nullptr};
    std::mutex lifecycle;
    // Protected by each channel's mutex. A snapshot/command locks all channels.
    Clock::time_point expires{};

    bool enabled(size_t i) const {
        return selected < 0 || i == static_cast<size_t>(selected);
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
                            if (reading.force >= max_force) fail("Measured tension reached the configured limit");
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
        Clock::time_point sample{}, last_step{}, dir_ready{};
        double integral = 0, previous_error = 0, filtered = 0, output = 0;
        int active_direction = 0, pin_direction = 0;
        std::unique_lock lock(channel.mutex);
        while (!stopping) {
            const auto now = Clock::now();
            const auto reading = channel.reading;
            if (expires != Clock::time_point{} && now >= expires) {
                fail("Motion command expired");
                break;
            }
            if (channel.target_changed) {
                output = 0;
                integral = 0;
                active_direction = 0;
                channel.target_changed = false;
            }
            const bool fresh = reading.stamp != Clock::time_point{} && now - reading.stamp < sample_age;
            const bool new_sample = fresh && reading.stamp != sample;
            if (new_sample) {
                const double dt = sample == Clock::time_point{} ? 0 :
                    std::chrono::duration<double>(reading.stamp - sample).count();
                filtered = dt <= 0 ? reading.force : filtered + alpha * (reading.force - filtered);
                sample = reading.stamp;
                channel.force = filtered;
                if (channel.force_mode && expires != Clock::time_point{}) {
                    const double measured = std::max(0.0, filtered);
                    const double error = channel.target - measured;
                    if (active_direction && active_direction * error <= deadband / 2) {
                        active_direction = 0;
                        integral = 0;
                        output = 0;
                    } else {
                        if (!active_direction && std::abs(error) > deadband) {
                            active_direction = error > 0 ? 1 : -1;
                            previous_error = error;
                        }
                        if (active_direction) {
                            if (measured == 0) integral = std::max(0.0, integral);
                            const double candidate = integral + error * dt;
                            const double derivative = dt > 0 ? (error - previous_error) / dt : 0;
                            previous_error = error;
                            const double proposed = kp * error + ki * candidate + kd * derivative;
                            if (!std::isfinite(proposed)) {
                                fail("Non-finite PID output");
                                break;
                            }
                            if (std::abs(proposed) <= max_rate) integral = candidate;
                            output = proposed * active_direction > 0 ?
                                std::clamp(proposed, -max_rate, max_rate) : 0;
                        }
                    }
                }
            }
            if (!fresh && expires != Clock::time_point{}) {
                fail("HX711 reading missing or stale");
                break;
            }
            if (expires == Clock::time_point{}) {
                channel.wake.wait_for(lock, 5ms);
                continue;
            }
            const double remaining = channel.target - channel.steps;
            const double speed = channel.force_mode ? output :
                std::clamp(remaining / 0.065, -max_rate, max_rate);
            const int direction = speed > 0 ? 1 : -1;
            auto next = std::min(expires, reading.stamp + sample_age);
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
               double band, double smoothing, bool simulate, int motor, long max_tightening_steps)
        : motors(std::move(config)), max_force(force), max_rate(rate),
          deadband(band / 1000), alpha(smoothing),
          selected(motor), tightening_limit(max_tightening_steps) {
        if (motors.size() != channels.size() || port < 1 || port > 65535 ||
            !std::isfinite(force) || force <= 0 || !(rate > 0 && rate <= 2133) ||
            !std::isfinite(band) || band < 0 || !(alpha > 0 && alpha <= 1) ||
            selected < -1 || selected >= 6 || tightening_limit < 0)
            throw std::invalid_argument("Invalid controller configuration");
        uint32_t used = 0;
        for (auto [step, dir, kp, ki, kd, low, high] : motors) {
            for (int pin : {step, dir}) {
                if (pin < 0 || pin > 27 || (used & (1u << pin)))
                    throw std::invalid_argument("Invalid or duplicated motor pin");
                used |= 1u << pin;
            }
            for (double gain : {kp, ki, kd})
                if (!std::isfinite(gain) || gain < 0) throw std::invalid_argument("Invalid PID gain");
            if (low > 0 || high < 0 || low >= high)
                throw std::invalid_argument("Invalid step bounds");
        }
        gpio = std::make_unique<Gpio>(motors, simulate, selected);
        receiver = socket(AF_INET, SOCK_DGRAM, 0);
        if (receiver < 0) throw std::runtime_error("Cannot open HX711 socket");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(port);
        try {
            if (connect(receiver, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
                throw std::runtime_error("Cannot connect HX711 socket");
            acquisition = std::thread(&Controller::receive, this);
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
        const auto now = Clock::now();
        for (size_t i = 0; i < channels.size(); ++i) {
            const auto stamp = channels[i].reading.stamp;
            if (enabled(i) && (stamp == Clock::time_point{} || now - stamp >= sample_age))
                return false;
        }
        return true;
    }

    void command(const std::array<double, 6>& targets, bool force_mode, double ttl) {
        if (!(ttl > 0 && ttl <= 0.25)) throw std::invalid_argument("Invalid command lifetime");
        for (size_t i = 0; i < targets.size(); ++i) {
            if (!enabled(i)) continue;
            const auto target = targets[i];
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
            const auto stamp = channels[i].reading.stamp;
            if (enabled(i) && (stamp == Clock::time_point{} || now - stamp >= sample_age))
                throw std::runtime_error("HX711 reading missing or stale");
        }
        expires = now + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(ttl));
        for (size_t i = 0; i < channels.size(); ++i) {
            channels[i].target_changed |= channels[i].force_mode != force_mode ||
                (force_mode && channels[i].target != targets[i]);
            channels[i].target = targets[i];
            channels[i].force_mode = force_mode;
            channels[i].wake.notify_one();
        }
    }

    long tightening_steps(size_t motor) {
        auto& channel = channels.at(motor);
        std::lock_guard lock(channel.mutex);
        return channel.tightening_steps;
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
        .def(py::init<std::vector<Motor>, int, double, double, double, double, bool, int, long>(),
             py::arg("motors"), py::arg("port"), py::arg("max_force"), py::arg("max_rate"),
             py::arg("deadband_g"), py::arg("filter_alpha"), py::arg("simulate") = false,
             py::arg("motor") = -1, py::arg("max_tightening_steps") = 0)
        .def("ready", &Controller::ready, py::call_guard<py::gil_scoped_release>())
        .def("command", &Controller::command, py::call_guard<py::gil_scoped_release>())
        .def("tightening_steps", &Controller::tightening_steps, py::call_guard<py::gil_scoped_release>())
        .def("status", &Controller::status, py::call_guard<py::gil_scoped_release>())
        .def("stop", &Controller::stop, py::call_guard<py::gil_scoped_release>());
}
