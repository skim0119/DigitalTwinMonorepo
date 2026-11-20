#include <boost/asio.hpp>
#include <boost/beast/websocket.hpp>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string_view>
#include <system_error>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace ws = beast::websocket;
using asio::ip::tcp;
using asio::ip::udp;
using namespace std::chrono_literals;

class GpioLevels {
public:
    explicit GpioLevels(bool hx711) : hx711_(hx711) {
        std::ifstream model("/proc/device-tree/model");
        std::string name;
        if (!std::getline(model, name) || name.find("Raspberry Pi 4 Model B") != 0)
            throw std::runtime_error("This GPIO sampler requires a Raspberry Pi 4 Model B");

        int fd = open("/dev/gpiomem", (hx711 ? O_RDWR : O_RDONLY) | O_CLOEXEC);
        if (fd < 0)
            throw std::system_error(errno, std::generic_category(), "open /dev/gpiomem");
        mapping_ = mmap(nullptr, 4096, PROT_READ | (hx711 ? PROT_WRITE : 0), MAP_SHARED, fd, 0);
        int map_error = errno;
        close(fd);
        if (mapping_ == MAP_FAILED)
            throw std::system_error(map_error, std::generic_category(), "mmap /dev/gpiomem");
        if (hx711_) {
            for (std::size_t i = 0; i < clock.size(); ++i) {
                old_data_mode_[i] = mode(data[i]);
                old_clock_mode_[i] = mode(clock[i]);
                set_mode(data[i], 0);
                write_clock(i, false);
                set_mode(clock[i], 1);
                write_clock(i, true);
                std::this_thread::sleep_for(80us);
                write_clock(i, false);
            }
        }
    }

    ~GpioLevels() {
        if (hx711_) {
            for (std::size_t i = 0; i < clock.size(); ++i) {
                write_clock(i, false);
                set_mode(clock[i], old_clock_mode_[i]);
                set_mode(data[i], old_data_mode_[i]);
            }
        }
        munmap(mapping_, 4096);
    }
    GpioLevels(const GpioLevels&) = delete;
    GpioLevels& operator=(const GpioLevels&) = delete;

    uint32_t read() const {
        // BCM2711 GPLEV0 reports the observed levels of GPIO 0 through 31.
        constexpr auto gplev0 = 0x34 / sizeof(uint32_t);
        return static_cast<volatile const uint32_t*>(mapping_)[gplev0] & 0x0fffffff;
    }

    bool ready(std::size_t motor) const {
        return !(read() & (1u << data[motor]));
    }

    bool conversion(std::size_t motor, int32_t& raw) {
        uint32_t bits = 0;
        bool timely = true;
        for (int pulse = 0; pulse < 27; ++pulse) {
            const auto high = std::chrono::steady_clock::now();
            write_clock(motor, true);
            const auto raised = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - raised < 1us) {}
            const bool bit = !ready(motor);
            write_clock(motor, false);
            const auto low = std::chrono::steady_clock::now();
            if (low - high >= 50us) timely = false;
            if (pulse < 24) bits = (bits << 1) | bit;
            while (std::chrono::steady_clock::now() - low < 1us) {}
        }
        // 27 pulses select channel A at gain 64 for the next conversion.
        raw = static_cast<int32_t>(bits) - ((bits & 0x800000) ? 0x1000000 : 0);
        return timely && !ready(motor);
    }

private:
    static constexpr std::array<unsigned, 6> data{2, 5, 7, 21, 17, 12};
    static constexpr std::array<unsigned, 6> clock{3, 6, 11, 13, 20, 19};

    volatile uint32_t& reg(unsigned offset) {
        return static_cast<volatile uint32_t*>(mapping_)[offset / sizeof(uint32_t)];
    }

    uint32_t mode(unsigned pin) {
        return (reg((pin / 10) * 4) >> ((pin % 10) * 3)) & 7u;
    }

    void set_mode(unsigned pin, uint32_t value) {
        auto& select = reg((pin / 10) * 4);
        const unsigned shift = (pin % 10) * 3;
        select = (select & ~(7u << shift)) | (value << shift);
    }

    void write_clock(std::size_t motor, bool high) {
        reg(high ? 0x1c : 0x28) = 1u << clock[motor];
    }

    bool hx711_;
    std::array<uint32_t, 6> old_data_mode_{}, old_clock_mode_{};
    void* mapping_;
};

class Session : public std::enable_shared_from_this<Session> {
public:
    Session(tcp::socket socket, const std::string& snapshot)
        : socket_(std::move(socket)), timer_(socket_.get_executor()), snapshot_(snapshot) {}

    void start() {
        auto timeout = ws::stream_base::timeout::suggested(beast::role_type::server);
        timeout.handshake_timeout = 5s;
        timeout.idle_timeout = 10s;
        socket_.set_option(timeout);
        socket_.read_message_max(1024);
        socket_.async_accept([self = shared_from_this()](beast::error_code error) {
            if (!error) {
                self->read();
                self->send();
            }
        });
    }

private:
    void stop() {
        timer_.cancel();
        beast::error_code ignored;
        socket_.next_layer().close(ignored);
    }

    void read() {
        // Keep a read pending so Beast can reply to WebSocket ping frames.
        socket_.async_read(incoming_, [self = shared_from_this()](beast::error_code error, std::size_t) {
            if (error) return self->stop();
            self->incoming_.consume(self->incoming_.size());
            self->read();
        });
    }

    void send() {
        frame_ = snapshot_;
        timer_.expires_after(2s);
        timer_.async_wait([self = shared_from_this()](beast::error_code error) {
            if (!error) self->stop();
        });
        socket_.async_write(asio::buffer(frame_), [self = shared_from_this()](beast::error_code error, std::size_t) {
            if (error) return self->stop();
            self->timer_.expires_after(50ms);
            self->timer_.async_wait([self](beast::error_code expired) {
                if (!expired) self->send();
            });
        });
    }

    ws::stream<tcp::socket> socket_;
    asio::steady_timer timer_;
    beast::flat_buffer incoming_;
    const std::string& snapshot_;
    std::string frame_;
};

int main(int argc, char** argv) {
    const char* usage = "Usage: ./stream-gpio [--hx711] [-p PORT]\n";
    unsigned port = 5050;
    bool hx711 = false;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--help") {
            std::cout << usage;
            return 0;
        }
        if (arg == "--hx711") {
            hx711 = true;
        } else if (arg == "-p" && i + 1 < argc) {
            std::string_view value(argv[++i]);
            auto end = value.data() + value.size();
            auto result = std::from_chars(value.data(), end, port);
            if (result.ec == std::errc{} && result.ptr == end && port > 0 && port <= 65535)
                continue;
            std::cerr << usage;
            return 2;
        } else {
            std::cerr << usage;
            return 2;
        }
    }

    try {
        std::string snapshot;
        asio::io_context io;
        tcp::acceptor acceptor(io, {asio::ip::address_v4::loopback(), static_cast<unsigned short>(port)});
        udp::socket telemetry(io, {asio::ip::address_v4::loopback(), static_cast<unsigned short>(port)});
        telemetry.non_blocking(true);
        GpioLevels gpio(hx711);
        udp::endpoint sender;
        std::array<char, 128> packet;
        struct Reading {
            double raw;
            double kg;
            std::chrono::steady_clock::time_point received;
        };
        std::array<Reading, 6> readings{};
        asio::steady_timer hx_timer(io);
        struct Average {
            int32_t sum = 0;
            int count = 0;
            bool gain_selected = false;
        };
        std::array<Average, 6> averages{};
        constexpr std::array<double, 6> offsets{170539, -285116.4, 276320, 10569, 236592, -170491};
        constexpr std::array<double, 6> factors{34170, 34170, 39293.8, 34824, 33760, 31950};
        std::function<void()> read_load_cells = [&] {
            for (std::size_t motor = 0; motor < readings.size(); ++motor) {
                if (!gpio.ready(motor)) continue;
                auto& average = averages[motor];
                int32_t raw;
                if (!gpio.conversion(motor, raw)) {
                    average = {};
                    readings[motor] = {};
                    continue;
                }
                if (!average.gain_selected) {
                    average.gain_selected = true;
                    continue;
                }
                average.sum += raw;
                if (++average.count == 5) {
                    const double mean = average.sum / 5.0;
                    readings[motor] = {mean, (mean - offsets[motor]) / factors[motor],
                                       std::chrono::steady_clock::now()};
                    average.sum = 0;
                    average.count = 0;
                }
            }
            hx_timer.expires_after(1ms);
            hx_timer.async_wait([&](beast::error_code error) { if (!error) read_load_cells(); });
        };
        asio::steady_timer timer(io);
        asio::signal_set signals(io, SIGINT, SIGTERM);
        signals.async_wait([&](beast::error_code error, int) { if (!error) io.stop(); });

        std::function<void()> sample = [&] {
            const auto levels = gpio.read();
            const auto now = std::chrono::system_clock::now().time_since_epoch();
            const auto unix_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
            const auto steady_now = std::chrono::steady_clock::now();
            std::ostringstream frame;
            frame.precision(std::numeric_limits<double>::max_digits10);
            frame << "{\"unix_ms\":" << unix_ms << ",\"levels\":" << levels << ",\"hx711\":[";
            for (std::size_t motor = 0; motor < readings.size(); ++motor) {
                if (motor != 0) frame << ',';
                const auto& reading = readings[motor];
                if (reading.received == std::chrono::steady_clock::time_point{} ||
                    steady_now - reading.received > 2s) {
                    frame << "null";
                } else {
                    frame << "{\"raw\":" << reading.raw << ",\"kg\":" << reading.kg << '}';
                }
            }
            frame << "]}";
            snapshot = frame.str();
            timer.expires_after(50ms);
            timer.async_wait([&](beast::error_code error) { if (!error) sample(); });
        };
        std::function<void()> receive = [&] {
            telemetry.async_receive_from(asio::buffer(packet), sender,
                [&](beast::error_code error, std::size_t size) {
                    if (!error && size < packet.size()) {
                        if (hx711 && std::string_view(packet.data(), size) == "hx711") {
                            std::ostringstream response;
                            response.precision(std::numeric_limits<double>::max_digits10);
                            for (const auto& reading : readings) {
                                const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    reading.received.time_since_epoch()).count();
                                response << ns << ' ' << reading.kg << '\n';
                            }
                            const auto payload = response.str();
                            beast::error_code ignored;
                            telemetry.send_to(asio::buffer(payload), sender, 0, ignored);
                        } else if (!hx711) {
                            std::istringstream input(std::string(packet.data(), size));
                            std::size_t motor;
                            double raw, kg;
                            if (input >> motor >> raw >> kg && motor < readings.size() &&
                                std::isfinite(raw) && std::isfinite(kg) && (input >> std::ws).eof()) {
                                readings[motor] = {raw, kg, std::chrono::steady_clock::now()};
                            }
                        }
                    }
                    if (error != asio::error::operation_aborted) receive();
                });
        };
        std::function<void()> accept = [&] {
            acceptor.async_accept([&](beast::error_code error, tcp::socket socket) {
                if (error) {
                    std::cerr << "Accept failed: " << error.message() << '\n';
                    io.stop();
                    return;
                }
                std::make_shared<Session>(std::move(socket), snapshot)->start();
                accept();
            });
        };

        if (hx711) read_load_cells();
        sample();
        receive();
        accept();
        std::cout << "Streaming GPIO on ws://127.0.0.1:" << port << std::endl;
        io.run();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
