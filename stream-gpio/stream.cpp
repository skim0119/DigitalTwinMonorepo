#include "gpio.hpp"
#include "hx711.hpp"

#include <charconv>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string_view>
#include <system_error>

#include <boost/asio.hpp>
#include <boost/beast/websocket.hpp>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace ws = beast::websocket;
using asio::ip::tcp;
using asio::ip::udp;
using namespace std::chrono_literals;

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
        socket_.async_read(incoming_,
                           [self = shared_from_this()](beast::error_code error, std::size_t) {
                               if (error)
                                   return self->stop();
                               self->incoming_.consume(self->incoming_.size());
                               self->read();
                           });
    }

    void send() {
        frame_ = snapshot_;
        timer_.expires_after(2s);
        timer_.async_wait([self = shared_from_this()](beast::error_code error) {
            if (!error)
                self->stop();
        });
        socket_.async_write(asio::buffer(frame_),
                            [self = shared_from_this()](beast::error_code error, std::size_t) {
                                if (error)
                                    return self->stop();
                                self->timer_.expires_after(50ms);
                                self->timer_.async_wait([self](beast::error_code expired) {
                                    if (!expired)
                                        self->send();
                                });
                            });
    }

    ws::stream<tcp::socket> socket_;
    asio::steady_timer timer_;
    beast::flat_buffer incoming_;
    const std::string& snapshot_;
    std::string frame_;
};

std::string format_snapshot(uint32_t levels, const Hx711* hx711) {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto unix_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    std::ostringstream frame;
    frame.precision(std::numeric_limits<double>::max_digits10);
    frame << "{\"unix_ms\":" << unix_ms << ",\"levels\":" << levels;
    if (hx711) {
        const auto steady_now = std::chrono::steady_clock::now();
        const auto& readings = hx711->averaged_readings();
        frame << ",\"hx711\":[";
        for (std::size_t motor = 0; motor < readings.size(); ++motor) {
            if (motor != 0)
                frame << ',';
            const auto& reading = readings[motor];
            if (reading.received == std::chrono::steady_clock::time_point{} ||
                steady_now - reading.received > 2s) {
                frame << "null";
            } else {
                frame << "{\"raw\":" << reading.raw << ",\"kg\":" << reading.kg << '}';
            }
        }
        frame << ']';
    }
    frame << '}';
    return frame.str();
}

namespace {
struct Options {
    unsigned short port = 5050;
    bool dry_run = false;
    bool gpio_only = false;
};

std::optional<int> parse_args(int argc, char** argv, Options& options) {
    constexpr auto help_msg = "Usage: ./stream-gpio [--dry-run] [--gpio-only] [-p PORT]\n";
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--help") {
            std::cout << help_msg;
            return 0;
        }
        if (arg == "--dry-run") {
            options.dry_run = true;
            continue;
        }
        if (arg == "--gpio-only") {
            options.gpio_only = true;
            continue;
        }
        // Custom Port
        if (arg == "-p" && i + 1 < argc) {
            std::string_view value(argv[++i]);
            auto end = value.data() + value.size();
            auto result = std::from_chars(value.data(), end, options.port);
            if (result.ec == std::errc{} && result.ptr == end && options.port > 0 &&
                options.port <= 65535)
                continue;
        }
        std::cerr << help_msg;
        return 2;
    }
    return std::nullopt;
}
} // namespace

int main(int argc, char** argv) {
    Options options;
    if (const auto exit_code = parse_args(argc, argv, options))
        return *exit_code;


    try {
        std::string snapshot;
        asio::io_context io;
        tcp::acceptor acceptor(io);
        const auto address = asio::ip::address_v4::loopback();
        const auto listen_port = options.port;
        if (!options.dry_run)
            acceptor = tcp::acceptor(io, {address, listen_port});

        GpioRegisters gpio(!options.gpio_only);
        std::optional<Hx711> hx711;
        if (!options.gpio_only) {
            std::optional<udp::endpoint> endpoint;
            if (!options.dry_run)
                endpoint.emplace(address, listen_port);
            hx711.emplace(gpio, io, endpoint);
        }

        asio::steady_timer timer(io);
        asio::signal_set signals(io, SIGINT, SIGTERM);
        signals.async_wait([&](beast::error_code error, int) {
            if (!error)
                io.stop();
        });

        std::function<void()> sample = [&] {
            snapshot = format_snapshot(gpio.levels(), hx711 ? &*hx711 : nullptr);
            if (options.dry_run)
                std::cout << snapshot << std::endl;
            timer.expires_after(50ms);
            timer.async_wait([&](beast::error_code error) {
                if (!error)
                    sample();
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


        if (hx711)
            hx711->start();

        sample();
        if (!options.dry_run) {
            accept();
            std::cout << "Streaming GPIO on ws://127.0.0.1:" << options.port << std::endl;
        }
        io.run();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
