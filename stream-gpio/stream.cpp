#include <boost/asio.hpp>
#include <boost/beast/websocket.hpp>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string_view>
#include <system_error>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace ws = beast::websocket;
using asio::ip::tcp;
using namespace std::chrono_literals;

class GpioLevels {
public:
    GpioLevels() {
        std::ifstream model("/proc/device-tree/model");
        std::string name;
        if (!std::getline(model, name) || name.find("Raspberry Pi 4 Model B") != 0)
            throw std::runtime_error("This GPIO sampler requires a Raspberry Pi 4 Model B");

        int fd = open("/dev/gpiomem", O_RDONLY | O_CLOEXEC);
        if (fd < 0)
            throw std::system_error(errno, std::generic_category(), "open /dev/gpiomem");
        mapping_ = mmap(nullptr, 4096, PROT_READ, MAP_SHARED, fd, 0);
        int map_error = errno;
        close(fd);
        if (mapping_ == MAP_FAILED)
            throw std::system_error(map_error, std::generic_category(), "mmap /dev/gpiomem");
    }

    ~GpioLevels() { munmap(mapping_, 4096); }
    GpioLevels(const GpioLevels&) = delete;
    GpioLevels& operator=(const GpioLevels&) = delete;

    uint32_t read() const {
        // BCM2711 GPLEV0 reports the observed levels of GPIO 0 through 31.
        constexpr auto gplev0 = 0x34 / sizeof(uint32_t);
        return static_cast<volatile const uint32_t*>(mapping_)[gplev0] & 0x0fffffff;
    }

private:
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
    const char* usage = "Usage: ./run-stream [-p PORT]\n";
    unsigned port = 5050;
    if (argc == 2 && std::string_view(argv[1]) == "--help") {
        std::cout << usage;
        return 0;
    }
    if (argc != 1) {
        if (argc != 3 || std::string_view(argv[1]) != "-p") {
            std::cerr << usage;
            return 2;
        }
        std::string_view value(argv[2]);
        auto end = value.data() + value.size();
        auto result = std::from_chars(value.data(), end, port);
        if (result.ec != std::errc{} || result.ptr != end || port == 0 || port > 65535) {
            std::cerr << usage;
            return 2;
        }
    }

    try {
        GpioLevels gpio;
        std::string snapshot;
        asio::io_context io;
        tcp::acceptor acceptor(io, {asio::ip::address_v4::loopback(), static_cast<unsigned short>(port)});
        asio::steady_timer timer(io);
        asio::signal_set signals(io, SIGINT, SIGTERM);
        signals.async_wait([&](beast::error_code error, int) { if (!error) io.stop(); });

        std::function<void()> sample = [&] {
            const auto levels = gpio.read();
            const auto now = std::chrono::system_clock::now().time_since_epoch();
            const auto unix_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
            snapshot = "{\"unix_ms\":" + std::to_string(unix_ms) +
                       ",\"levels\":" + std::to_string(levels) + "}";
            timer.expires_after(50ms);
            timer.async_wait([&](beast::error_code error) { if (!error) sample(); });
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

        sample();
        accept();
        std::cout << "Streaming GPIO on ws://127.0.0.1:" << port << std::endl;
        io.run();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
