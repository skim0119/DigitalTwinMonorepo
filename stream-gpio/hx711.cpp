#include "hx711.hpp"

#include <limits>
#include <sstream>
#include <string_view>
#include <thread>

namespace asio = boost::asio;
using asio::ip::udp;
using namespace std::chrono_literals;

// Configuration for sensor calibration and pins
// TODO: move to configuration file at root
namespace {
constexpr std::array<unsigned, Hx711::motor_count> data_pins{2, 5, 7, 21, 17, 12};
constexpr std::array<unsigned, Hx711::motor_count> clock_pins{3, 6, 11, 13, 20, 19};
constexpr std::array<double, Hx711::motor_count> offsets{170539, -285116.4, 276320,
                                                         10569,  236592,    -170491};
constexpr std::array<double, Hx711::motor_count> counts_per_kg{34170, 34170, 39293.8,
                                                               34824, 33760, 31950};
constexpr int conversions_per_average = 5;
} // namespace

Hx711::Hx711(GpioRegisters& gpio, asio::io_context& io, std::optional<udp::endpoint> endpoint)
    : gpio_(gpio), timer_(io), telemetry_(io) {
    if (endpoint) {
        telemetry_ = udp::socket(io, *endpoint);
        telemetry_.non_blocking(true);
    }
    for (std::size_t motor = 0; motor < motor_count; ++motor) {
        auto& state = motors_[motor];
        const auto data = data_pins[motor];
        const auto clock = clock_pins[motor];
        state.old_data_mode = gpio_.mode(data);
        state.old_clock_mode = gpio_.mode(clock);
        gpio_.set_mode(data, 0);
        gpio_.write(clock, false);
        gpio_.set_mode(clock, 1);
        // Hold PD_SCK high long enough to power down the HX711, then reset it by lowering the pin.
        gpio_.write(clock, true);
        std::this_thread::sleep_for(80us);
        gpio_.write(clock, false);
    }
}

Hx711::~Hx711() {
    for (std::size_t motor = 0; motor < motor_count; ++motor) {
        gpio_.write(clock_pins[motor], false);
        gpio_.set_mode(clock_pins[motor], motors_[motor].old_clock_mode);
        gpio_.set_mode(data_pins[motor], motors_[motor].old_data_mode);
    }
}

void Hx711::start() {
    poll();
    if (telemetry_.is_open())
        receive();
}

bool Hx711::data_is_high(std::size_t motor) const {
    return (gpio_.levels() & (1u << data_pins[motor])) != 0;
}

bool Hx711::conversion(std::size_t motor, int32_t& raw) {
    uint32_t bits = 0;
    bool timely = true;
    for (int pulse = 0; pulse < 27; ++pulse) {
        const auto high = std::chrono::steady_clock::now();
        gpio_.write(clock_pins[motor], true);
        const auto raised = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - raised < 1us) {
        }
        const bool bit = data_is_high(motor);
        gpio_.write(clock_pins[motor], false);
        const auto low = std::chrono::steady_clock::now();
        if (low - high >= 50us)
            timely = false;
        if (pulse < 24)
            bits = (bits << 1) | bit;
        while (std::chrono::steady_clock::now() - low < 1us) {
        }
    }
    // 27 pulses select channel A at gain 64 for the next conversion.
    raw = static_cast<int32_t>(bits) - ((bits & 0x800000) ? 0x1000000 : 0);
    return timely && data_is_high(motor);
}

void Hx711::poll() {
    for (std::size_t motor = 0; motor < motor_count; ++motor) {
        if (data_is_high(motor))
            continue;
        auto& state = motors_[motor];
        int32_t raw;
        if (!conversion(motor, raw)) {
            state.sum = 0;
            state.count = 0;
            state.gain_selected = false;
            averaged_readings_[motor] = {};
            latest_conversions_[motor] = {};
            continue;
        }
        // The gain selected by the clock pulses applies to the next conversion.
        if (!state.gain_selected) {
            state.gain_selected = true;
            continue;
        }
        latest_conversions_[motor] = {double(raw), (raw - offsets[motor]) / counts_per_kg[motor],
                                      std::chrono::steady_clock::now()};
        state.sum += raw;
        if (++state.count == conversions_per_average) {
            const double mean = static_cast<double>(state.sum) / conversions_per_average;
            averaged_readings_[motor] = {mean, (mean - offsets[motor]) / counts_per_kg[motor],
                                         std::chrono::steady_clock::now()};
            state.sum = 0;
            state.count = 0;
        }
    }
    timer_.expires_after(1ms);
    timer_.async_wait([this](boost::system::error_code error) {
        if (!error)
            poll();
    });
}

void Hx711::receive() {
    telemetry_.async_receive_from(
        asio::buffer(packet_), sender_, [this](boost::system::error_code error, std::size_t size) {
            if (!error && size < packet_.size()) {
                const std::string_view request(packet_.data(), size);
                if (request == "hx711" || request == "hx711-fast") {
                    const auto& readings =
                        request == "hx711-fast" ? latest_conversions_ : averaged_readings_;
                    std::ostringstream response;
                    response.precision(std::numeric_limits<double>::max_digits10);
                    for (const auto& reading : readings) {
                        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                            reading.received.time_since_epoch())
                                            .count();
                        response << ns << ' ' << reading.kg << '\n';
                    }
                    const auto payload = response.str();
                    boost::system::error_code ignored;
                    telemetry_.send_to(asio::buffer(payload), sender_, 0, ignored);
                }
            }
            if (error != asio::error::operation_aborted)
                receive();
        });
}
