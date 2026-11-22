#pragma once

#include "gpio.hpp"

#include <array>
#include <chrono>
#include <optional>

#include <boost/asio.hpp>

class Hx711 {
public:
    struct Reading {
        double raw;
        double kg;
        std::chrono::steady_clock::time_point received;
    };
    static constexpr std::size_t motor_count = 6;

    Hx711(GpioRegisters& gpio, boost::asio::io_context& io,
          std::optional<boost::asio::ip::udp::endpoint> endpoint);
    ~Hx711();

    void start();
    const auto& averaged_readings() const { return averaged_readings_; }

private:
    struct MotorState {
        uint32_t old_data_mode = 0;
        uint32_t old_clock_mode = 0;
        int32_t sum = 0;
        int count = 0;
        bool gain_selected = false;
    };

    bool data_is_high(std::size_t motor) const;
    bool conversion(std::size_t motor, int32_t& raw);
    void poll();
    void receive();

    GpioRegisters& gpio_;
    boost::asio::steady_timer timer_;
    boost::asio::ip::udp::socket telemetry_;
    boost::asio::ip::udp::endpoint sender_;
    std::array<char, 128> packet_;
    std::array<MotorState, motor_count> motors_{};
    std::array<Reading, motor_count> averaged_readings_{}, latest_conversions_{};
};
