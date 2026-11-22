#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// Raspberry Pi memory layout
// https://forums.raspberrypi.com/viewtopic.php?t=316984

class GpioRegisters {
public:
    explicit GpioRegisters(bool writable) {
        std::ifstream model("/proc/device-tree/model");
        std::string name;
        if (!std::getline(model, name) || name.find("Raspberry Pi 4 Model B") != 0)
            throw std::runtime_error("This GPIO sampler requires a Raspberry Pi 4 Model B");

        int fd = open("/dev/gpiomem", (writable ? O_RDWR : O_RDONLY) | O_CLOEXEC);
        if (fd < 0)
            throw std::system_error(errno, std::generic_category(), "open /dev/gpiomem");
        mapping_ = mmap(nullptr, gpio_mapping_size, PROT_READ | (writable ? PROT_WRITE : 0),
                        MAP_SHARED, fd, 0);
        int map_error = errno;
        close(fd);
        if (mapping_ == MAP_FAILED)
            throw std::system_error(map_error, std::generic_category(), "mmap /dev/gpiomem");
    }

    ~GpioRegisters() { munmap(mapping_, gpio_mapping_size); }

    GpioRegisters(const GpioRegisters&) = delete;
    GpioRegisters& operator=(const GpioRegisters&) = delete;

    uint32_t levels() const {
        // BCM2711 GPLEV0 reports the observed levels of GPIO 0 through 31.
        constexpr auto gplev0 = 0x34 / sizeof(uint32_t);
        return static_cast<volatile const uint32_t*>(mapping_)[gplev0] & 0x0fffffff;
    }

    uint32_t mode(unsigned pin) { return (reg((pin / 10) * 4) >> ((pin % 10) * 3)) & 7u; }

    void set_mode(unsigned pin, uint32_t value) {
        auto& select = reg((pin / 10) * 4);
        const unsigned shift = (pin % 10) * 3;
        select = (select & ~(7u << shift)) | (value << shift);
    }

    void write(unsigned pin, bool high) { reg(high ? 0x1c : 0x28) = 1u << pin; }

private:
    volatile uint32_t& reg(unsigned offset) {
        return static_cast<volatile uint32_t*>(mapping_)[offset / sizeof(uint32_t)];
    }

    static constexpr std::size_t gpio_mapping_size = 4096;
    void* mapping_;
};
