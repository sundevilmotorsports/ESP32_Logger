#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

namespace ext::spi_adc {

inline constexpr std::size_t kMaxChannels = 8;

using Channel = std::uint8_t;

struct BusConfig {
    spi_host_device_t host_id;
    spi_dma_chan_t dma_chan;
    gpio_num_t sclk_io_num;
    gpio_num_t mosi_io_num;
    gpio_num_t miso_io_num;
    int max_transfer_sz;
    bool initialize_bus;
};

struct DeviceConfig {
    gpio_num_t cs_io_num;
    int clock_speed_hz;
    std::uint8_t spi_mode;
    int queue_size;
};

struct ProtocolConfig {
    std::uint8_t channel_count;
    // For the current ADC, the logical channel select bits live in the first
    // transmitted byte and the second byte is zero by default.
    std::uint8_t channel_command_shift;
    std::uint16_t sample_mask;
};

struct Config {
    BusConfig bus;
    DeviceConfig device;
    ProtocolConfig protocol;
};

struct ChannelConfig {
    // Raw 16-bit SPI frame in transmit order: tx_data[0] is shifted out first.
    std::array<std::uint8_t, 2> tx_data{};
};

struct Frame {
    std::uint8_t channel_count = 0;
    std::array<std::uint16_t, kMaxChannels> values{};
};

class Driver {
public:
    Driver();
    ~Driver();

    Driver(const Driver &) = delete;
    Driver &operator=(const Driver &) = delete;

    Driver(Driver &&) noexcept;
    Driver &operator=(Driver &&) noexcept;

    esp_err_t initialize(const Config &config);
    esp_err_t config_channel(Channel channel);
    esp_err_t config_channel(Channel channel, const ChannelConfig &config);
    esp_err_t read(Channel channel, int *out_raw) const;
    esp_err_t read_frame(Frame *out_frame) const;
    esp_err_t shutdown();

    [[nodiscard]] bool initialized() const { return impl_ != nullptr; }

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ext::spi_adc
