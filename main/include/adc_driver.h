#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include "esp_err.h"
#include "ext_spi_adc.h"

class AdcDriver {
public:
    using Channel = ext::spi_adc::Channel;
    using Config = ext::spi_adc::Config;
    using ChannelConfig = ext::spi_adc::ChannelConfig;
    using Frame = ext::spi_adc::Frame;

    AdcDriver() = default;
    ~AdcDriver();

    AdcDriver(const AdcDriver &) = delete;
    AdcDriver &operator=(const AdcDriver &) = delete;
    AdcDriver(AdcDriver &&) = delete;
    AdcDriver &operator=(AdcDriver &&) = delete;

    esp_err_t init(const Config &config);
    esp_err_t config_channel(Channel channel);
    esp_err_t config_channel(Channel channel, const ChannelConfig &config);
    esp_err_t read(Channel channel, int *out_raw) const;
    esp_err_t read_frame(Frame *out_frame) const;
    esp_err_t deinit();

    [[nodiscard]] bool initialized() const { return initialized_; }

private:
    using CommandBytes = std::array<std::uint8_t, 2>;

    void normalize_config();
    void reset_channel_commands();
    [[nodiscard]] bool is_valid_channel(Channel channel) const;

    esp_err_t validate_config() const;
    esp_err_t initialize_bus_if_needed();
    esp_err_t add_spi_device();
    esp_err_t remove_spi_device();
    esp_err_t free_bus_if_owned();

    esp_err_t read_frame_locked(Frame &out_frame) const;
    esp_err_t transmit_command(const CommandBytes &command, std::uint16_t *out_sample) const;
    [[nodiscard]] std::uint16_t decode_sample(const CommandBytes &rx_data) const;

    Config config_{};
    mutable std::mutex mutex_;
    spi_device_handle_t spi_handle_ = nullptr;
    bool bus_initialized_ = false;
    bool initialized_ = false;
    std::array<bool, ext::spi_adc::kMaxChannels> configured_{};
    std::array<CommandBytes, ext::spi_adc::kMaxChannels> channel_commands_{};
};
