#pragma once

#include <array>
#include <cstdint>
#include "ext_spi_adc.h"

namespace ext::adc_oneshot {

using Channel = spi_adc::Channel;
using ChannelConfig = spi_adc::ChannelConfig;
using Frame = spi_adc::Frame;

struct UnitConfig {
    spi_adc::Config driver_config;
};

class Unit {
public:
    Unit() = default;
    ~Unit() = default;

    Unit(const Unit &) = delete;
    Unit &operator=(const Unit &) = delete;

    Unit(Unit &&) noexcept;
    Unit &operator=(Unit &&) noexcept;

    esp_err_t initialize(const UnitConfig &config);
    esp_err_t config_channel(Channel channel);
    esp_err_t config_channel(Channel channel, const ChannelConfig &config);
    esp_err_t read(Channel channel, int *out_raw) const;
    esp_err_t read_frame(Frame *out_frame) const;
    esp_err_t shutdown();

    [[nodiscard]] bool initialized() const { return driver_.initialized(); }

private:
    spi_adc::Driver driver_;
    std::uint8_t channel_count_ = 0;
    std::array<bool, spi_adc::kMaxChannels> configured_{};
};

}  // namespace ext::adc_oneshot
