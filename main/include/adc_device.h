#pragma once

#include <cstdint>

#include "ext_adc_oneshot.h"

class AdcDevice {
public:
    using Channel = ext::adc_oneshot::Channel;
    using UnitConfig = ext::adc_oneshot::UnitConfig;
    using ChannelConfig = ext::adc_oneshot::ChannelConfig;
    using Frame = ext::adc_oneshot::Frame;

    AdcDevice() = default;
    ~AdcDevice();

    AdcDevice(const AdcDevice &) = delete;
    AdcDevice &operator=(const AdcDevice &) = delete;

    AdcDevice(AdcDevice &&other) noexcept;
    AdcDevice &operator=(AdcDevice &&other) noexcept;

    esp_err_t init(const UnitConfig &config);
    esp_err_t config_channel(Channel channel);
    esp_err_t config_channel(Channel channel, const ChannelConfig &config);
    esp_err_t read(Channel channel, int *out_raw) const;
    esp_err_t read_frame(Frame *out_frame) const;
    esp_err_t deinit();

    [[nodiscard]] bool initialized() const { return unit_.initialized(); }

private:
    ext::adc_oneshot::Unit unit_;
};

struct AdcDeviceDef {
    const char *name = nullptr;
    AdcDevice::Channel channel = 0;
    int command_msb = -1;
    std::uint8_t command_lsb = 0;
};

struct AdcDeviceState {
    AdcDeviceDef def;
    int value = 0;
};
