#pragma once

#include "ext_adc_oneshot.h"

class AdcDevice {
public:
    using Channel = ext_adc_channel_t;
    using UnitConfig = ext_adc_oneshot_unit_init_cfg_t;
    using ChannelConfig = ext_adc_oneshot_chan_cfg_t;
    using Frame = ext_spi_adc_frame_t;

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

    [[nodiscard]] bool initialized() const { return handle_ != nullptr; }
    [[nodiscard]] ext_adc_oneshot_unit_handle_t handle() const { return handle_; }

private:
    ext_adc_oneshot_unit_handle_t handle_ = nullptr;
};

struct AdcDeviceDef {
    const char *name = nullptr;
    AdcDevice::Channel channel = 0;
    int command_msb = -1;
    uint8_t command_lsb = 0;
};

struct AdcDeviceState {
    AdcDeviceDef def;
    int value = 0;
};
