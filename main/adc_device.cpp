#include "adc_device.h"

AdcDevice::~AdcDevice() {
    deinit();
}

AdcDevice::AdcDevice(AdcDevice &&other) noexcept = default;
AdcDevice &AdcDevice::operator=(AdcDevice &&other) noexcept = default;

esp_err_t AdcDevice::init(const UnitConfig &config) {
    return unit_.initialize(config);
}

esp_err_t AdcDevice::config_channel(Channel channel) {
    return unit_.config_channel(channel);
}

esp_err_t AdcDevice::config_channel(Channel channel, const ChannelConfig &config) {
    return unit_.config_channel(channel, config);
}

esp_err_t AdcDevice::read(Channel channel, int *out_raw) const {
    return unit_.read(channel, out_raw);
}

esp_err_t AdcDevice::read_frame(Frame *out_frame) const {
    return unit_.read_frame(out_frame);
}

esp_err_t AdcDevice::deinit() {
    return unit_.shutdown();
}
