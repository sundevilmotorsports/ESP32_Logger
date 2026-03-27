#include "adc_device.h"

#include <utility>

AdcDevice::~AdcDevice() {
    deinit();
}

AdcDevice::AdcDevice(AdcDevice &&other) noexcept : handle_(other.handle_) {
    other.handle_ = nullptr;
}

AdcDevice &AdcDevice::operator=(AdcDevice &&other) noexcept {
    if (this != &other) {
        deinit();
        handle_ = other.handle_;
        other.handle_ = nullptr;
    }

    return *this;
}

esp_err_t AdcDevice::init(const UnitConfig &config) {
    if (handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return ext_adc_oneshot_new_unit(&config, &handle_);
}

esp_err_t AdcDevice::config_channel(Channel channel) {
    if (handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return ext_adc_oneshot_config_channel(handle_, channel, nullptr);
}

esp_err_t AdcDevice::config_channel(Channel channel, const ChannelConfig &config) {
    if (handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return ext_adc_oneshot_config_channel(handle_, channel, &config);
}

esp_err_t AdcDevice::read(Channel channel, int *out_raw) const {
    if (handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return ext_adc_oneshot_read(handle_, channel, out_raw);
}

esp_err_t AdcDevice::read_frame(Frame *out_frame) const {
    if (handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return ext_adc_oneshot_read_frame(handle_, out_frame);
}

esp_err_t AdcDevice::deinit() {
    if (handle_ == nullptr) {
        return ESP_OK;
    }

    esp_err_t ret = ext_adc_oneshot_del_unit(handle_);
    handle_ = nullptr;
    return ret;
}
