#include "ext_adc_oneshot.h"

namespace ext::adc_oneshot {

Unit::Unit(Unit &&) noexcept = default;
Unit &Unit::operator=(Unit &&) noexcept = default;

esp_err_t Unit::initialize(const UnitConfig &config) {
    if (driver_.initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = driver_.initialize(config.driver_config);
    if (ret != ESP_OK) {
        return ret;
    }

    channel_count_ = config.driver_config.protocol.channel_count;
    configured_.fill(false);
    return ESP_OK;
}

esp_err_t Unit::config_channel(Channel channel) {
    if (!driver_.initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= channel_count_) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = driver_.config_channel(channel);
    if (ret == ESP_OK) {
        configured_[channel] = true;
    }

    return ret;
}

esp_err_t Unit::config_channel(Channel channel, const ChannelConfig &config) {
    if (!driver_.initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= channel_count_) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = driver_.config_channel(channel, config);
    if (ret == ESP_OK) {
        configured_[channel] = true;
    }

    return ret;
}

esp_err_t Unit::read(Channel channel, int *out_raw) const {
    if (!driver_.initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= channel_count_) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!configured_[channel]) {
        return ESP_ERR_INVALID_STATE;
    }

    return driver_.read(channel, out_raw);
}

esp_err_t Unit::read_frame(Frame *out_frame) const {
    if (!driver_.initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    return driver_.read_frame(out_frame);
}

esp_err_t Unit::shutdown() {
    if (!driver_.initialized()) {
        return ESP_OK;
    }

    esp_err_t ret = driver_.shutdown();
    configured_.fill(false);
    channel_count_ = 0;
    return ret;
}

}  // namespace ext::adc_oneshot
