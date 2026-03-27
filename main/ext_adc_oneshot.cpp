#include "ext_adc_oneshot.h"

#include <memory>
#include <new>

namespace {

class ExtAdcOneShotUnit {
public:
    explicit ExtAdcOneShotUnit(ext_spi_adc_handle_t driver, uint8_t channel_count)
        : driver_(driver), channel_count_(channel_count) {}

    ~ExtAdcOneShotUnit() {
        if (driver_ != nullptr) {
            ext_spi_adc_del(driver_);
        }
    }

    esp_err_t config_channel(ext_adc_channel_t channel,
                             const ext_adc_oneshot_chan_cfg_t *config) {
        if (channel >= channel_count_) {
            return ESP_ERR_INVALID_ARG;
        }

        esp_err_t ret = ESP_OK;
        if (config == nullptr) {
            ret = ext_spi_adc_config_channel(driver_, channel, nullptr);
        } else {
            ext_spi_adc_channel_config_t low_level_config = {
                .tx_data = {
                    config->tx_data[0],
                    config->tx_data[1],
                },
            };
            ret = ext_spi_adc_config_channel(driver_, channel, &low_level_config);
        }
        if (ret == ESP_OK) {
            configured_[channel] = true;
        }

        return ret;
    }

    esp_err_t read(ext_adc_channel_t channel, int *out_raw) const {
        if (channel >= channel_count_) {
            return ESP_ERR_INVALID_ARG;
        }

        if (!configured_[channel]) {
            return ESP_ERR_INVALID_STATE;
        }

        return ext_spi_adc_read_channel(driver_, channel, out_raw);
    }

    esp_err_t read_frame(ext_spi_adc_frame_t *out_frame) const {
        return ext_spi_adc_read_frame(driver_, out_frame);
    }

    esp_err_t shutdown() {
        if (driver_ == nullptr) {
            return ESP_OK;
        }

        esp_err_t ret = ext_spi_adc_del(driver_);
        driver_ = nullptr;
        return ret;
    }

private:
    ext_spi_adc_handle_t driver_ = nullptr;
    uint8_t channel_count_ = 0;
    bool configured_[EXT_SPI_ADC_MAX_CHANNELS] = {};
};

}  // namespace

struct ext_adc_oneshot_unit_t {
    std::unique_ptr<ExtAdcOneShotUnit> impl;
};

extern "C" {

esp_err_t ext_adc_oneshot_new_unit(const ext_adc_oneshot_unit_init_cfg_t *init_config,
                                   ext_adc_oneshot_unit_handle_t *ret_unit) {
    if (init_config == nullptr || ret_unit == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    ext_spi_adc_handle_t driver = nullptr;
    esp_err_t ret = ext_spi_adc_new(&init_config->driver_config, &driver);
    if (ret != ESP_OK) {
        return ret;
    }

    auto *unit = new (std::nothrow) ext_adc_oneshot_unit_t;
    if (unit == nullptr) {
        ext_spi_adc_del(driver);
        return ESP_ERR_NO_MEM;
    }

    unit->impl = std::unique_ptr<ExtAdcOneShotUnit>(new (std::nothrow) ExtAdcOneShotUnit(
        driver, init_config->driver_config.protocol.channel_count));
    if (!unit->impl) {
        delete unit;
        ext_spi_adc_del(driver);
        return ESP_ERR_NO_MEM;
    }

    *ret_unit = unit;
    return ESP_OK;
}

esp_err_t ext_adc_oneshot_config_channel(ext_adc_oneshot_unit_handle_t unit,
                                         ext_adc_channel_t channel,
                                         const ext_adc_oneshot_chan_cfg_t *config) {
    if (unit == nullptr || unit->impl == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    return unit->impl->config_channel(channel, config);
}

esp_err_t ext_adc_oneshot_read(ext_adc_oneshot_unit_handle_t unit,
                               ext_adc_channel_t channel,
                               int *out_raw) {
    if (unit == nullptr || unit->impl == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    return unit->impl->read(channel, out_raw);
}

esp_err_t ext_adc_oneshot_read_frame(ext_adc_oneshot_unit_handle_t unit,
                                     ext_spi_adc_frame_t *out_frame) {
    if (unit == nullptr || unit->impl == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    return unit->impl->read_frame(out_frame);
}

esp_err_t ext_adc_oneshot_del_unit(ext_adc_oneshot_unit_handle_t unit) {
    if (unit == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    if (unit->impl) {
        ret = unit->impl->shutdown();
    }

    delete unit;
    return ret;
}

}  // extern "C"
