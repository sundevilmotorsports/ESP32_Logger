#include "ext_spi_adc.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <new>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {

class FreeRtosMutex {
public:
    FreeRtosMutex() : handle_(xSemaphoreCreateMutex()) {}

    ~FreeRtosMutex() {
        if (handle_ != nullptr) {
            vSemaphoreDelete(handle_);
        }
    }

    FreeRtosMutex(const FreeRtosMutex &) = delete;
    FreeRtosMutex &operator=(const FreeRtosMutex &) = delete;

    [[nodiscard]] bool valid() const {
        return handle_ != nullptr;
    }

    [[nodiscard]] bool lock() const {
        return handle_ != nullptr && xSemaphoreTake(handle_, portMAX_DELAY) == pdTRUE;
    }

    void unlock() const {
        if (handle_ != nullptr) {
            xSemaphoreGive(handle_);
        }
    }

private:
    SemaphoreHandle_t handle_ = nullptr;
};

class ScopedLock {
public:
    explicit ScopedLock(const FreeRtosMutex &mutex) : mutex_(mutex), locked_(mutex_.lock()) {}

    ~ScopedLock() {
        if (locked_) {
            mutex_.unlock();
        }
    }

    [[nodiscard]] bool locked() const {
        return locked_;
    }

private:
    const FreeRtosMutex &mutex_;
    bool locked_ = false;
};

void make_default_command(const ext_spi_adc_protocol_config_t &protocol,
                          ext_adc_channel_t channel,
                          uint8_t out_command[2]) {
    // Preserve the current device protocol: channel select lives in the first
    // byte, matching the legacy driver sequence of tx_data[0] = channel << 3.
    out_command[0] = static_cast<uint8_t>(channel << protocol.channel_command_shift);
    out_command[1] = 0;
}

class ExtSpiAdcDevice {
public:
    explicit ExtSpiAdcDevice(ext_spi_adc_config_t config) : config_(config) {
        if (config_.protocol.sample_mask == 0) {
            config_.protocol.sample_mask = 0xFFFF;
        }

        for (uint8_t channel = 0; channel < EXT_SPI_ADC_MAX_CHANNELS; ++channel) {
            make_default_command(config_.protocol, channel, channel_commands_[channel]);
        }
    }

    ~ExtSpiAdcDevice() {
        shutdown();
    }

    esp_err_t init() {
        if (!mutex_.valid()) {
            return ESP_ERR_NO_MEM;
        }

        esp_err_t ret = validate_config();
        if (ret != ESP_OK) {
            return ret;
        }

        if (config_.bus.initialize_bus) {
            spi_bus_config_t bus_cfg = {};
            bus_cfg.mosi_io_num = config_.bus.mosi_io_num;
            bus_cfg.miso_io_num = config_.bus.miso_io_num;
            bus_cfg.sclk_io_num = config_.bus.sclk_io_num;
            bus_cfg.quadwp_io_num = -1;
            bus_cfg.quadhd_io_num = -1;
            bus_cfg.max_transfer_sz = config_.bus.max_transfer_sz > 0 ? config_.bus.max_transfer_sz : 2;

            ret = spi_bus_initialize(config_.bus.host_id, &bus_cfg, config_.bus.dma_chan);
            if (ret != ESP_OK) {
                return ret;
            }

            bus_initialized_ = true;
        }

        spi_device_interface_config_t dev_cfg = {};
        dev_cfg.mode = config_.device.spi_mode;
        dev_cfg.clock_speed_hz = config_.device.clock_speed_hz;
        dev_cfg.spics_io_num = config_.device.cs_io_num;
        dev_cfg.queue_size = config_.device.queue_size > 0 ? config_.device.queue_size : 1;

        ret = spi_bus_add_device(config_.bus.host_id, &dev_cfg, &spi_handle_);
        if (ret != ESP_OK) {
            cleanup_bus();
            return ret;
        }

        initialized_ = true;
        return ESP_OK;
    }

    esp_err_t config_channel(ext_adc_channel_t channel,
                             const ext_spi_adc_channel_config_t *config) {
        if (!initialized_) {
            return ESP_ERR_INVALID_STATE;
        }

        if (channel >= config_.protocol.channel_count) {
            return ESP_ERR_INVALID_ARG;
        }

        ScopedLock lock(mutex_);
        if (!lock.locked()) {
            return ESP_ERR_TIMEOUT;
        }

        if (config == nullptr) {
            make_default_command(config_.protocol, channel, channel_commands_[channel]);
        } else {
            memcpy(channel_commands_[channel], config->tx_data, sizeof(config->tx_data));
        }

        return ESP_OK;
    }

    esp_err_t read_channel(ext_adc_channel_t channel, int *out_raw) {
        if (out_raw == nullptr) {
            return ESP_ERR_INVALID_ARG;
        }

        if (channel >= config_.protocol.channel_count) {
            return ESP_ERR_INVALID_ARG;
        }

        if (!initialized_) {
            return ESP_ERR_INVALID_STATE;
        }

        ScopedLock lock(mutex_);
        if (!lock.locked()) {
            return ESP_ERR_TIMEOUT;
        }

        ext_spi_adc_frame_t frame = {};
        esp_err_t ret = read_frame_locked(&frame);
        if (ret != ESP_OK) {
            return ret;
        }

        *out_raw = frame.values[channel];
        return ESP_OK;
    }

    esp_err_t read_frame(ext_spi_adc_frame_t *out_frame) {
        if (out_frame == nullptr) {
            return ESP_ERR_INVALID_ARG;
        }

        if (!initialized_) {
            return ESP_ERR_INVALID_STATE;
        }

        ScopedLock lock(mutex_);
        if (!lock.locked()) {
            return ESP_ERR_TIMEOUT;
        }

        return read_frame_locked(out_frame);
    }

    esp_err_t shutdown() {
        esp_err_t ret = ESP_OK;

        if (spi_handle_ != nullptr) {
            ret = spi_bus_remove_device(spi_handle_);
            spi_handle_ = nullptr;
        }

        esp_err_t bus_ret = cleanup_bus();
        if (ret == ESP_OK) {
            ret = bus_ret;
        }

        initialized_ = false;
        return ret;
    }

private:
    esp_err_t validate_config() const {
        if (config_.protocol.channel_count == 0 ||
            config_.protocol.channel_count > EXT_SPI_ADC_MAX_CHANNELS) {
            return ESP_ERR_INVALID_ARG;
        }

        if (config_.device.clock_speed_hz <= 0) {
            return ESP_ERR_INVALID_ARG;
        }

        if (config_.device.cs_io_num < 0) {
            return ESP_ERR_INVALID_ARG;
        }

        if (config_.bus.initialize_bus) {
            if (config_.bus.mosi_io_num < 0 ||
                config_.bus.miso_io_num < 0 ||
                config_.bus.sclk_io_num < 0) {
                return ESP_ERR_INVALID_ARG;
            }
        }

        return ESP_OK;
    }

    esp_err_t read_frame_locked(ext_spi_adc_frame_t *out_frame) {
        std::fill(std::begin(out_frame->values), std::end(out_frame->values), 0);
        out_frame->channel_count = config_.protocol.channel_count;

        // The current ADC behaves like a pipelined SAR: each transfer selects the
        // next channel while the returned word belongs to the previous selection.
        // Preserve correctness by always reading a full frame and using a final
        // flush transfer to capture the last channel.
        uint16_t raw_sample = 0;
        esp_err_t ret = transmit_command(channel_commands_[0], &raw_sample);
        if (ret != ESP_OK) {
            return ret;
        }

        for (uint8_t channel = 1; channel < config_.protocol.channel_count; ++channel) {
            ret = transmit_command(channel_commands_[channel], &raw_sample);
            if (ret != ESP_OK) {
                return ret;
            }

            out_frame->values[channel - 1] = raw_sample;
        }

        ret = transmit_command(channel_commands_[0], &raw_sample);
        if (ret != ESP_OK) {
            return ret;
        }

        out_frame->values[config_.protocol.channel_count - 1] = raw_sample;
        return ESP_OK;
    }

    esp_err_t transmit_command(const uint8_t command[2], uint16_t *out_sample) {
        uint8_t tx_data[2] = {
            command[0],
            command[1],
        };
        uint8_t rx_data[2] = {};

        spi_transaction_t transaction = {};
        transaction.length = 16;
        transaction.rxlength = 16;
        transaction.tx_buffer = tx_data;
        transaction.rx_buffer = rx_data;

        esp_err_t ret = spi_device_transmit(spi_handle_, &transaction);
        if (ret != ESP_OK) {
            return ret;
        }

        if (out_sample != nullptr) {
            *out_sample = decode_sample(rx_data);
        }

        return ESP_OK;
    }

    uint16_t decode_sample(const uint8_t rx_data[2]) const {
        uint16_t sample = (static_cast<uint16_t>(rx_data[0]) << 8) | rx_data[1];
        return sample & config_.protocol.sample_mask;
    }

    esp_err_t cleanup_bus() {
        if (!bus_initialized_) {
            return ESP_OK;
        }

        esp_err_t ret = spi_bus_free(config_.bus.host_id);
        if (ret == ESP_OK) {
            bus_initialized_ = false;
        }

        return ret;
    }

    ext_spi_adc_config_t config_;
    FreeRtosMutex mutex_;
    spi_device_handle_t spi_handle_ = nullptr;
    bool bus_initialized_ = false;
    bool initialized_ = false;
    uint8_t channel_commands_[EXT_SPI_ADC_MAX_CHANNELS][2] = {};
};

}  // namespace

struct ext_spi_adc_t {
    std::unique_ptr<ExtSpiAdcDevice> impl;
};

extern "C" {

esp_err_t ext_spi_adc_new(const ext_spi_adc_config_t *config, ext_spi_adc_handle_t *ret_handle) {
    if (config == nullptr || ret_handle == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    auto *handle = new (std::nothrow) ext_spi_adc_t;
    if (handle == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    handle->impl = std::unique_ptr<ExtSpiAdcDevice>(new (std::nothrow) ExtSpiAdcDevice(*config));
    if (!handle->impl) {
        delete handle;
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = handle->impl->init();
    if (ret != ESP_OK) {
        delete handle;
        return ret;
    }

    *ret_handle = handle;
    return ESP_OK;
}

esp_err_t ext_spi_adc_config_channel(ext_spi_adc_handle_t handle,
                                     ext_adc_channel_t channel,
                                     const ext_spi_adc_channel_config_t *config) {
    if (handle == nullptr || handle->impl == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    return handle->impl->config_channel(channel, config);
}

esp_err_t ext_spi_adc_read_channel(ext_spi_adc_handle_t handle,
                                   ext_adc_channel_t channel,
                                   int *out_raw) {
    if (handle == nullptr || handle->impl == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    return handle->impl->read_channel(channel, out_raw);
}

esp_err_t ext_spi_adc_read_frame(ext_spi_adc_handle_t handle, ext_spi_adc_frame_t *out_frame) {
    if (handle == nullptr || handle->impl == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    return handle->impl->read_frame(out_frame);
}

esp_err_t ext_spi_adc_del(ext_spi_adc_handle_t handle) {
    if (handle == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    if (handle->impl) {
        ret = handle->impl->shutdown();
    }

    delete handle;
    return ret;
}

}  // extern "C"
