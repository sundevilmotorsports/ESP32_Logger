#include "ext_spi_adc.h"

#include <algorithm>
#include <array>
#include <memory>
#include <mutex>
#include <new>

namespace {

using CommandBytes = std::array<std::uint8_t, 2>;

CommandBytes make_default_command(const ext_spi_adc_protocol_config_t &protocol,
                                  ext_adc_channel_t channel) {
    // Preserve the current device protocol: channel select lives in the first
    // byte, matching the legacy driver sequence of tx_data[0] = channel << 3.
    return {
        static_cast<std::uint8_t>(channel << protocol.channel_command_shift),
        0,
    };
}

CommandBytes make_custom_command(const ext_spi_adc_channel_config_t &config) {
    return { config.tx_data[0], config.tx_data[1] };
}

class ExtSpiAdcDevice {
public:
    explicit ExtSpiAdcDevice(ext_spi_adc_config_t config) : config_(config) {
        normalize_config();
        reset_channel_commands();
    }

    ~ExtSpiAdcDevice() {
        shutdown();
    }

    esp_err_t initialize() {
        esp_err_t ret = validate_config();
        if (ret != ESP_OK) {
            return ret;
        }

        ret = initialize_bus_if_needed();
        if (ret != ESP_OK) {
            return ret;
        }

        ret = add_spi_device();
        if (ret != ESP_OK) {
            free_bus_if_owned();
            return ret;
        }

        initialized_ = true;
        return ESP_OK;
    }

    esp_err_t configure_channel(ext_adc_channel_t channel,
                                const ext_spi_adc_channel_config_t *config) {
        if (!initialized_) {
            return ESP_ERR_INVALID_STATE;
        }

        if (!is_valid_channel(channel)) {
            return ESP_ERR_INVALID_ARG;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        if (config == nullptr) {
            channel_commands_[channel] = make_default_command(config_.protocol, channel);
        } else {
            channel_commands_[channel] = make_custom_command(*config);
        }

        return ESP_OK;
    }

    esp_err_t read_channel(ext_adc_channel_t channel, int *out_raw) {
        if (out_raw == nullptr) {
            return ESP_ERR_INVALID_ARG;
        }

        if (!is_valid_channel(channel)) {
            return ESP_ERR_INVALID_ARG;
        }

        if (!initialized_) {
            return ESP_ERR_INVALID_STATE;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        ext_spi_adc_frame_t frame = {};
        esp_err_t ret = read_frame_locked(frame);
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

        std::lock_guard<std::mutex> lock(mutex_);

        return read_frame_locked(*out_frame);
    }

    esp_err_t shutdown() {
        std::lock_guard<std::mutex> lock(mutex_);

        esp_err_t ret = ESP_OK;

        if (spi_handle_ != nullptr) {
            ret = remove_spi_device();
        }

        esp_err_t bus_ret = free_bus_if_owned();
        if (ret == ESP_OK && bus_ret != ESP_OK) {
            ret = bus_ret;
        }

        initialized_ = false;
        return ret;
    }

private:
    void normalize_config() {
        if (config_.protocol.sample_mask == 0) {
            config_.protocol.sample_mask = 0xFFFF;
        }
    }

    void reset_channel_commands() {
        for (ext_adc_channel_t channel = 0; channel < EXT_SPI_ADC_MAX_CHANNELS; ++channel) {
            channel_commands_[channel] = make_default_command(config_.protocol, channel);
        }
    }

    [[nodiscard]] bool is_valid_channel(ext_adc_channel_t channel) const {
        return channel < config_.protocol.channel_count;
    }

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

    esp_err_t initialize_bus_if_needed() {
        if (!config_.bus.initialize_bus) {
            return ESP_OK;
        }

        spi_bus_config_t bus_config = {};
        bus_config.mosi_io_num = config_.bus.mosi_io_num;
        bus_config.miso_io_num = config_.bus.miso_io_num;
        bus_config.sclk_io_num = config_.bus.sclk_io_num;
        bus_config.quadwp_io_num = -1;
        bus_config.quadhd_io_num = -1;
        bus_config.max_transfer_sz = config_.bus.max_transfer_sz > 0 ? config_.bus.max_transfer_sz : 2;

        esp_err_t ret = spi_bus_initialize(config_.bus.host_id, &bus_config, config_.bus.dma_chan);
        if (ret == ESP_OK) {
            bus_initialized_ = true;
        }

        return ret;
    }

    esp_err_t add_spi_device() {
        spi_device_interface_config_t device_config = {};
        device_config.mode = config_.device.spi_mode;
        device_config.clock_speed_hz = config_.device.clock_speed_hz;
        device_config.spics_io_num = config_.device.cs_io_num;
        device_config.queue_size = config_.device.queue_size > 0 ? config_.device.queue_size : 1;

        return spi_bus_add_device(config_.bus.host_id, &device_config, &spi_handle_);
    }

    esp_err_t remove_spi_device() {
        if (spi_handle_ == nullptr) {
            return ESP_OK;
        }

        esp_err_t ret = spi_bus_remove_device(spi_handle_);
        if (ret == ESP_OK) {
            spi_handle_ = nullptr;
        }

        return ret;
    }

    esp_err_t free_bus_if_owned() {
        if (!bus_initialized_) {
            return ESP_OK;
        }

        esp_err_t ret = spi_bus_free(config_.bus.host_id);
        if (ret == ESP_OK) {
            bus_initialized_ = false;
        }

        return ret;
    }

    esp_err_t read_frame_locked(ext_spi_adc_frame_t &out_frame) {
        std::fill(std::begin(out_frame.values), std::end(out_frame.values), 0);
        out_frame.channel_count = config_.protocol.channel_count;

        // The current ADC behaves like a pipelined SAR: each transfer selects the
        // next channel while the returned word belongs to the previous selection.
        // Preserve correctness by always reading a full frame and using a final
        // flush transfer to capture the last channel.
        std::uint16_t raw_sample = 0;
        esp_err_t ret = transmit_command(channel_commands_[0], &raw_sample);
        if (ret != ESP_OK) {
            return ret;
        }

        for (ext_adc_channel_t channel = 1; channel < config_.protocol.channel_count; ++channel) {
            ret = transmit_command(channel_commands_[channel], &raw_sample);
            if (ret != ESP_OK) {
                return ret;
            }

            out_frame.values[channel - 1] = raw_sample;
        }

        ret = transmit_command(channel_commands_[0], &raw_sample);
        if (ret != ESP_OK) {
            return ret;
        }

        out_frame.values[config_.protocol.channel_count - 1] = raw_sample;
        return ESP_OK;
    }

    esp_err_t transmit_command(const CommandBytes &command, std::uint16_t *out_sample) {
        CommandBytes rx_data = {};

        spi_transaction_t transaction = {};
        transaction.length = 16;
        transaction.rxlength = 16;
        transaction.tx_buffer = command.data();
        transaction.rx_buffer = rx_data.data();

        esp_err_t ret = spi_device_transmit(spi_handle_, &transaction);
        if (ret != ESP_OK) {
            return ret;
        }

        if (out_sample != nullptr) {
            *out_sample = decode_sample(rx_data);
        }

        return ESP_OK;
    }

    std::uint16_t decode_sample(const CommandBytes &rx_data) const {
        std::uint16_t sample = (static_cast<std::uint16_t>(rx_data[0]) << 8) | rx_data[1];
        return sample & config_.protocol.sample_mask;
    }

    ext_spi_adc_config_t config_;
    std::mutex mutex_;
    spi_device_handle_t spi_handle_ = nullptr;
    bool bus_initialized_ = false;
    bool initialized_ = false;
    std::array<CommandBytes, EXT_SPI_ADC_MAX_CHANNELS> channel_commands_{};
};

}  // namespace

struct ext_spi_adc_t {
    std::unique_ptr<ExtSpiAdcDevice> impl;
};

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

    esp_err_t ret = handle->impl->initialize();
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

    return handle->impl->configure_channel(channel, config);
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
        handle->impl.reset();
    }

    delete handle;
    return ret;
}
