#include "ext_spi_adc.h"

#include <array>
#include <mutex>
#include <new>

namespace ext::spi_adc {

namespace {

using CommandBytes = std::array<std::uint8_t, 2>;

CommandBytes make_default_command(const ProtocolConfig &protocol, Channel channel) {
    // Preserve the current device protocol: channel select lives in the first
    // byte, matching the legacy driver sequence of tx_data[0] = channel << 3.
    return {
        static_cast<std::uint8_t>(channel << protocol.channel_command_shift),
        0,
    };
}

CommandBytes make_custom_command(const ChannelConfig &config) {
    return config.tx_data;
}

}  // namespace

class Driver::Impl {
public:
    explicit Impl(Config config) : config_(config) {
        normalize_config();
        reset_channel_commands();
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

    esp_err_t config_channel(Channel channel, const ChannelConfig *config) {
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

    esp_err_t read(Channel channel, int *out_raw) const {
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

        Frame frame;
        esp_err_t ret = read_frame_locked(frame);
        if (ret != ESP_OK) {
            return ret;
        }

        *out_raw = frame.values[channel];
        return ESP_OK;
    }

    esp_err_t read_frame(Frame *out_frame) const {
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

        const esp_err_t bus_ret = free_bus_if_owned();
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
        for (Channel channel = 0; channel < kMaxChannels; ++channel) {
            channel_commands_[channel] = make_default_command(config_.protocol, channel);
        }
    }

    [[nodiscard]] bool is_valid_channel(Channel channel) const {
        return channel < config_.protocol.channel_count;
    }

    esp_err_t validate_config() const {
        if (config_.protocol.channel_count == 0 || config_.protocol.channel_count > kMaxChannels) {
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

    esp_err_t read_frame_locked(Frame &out_frame) const {
        out_frame.values.fill(0);
        out_frame.channel_count = config_.protocol.channel_count;

        // The ADC is pipelined: each transfer selects the next channel while the
        // returned sample belongs to the previous selection. The sequence is:
        // 1. Prime the pipeline with channel 0.
        // 2. Walk across the remaining channels, storing the previous result.
        // 3. Issue one final flush transfer to collect the last channel.
        std::uint16_t raw_sample = 0;
        esp_err_t ret = transmit_command(channel_commands_[0], &raw_sample);
        if (ret != ESP_OK) {
            return ret;
        }

        for (Channel channel = 1; channel < config_.protocol.channel_count; ++channel) {
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

    esp_err_t transmit_command(const CommandBytes &command, std::uint16_t *out_sample) const {
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
        const std::uint16_t sample =
            (static_cast<std::uint16_t>(rx_data[0]) << 8) | rx_data[1];
        return sample & config_.protocol.sample_mask;
    }

    Config config_;
    mutable std::mutex mutex_;
    spi_device_handle_t spi_handle_ = nullptr;
    bool bus_initialized_ = false;
    bool initialized_ = false;
    std::array<CommandBytes, kMaxChannels> channel_commands_{};
};

Driver::Driver() = default;

Driver::~Driver() {
    (void)shutdown();
}

Driver::Driver(Driver &&) noexcept = default;
Driver &Driver::operator=(Driver &&) noexcept = default;

esp_err_t Driver::initialize(const Config &config) {
    if (impl_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    auto impl = std::unique_ptr<Impl>(new (std::nothrow) Impl(config));
    if (!impl) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = impl->initialize();
    if (ret != ESP_OK) {
        return ret;
    }

    impl_ = std::move(impl);
    return ESP_OK;
}

esp_err_t Driver::config_channel(Channel channel) {
    if (!impl_) {
        return ESP_ERR_INVALID_STATE;
    }

    return impl_->config_channel(channel, nullptr);
}

esp_err_t Driver::config_channel(Channel channel, const ChannelConfig &config) {
    if (!impl_) {
        return ESP_ERR_INVALID_STATE;
    }

    return impl_->config_channel(channel, &config);
}

esp_err_t Driver::read(Channel channel, int *out_raw) const {
    if (!impl_) {
        return ESP_ERR_INVALID_STATE;
    }

    return impl_->read(channel, out_raw);
}

esp_err_t Driver::read_frame(Frame *out_frame) const {
    if (!impl_) {
        return ESP_ERR_INVALID_STATE;
    }

    return impl_->read_frame(out_frame);
}

esp_err_t Driver::shutdown() {
    if (!impl_) {
        return ESP_OK;
    }

    esp_err_t ret = impl_->shutdown();
    impl_.reset();
    return ret;
}

}  // namespace ext::spi_adc
