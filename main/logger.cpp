#include "logger.h"

#include <chrono>
#include <filesystem>

#include "log.h"

Logger::Logger() {
    this->name_ = "tmp";
    // sd_.init();

    this->register_can_device({
        .id = 0x01,
        .signals = {
            { "0first",  0, 2 },
            { "0second", 2, 2 },
            { "0third",  4, 2 },
            { "0fourth", 6, 2 },
        }
    });

    this->register_can_device({
        .id = 0x02,
        .signals = {
            { "large",  0, 8 },
        }
    });
}


void Logger::register_can_device(CanDeviceDef def) {
    can_states_.push_back({ .def = std::move(def) });
}

void Logger::register_adc_device(AdcDeviceDef def) {
    adc_states_.push_back({ .def = def });

    if (!adc_ready_) {
        return;
    }

    esp_err_t ret = apply_adc_channel_config(adc_states_.back().def);
    if (ret != ESP_OK) {
        ModuleCoreLogger::error("Failed to configure ADC channel %u: %s",
                                static_cast<unsigned>(def.channel), esp_err_to_name(ret));
    }
}

void Logger::on_can_frame(const CanFrame *frame) {
    if (frame->header.ide == 0) return;
    ParsedArbId parsed = parse_arb_id(frame->header.id);
    if (parsed.cmd != CMD_DATA) return;

    for (auto &s : can_states_) {
        if (s.def.id != parsed.source) continue;
        memcpy(s.data, frame->data, frame->len);
        s.data_len = frame->len;
        return;
    }
}

void Logger::write_header() {
    char   line[512];
    size_t pos = snprintf(line, sizeof(line), "timestamp");
    for (auto &s : can_states_)
        for (const auto &sig : s.def.signals)
            pos += snprintf(line + pos, sizeof(line) - pos, ",%s", sig.name);
    for (auto &s : adc_states_)
        pos += snprintf(line + pos, sizeof(line) - pos, ",%s", s.def.name);
    line[pos++] = '\n';
    write_log(line, pos);
}

esp_err_t Logger::init_adc(const AdcDevice::UnitConfig &config) {
    if (adc_ready_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = adc_device_.init(config);
    if (ret != ESP_OK) {
        return ret;
    }

    for (const auto &state : adc_states_) {
        ret = apply_adc_channel_config(state.def);
        if (ret != ESP_OK) {
            adc_device_.deinit();
            return ret;
        }
    }

    adc_ready_ = true;
    return ESP_OK;
}

esp_err_t Logger::apply_adc_channel_config(const AdcDeviceDef &def) {
    if (!adc_device_.initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (def.command_msb < 0) {
        return adc_device_.config_channel(def.channel);
    }

    AdcDevice::ChannelConfig config = {
        .tx_data = {
            static_cast<std::uint8_t>(def.command_msb),
            def.command_lsb,
        },
    };
    return adc_device_.config_channel(def.channel, config);
}

// Maybe switch to a template, would need to implement way to define length of bytes to type
uint64_t Logger::extract(const uint8_t *data, uint8_t data_len, const SignalSlice &sig) {
    if (sig.offset + sig.len > data_len) return 0;
    uint64_t raw = 0;
    for (uint8_t i = 0; i < sig.len; i++)
        raw |= static_cast<uint64_t>(data[sig.offset + i]) << (i * 8);
    return raw;
}

void Logger::log_sample() {
    char   line[512];
    size_t pos = snprintf(line, sizeof(line), "%llu",
                          static_cast<unsigned long long>(esp_timer_get_time() / 1000));

    for (auto &s : can_states_)
        for (const auto &sig : s.def.signals) {
            pos += snprintf(line + pos, sizeof(line) - pos,
                                ",%llu", extract(s.data, s.data_len, sig));
        }

    for (auto &s : adc_states_) {
        if (adc_ready_) {
            esp_err_t ret = adc_device_.read(s.def.channel, &s.value);
            if (ret != ESP_OK) {
                ModuleCoreLogger::error("ADC read failed for channel %u: %s",
                                        static_cast<unsigned>(s.def.channel), esp_err_to_name(ret));
                s.value = 0;
            }
        }
        pos += snprintf(line + pos, sizeof(line) - pos, ",%d", s.value);
    }

    line[pos++] = '\n';
    write_log(line, pos);
}

std::expected<void, ModuleCoreError> Logger::main() {
    write_header();

    esp_timer_handle_t timer;
    esp_timer_create_args_t args = {
        .callback = [](void *arg) {
            static_cast<Logger*>(arg)->log_sample();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "logger",
        .skip_unhandled_events = false,
    };
    esp_timer_create(&args, &timer);
    esp_timer_start_periodic(timer, 1000000 / hz_);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    return {};
}
