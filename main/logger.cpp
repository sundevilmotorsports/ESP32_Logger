#include "logger.h"

#include <chrono>
#include <filesystem>
#include "module_core.h"

Logger::Logger() : gnss_(GNSS::instance()) {
    this->name_ = "tmp";
    // sd_.init();

    this->register_can_device({
        .id = 0x01,
        .signals = {
            { "0first", 0, 2},
            {"0second", 2, 2},
            {"0third", 4, 2},
            {"0fourth", 6, 2},
        }
    });

    this->register_can_device({
        .id = 0x02,
        .signals = {
            {"large", 0, 8},
        }
    });

    this->gnss_.init();

    for (int i = 0; i < 5 && !this->gnss_.isInitialized(); i++) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    xTaskCreate(GNSS::uartTask, "GNSS", 4096, nullptr, 5, nullptr);
}


void Logger::register_can_device(CanDeviceDef def) {
    can_states_.push_back({ .def = std::move(def) });
}

void Logger::register_adc_device(AdcDeviceDef def) {
    ensure_adc_unit(def.unit);
    adc_oneshot_chan_cfg_t cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc_handles_[def.unit], def.channel, &cfg);
    adc_states_.push_back({ .def = def });
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

void Logger::ensure_adc_unit(adc_unit_t unit) {
    if (adc_handles_[unit]) return;
    adc_oneshot_unit_init_cfg_t cfg = { .unit_id = unit };
    adc_oneshot_new_unit(&cfg, &adc_handles_[unit]);
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
        adc_oneshot_read(adc_handles_[s.def.unit], s.def.channel, &s.value);
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
        .name = "logger"
    };
    esp_timer_create(&args, &timer);
    esp_timer_start_periodic(timer, 1000000 / hz_);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    return {};
}
