#include "logger.h"

#include <chrono>
#include <filesystem>

Logger::Logger() {
    this->name_ = "tmp";
    sd_.init();
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

uint32_t Logger::extract(const uint8_t *data, uint8_t data_len, const SignalSlice &sig) {
    if (sig.offset + sig.len > data_len) return 0;
    uint32_t raw = 0;
    for (uint8_t i = 0; i < sig.len; i++)
        raw |= (uint32_t)data[sig.offset + i] << (i * 8);
    return raw;
}

std::expected<void, ModuleCoreError> Logger::main() {
    write_header();

    char         line[512];
    TickType_t   last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / hz_);

    for (;;) {
        size_t pos = snprintf(line, sizeof(line), "%llu",
                              (unsigned long long)(esp_timer_get_time() / 1000));

        for (auto &s : can_states_)
            for (const auto &sig : s.def.signals)
                pos += snprintf(line + pos, sizeof(line) - pos,
                                ",%lu", (unsigned long)extract(s.data, s.data_len, sig));

        for (auto &s : adc_states_) {
            adc_oneshot_read(adc_handles_[s.def.unit], s.def.channel, &s.value);
            pos += snprintf(line + pos, sizeof(line) - pos, ",%d", s.value);
        }

        line[pos++] = '\n';
        write_log(line, pos);

        vTaskDelayUntil(&last_wake, period);
    }

    return {};
}
