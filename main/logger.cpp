#include "logger.h"

#include <chrono>
#include <filesystem>
#include "module_core.h"

Logger::Logger() : gnss_(GNSS::instance()) {
    sd_.init();

    auto mux_frame = [](uint8_t frame_index) {
        return [frame_index](const uint8_t* data, uint8_t len) -> bool {
            return len > 0 && data[0] == frame_index;
        };
    };

    // Shared helpers for wheel-board big-endian 16-bit extraction.
    auto be16 = [](const uint8_t* d) -> uint16_t {
        return static_cast<uint16_t>(d[0]) << 8 | d[1];
    };
    auto wheel_temp = [be16](uint8_t* d, uint8_t /*len*/) -> float {
        return (static_cast<float>(be16(d)) * 0.02f - 273.15f) / 1000.0f;
    };
    auto wheel_rpm = [be16](uint8_t* d, uint8_t /*len*/) -> float {
        return static_cast<float>(be16(d));
    };

    // IMU Gyro  (little-endian int16, scale = 17.5 mdps/LSB)
    this->register_can_device({
        .id = 0x360,
        .signals = {
            {"IMU_X_GYRO", 0, 2, [](uint8_t* d, uint8_t) -> float {
                int16_t raw; memcpy(&raw, d, 2); return raw * 17.50f; }},
            {"IMU_Y_GYRO", 2, 2, [](uint8_t* d, uint8_t) -> float {
                int16_t raw; memcpy(&raw, d, 2); return raw * 17.50f; }},
            {"IMU_Z_GYRO", 4, 2, [](uint8_t* d, uint8_t) -> float {
                int16_t raw; memcpy(&raw, d, 2); return raw * 17.50f; }},
        }
    });

    // IMU Accel  (little-endian int16, scale = 0.122 mg/LSB)
    this->register_can_device({
        .id = 0x361,
        .signals = {
            {"IMU_X_ACCEL", 0, 2, [](uint8_t* d, uint8_t) -> float {
                int16_t raw; memcpy(&raw, d, 2); return raw * 0.000122f; }},
            {"IMU_Y_ACCEL", 2, 2, [](uint8_t* d, uint8_t) -> float {
                int16_t raw; memcpy(&raw, d, 2); return raw * 0.000122f; }},
            {"IMU_Z_ACCEL", 4, 2, [](uint8_t* d, uint8_t) -> float {
                int16_t raw; memcpy(&raw, d, 2); return raw * 0.000122f; }},
        }
    });

    // FL WheelBoard
    this->register_can_device({
        .id = 0x370,
        .signals = {
            {"FLW_RPM", 0, 2, wheel_rpm},
            {"FLW_OBJ", 2, 2, wheel_temp},
            {"FLW_AMB", 4, 2, wheel_temp},
        }
    });

    // FR Wheel Board
    this->register_can_device({
        .id = 0x380,
        .signals = {
            {"FRW_RPM", 0, 2, wheel_rpm},
            {"FRW_OBJ", 2, 2, wheel_temp},
            {"FRW_AMB", 4, 2, wheel_temp},
        }
    });

    // RR WheelBoard
    this->register_can_device({
        .id = 0x390,
        .signals = {
            {"RRW_RPM", 0, 2, wheel_rpm},
            {"RRW_OBJ", 2, 2, wheel_temp},
            {"RRW_AMB", 4, 2, wheel_temp},
        }
    });

    // RL WheelBoard
    this->register_can_device({
        .id = 0x3A0,
        .signals = {
            {"RLW_RPM", 0, 2, wheel_rpm},
            {"RLW_OBJ", 2, 2, wheel_temp},
            {"RLW_AMB", 4, 2, wheel_temp},
        }
    });

    // Engine CAN Stream 2, muxed by data[0]
    this->register_can_device({
        .id = 0x3E8,
        .matches = mux_frame(0x0),
        .signals = {
            {"ECT",     3, 1},  // raw uint8, no processing
            {"OIL_PSR", 5, 2, [be16](uint8_t* d, uint8_t) -> float {
                return static_cast<float>(be16(d)); }},
        }
    });

    this->register_can_device({
        .id = 0x3E8,
        .matches = mux_frame(0x1),
        .signals = {
            {"TPS",         2, 1},  // raw uint8
            {"DRIVEN_WSPD", 4, 2, [be16](uint8_t* d, uint8_t) -> float {
                return static_cast<float>(be16(d)); }},
        }
    });

    this->register_can_device({
        .id = 0x3E8,
        .matches = mux_frame(0x2),
        .signals = {
            {"APS", 1, 1},  // raw uint8
        }
    });

    // Strain Gauges (raw uint16, no processing)
    this->register_can_device({ .id = 0x4E2, .signals = {{"FL_SG", 0, 2}} });
    this->register_can_device({ .id = 0x4E3, .signals = {{"FR_SG", 0, 2}} });
    this->register_can_device({ .id = 0x4E4, .signals = {{"RR_SG", 0, 2}} });
    this->register_can_device({ .id = 0x4E5, .signals = {{"RL_SG", 0, 2}} });

    this->register_adc_device({
        .name = "FRSHOCK",
        .channel = 0,
        .processing = [](int v) -> float { return -0.018444f * (v - 1324); }
    });

    this->register_adc_device({
        .name = "RRSHOCK",
        .channel = 1,
        .processing = [](int v) -> float { return -0.018498f * (v - 1370); }
    });

    this->register_adc_device({ .name = "R_BRAKEPRESSURE", .channel = 2 });

    this->register_adc_device({
        .name = "RLSHOCK",
        .channel = 3,
        .processing = [](int v) -> float { return -0.018498f * (v - 1370); }
    });

    this->register_adc_device({ .name = "F_BRAKEPRESSURE", .channel = 4 });

    this->register_adc_device({
        .name = "FLSHOCK",
        .channel = 6,
        .processing = [](int v) -> float { return -0.018586f * (v - 1311); }
    });

    this->register_adc_device({
        .name = "STEERING",
        .channel = 7,
        .processing = [](int v) -> float { return 0.084769f * (v - 1783); }
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
    for (auto &s : can_states_) {
        if (s.def.id != frame->header.id) continue;
        if (s.def.extended != static_cast<bool>(frame->header.ide)) continue;
        if (s.def.matches != nullptr && !s.def.matches(frame->data, frame->len)) continue;
        const uint8_t safe_len = std::min(frame->len, static_cast<uint8_t>(sizeof(s.data)));
        memcpy(s.data, frame->data, safe_len);
        if (safe_len < sizeof(s.data)) {
            memset(s.data + safe_len, 0, sizeof(s.data) - safe_len);
        }
        s.data_len = frame->len;
    }
}

void Logger::list_logs() {
    std::vector<std::string> files = sd_.get_logs_list();

    for (const std::string& file : files) {
        uint64_t size = sd_.get_file_size(file);
        // Payload: "<name>\0<size_decimal>" — host splits on null to get name and size
        std::string payload = file + '\0' + std::to_string(size);

        ModuleCore::UartResponse resp {};
        resp.msg_type      = 0xA1;
        resp.data          = reinterpret_cast<uint8_t *>(payload.data());
        resp.data_len      = payload.size();
        resp.source_device = g_module.getId();
        g_module.sendUartResponse(resp);
    }

    ModuleCore::UartResponse resp {};
    resp.msg_type      = 0xA1;
    resp.source_device = g_module.getId();
    g_module.sendUartResponse(resp);
}

void Logger::dump_log(const std::string& filename) {
    static constexpr size_t CHUNK = 128;

    auto send_chunk = [](const uint8_t *chunk, size_t len) {
        ModuleCore::UartResponse resp{};
        resp.msg_type      = 0xA2; // MSG_LOG_SLICE
        resp.source_device = g_module.getId();
        resp.data          = const_cast<uint8_t *>(chunk);
        resp.data_len      = len;
        g_module.sendUartResponse(resp);
    };

    if (filename.empty()) {
        sd_.stream_current(CHUNK, send_chunk);
    } else {
        sd_.stream_file(filename, CHUNK, send_chunk);
    }

    // Empty slice signals end-of-stream to the host
    ModuleCore::UartResponse eos{};
    eos.msg_type      = 0xA2;
    eos.source_device = g_module.getId();
    g_module.sendUartResponse(eos);
}

void Logger::on_uart_rx(const uint8_t *data, size_t len) {
    if (len < 2) { return; }

    switch (data[1]) {
        case 0x51:      // CMD_LIST_LOGS
            list_logs();
            break;
        case 0x52: {    // CMD_DUMP_LOG — optional filename in payload bytes [2..]
            std::string filename;
            if (len > 2) {
                filename = std::string(reinterpret_cast<const char*>(data + 2), len - 2);
                // Reject path traversal / absolute paths.
                if (filename.find('/') != std::string::npos ||
                    filename.find("..") != std::string::npos) {
                    break;
                }
            }
            dump_log(filename);
            break;
        }
        case 0x53: {    // CMD_GET_CURRENT_LOG
            std::string name = sd_.get_current_name();
            ModuleCore::UartResponse resp{};
            resp.msg_type      = 0xA4; // MSG_CURRENT_LOG
            resp.source_device = g_module.getId();
            resp.data          = reinterpret_cast<uint8_t *>(name.data());
            resp.data_len      = name.size();
            g_module.sendUartResponse(resp);
            break;
        }
        case 0x54:      // CMD_SNAPSHOT — binary data row (same layout as file rows)
            send_log_snapshot();
            break;
        case 0x55:      // CMD_GET_SCHEMA — binary file header describing every column
            send_schema();
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Binary format
// ---------------------------------------------------------------------------
//
// File header
//   [1]  num_columns
//   per column:
//     [1]  name_len
//     [N]  name bytes (no null terminator)
//     [1]  ColType tag (see logger.h)
//
// Data rows (fixed-width, no delimiters, written at 25 Hz):
//   uint64   timestamp  (ms since boot)
//   per CAN signal:   float32 if has processing fn, else raw bytes (sig.len)
//   per ADC channel:  float32 if has processing fn, else int32 (raw ADC count)
//   float32  Lat   (0.0 when no fix)
//   float32  Lon   (0.0 when no fix)
//   int32    Speed (0   when no fix)

std::vector<uint8_t> Logger::build_schema_bytes() const {
    std::vector<uint8_t> out;
    out.reserve(256);

    // Placeholder for num_columns (uint8, filled in below)
    out.push_back(0);
    uint8_t ncols = 0;

    auto append_col = [&](const char* name, ColType type) {
        const uint8_t nlen = static_cast<uint8_t>(strlen(name));
        out.push_back(nlen);
        out.insert(out.end(), name, name + nlen);
        out.push_back(static_cast<uint8_t>(type));
        ++ncols;
    };

    append_col("timestamp", ColType::UINT64);

    for (const auto &s : can_states_)
        for (const auto &sig : s.def.signals)
            append_col(sig.name, sig.processing ? ColType::FLOAT : col_type_for_len(sig.len));

    for (const auto &s : adc_states_)
        append_col(s.def.name, s.def.processing ? ColType::FLOAT : ColType::INT32);

    append_col("Lat",   ColType::FLOAT);
    append_col("Lon",   ColType::FLOAT);
    append_col("Speed", ColType::INT32);

    out[0] = ncols;
    return out;
}

void Logger::write_header() {
    const auto schema = build_schema_bytes();
    write_log(schema.data(), schema.size());
}

void Logger::send_schema() {
    auto schema = build_schema_bytes();
    ModuleCore::UartResponse resp{};
    resp.msg_type      = 0xA5; // MSG_SCHEMA
    resp.source_device = g_module.getId();
    resp.data          = schema.data();
    resp.data_len      = schema.size();
    g_module.sendUartResponse(resp);
}

size_t Logger::fill_data_row(uint8_t* row, bool update_adc) {
    size_t pos = 0;
    const auto put = [&](const void* d, size_t n) {
        memcpy(row + pos, d, n);
        pos += n;
    };

    const uint64_t ts = static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
    put(&ts, sizeof(ts));

    for (auto &s : can_states_) {
        for (const auto &sig : s.def.signals) {
            if (sig.processing) {
                const float val = (sig.offset + sig.len <= s.data_len)
                    ? sig.processing(s.data + sig.offset, sig.len) : 0.0f;
                put(&val, sizeof(val));
            } else {
                const uint8_t safe = (sig.offset + sig.len <= sizeof(s.data)) ? sig.len : 0;
                put(s.data + sig.offset, safe);
                if (safe < sig.len) {
                    static constexpr uint8_t zero[8] = {};
                    put(zero, sig.len - safe);
                }
            }
        }
    }

    for (auto &s : adc_states_) {
        if (update_adc && adc_ready_) {
            if (adc_driver_.read(s.def.channel, &s.raw_val) != ESP_OK)
                s.raw_val = 0;
        }
        if (s.def.processing) {
            const float val = s.def.processing(s.raw_val);
            put(&val, sizeof(val));
        } else {
            put(&s.raw_val, sizeof(s.raw_val));
        }
    }

    const auto gs = gnss_.get_state();
    const bool    fix = gs.satellites > 0;
    const float   lat = fix ? gs.fLat    : 0.0f;
    const float   lon = fix ? gs.fLon    : 0.0f;
    const int32_t spd = fix ? gs.gSpeed  : 0;
    put(&lat, sizeof(lat));
    put(&lon, sizeof(lon));
    put(&spd, sizeof(spd));

    return pos;
}

void Logger::send_log_snapshot() {
    uint8_t row[512];
    const size_t len = fill_data_row(row, /*update_adc=*/false);
    ModuleCore::UartResponse resp{};
    resp.msg_type      = 0xA3; // MSG_SNAPSHOT
    resp.source_device = g_module.getId();
    resp.data          = row;
    resp.data_len      = len;
    g_module.sendUartResponse(resp);
}

esp_err_t Logger::init_adc(const AdcDriver::Config &config) {
    if (adc_ready_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = adc_driver_.init(config);
    if (ret != ESP_OK) {
        return ret;
    }

    for (const auto &state : adc_states_) {
        ret = apply_adc_channel_config(state.def);
        if (ret != ESP_OK) {
            adc_driver_.deinit();
            return ret;
        }
    }

    adc_ready_ = true;
    return ESP_OK;
}

esp_err_t Logger::apply_adc_channel_config(const AdcDeviceDef &def) {
    if (!adc_driver_.initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (def.command_msb < 0) {
        return adc_driver_.config_channel(def.channel);
    }

    AdcDriver::ChannelConfig config = {
        .tx_data = {
            static_cast<std::uint8_t>(def.command_msb),
            def.command_lsb,
        },
    };
    return adc_driver_.config_channel(def.channel, config);
}

void Logger::log_sample() {
    uint8_t row[512];
    const size_t len = fill_data_row(row, /*update_adc=*/true);
    write_log(row, len);
}

std::expected<void, ModuleCoreError> Logger::main() {
    gnss_.init();

    for (int i = 0; i < 5 && !gnss_.isInitialized(); i++) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    xTaskCreate(GNSS::uartTask, "GNSS", 4096, nullptr, 5, &gnss_.taskHandle);

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
