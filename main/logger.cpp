#include "logger.h"

#include <chrono>
#include <filesystem>
#include "module_core.h"

Logger::Logger() : gnss_(GNSS::instance()) {
    sd_.init();

    auto p = [](uint8_t* data, uint8_t len) -> std::string {
        return "processed";
    };

    auto mux_frame = [](uint8_t frame_index) {
        return [frame_index](const uint8_t* data, uint8_t len) -> bool {
            return len > 0 && data[0] == frame_index;
        };
    };

    // this->register_can_device({
    //     .id = 0x01,
    //     .signals = {
    //         { "0first", 0, 2, p},
    //         {"0second", 2, 2},
    //         {"0third", 4, 2, p},
    //         {"0fourth", 6, 2},
    //     }
    // });

    // IMU Gyro
    this->register_can_device({
        .id = 0x360,
        .signals = {
            {"IMU_X_GYRO", 0, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    int16_t raw;
                    memcpy(&raw, data, sizeof(raw));
                    float out = ((float)raw * 17.50);
                    return std::to_string(out);
                }
            },
            {"IMU_Y_GYRO", 2, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    int16_t raw;
                    memcpy(&raw, data, sizeof(raw));
                    float out = ((float)raw * 17.50);
                    return std::to_string(out);
                }
            },
            {"IMU_Z_GYRO", 4, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    int16_t raw;
                    memcpy(&raw, data, sizeof(raw));
                    float out = ((float)raw * 17.50);
                    return std::to_string(out);
                }
            }

        }
    });

    // IMU Accel
    this->register_can_device({
        .id = 0x361,
        .signals = {
            {"IMU_X_ACCEL", 0, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    int16_t raw;
                    memcpy(&raw, data, sizeof(raw));
                    float out = ((float)raw * 0.122) / (float)1000;
                    return std::to_string(out);
                }
            },
            {"IMU_Y_ACCEL", 2, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    int16_t raw;
                    memcpy(&raw, data, sizeof(raw));
                    float out = ((float)raw * 0.122) / (float)1000;
                    return std::to_string(out);
                }
            },
            {"IMU_Z_ACCEL", 4, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    int16_t raw;
                    memcpy(&raw, data, sizeof(raw));
                    float out = ((float)raw * 0.122) / (float)1000;
                    return std::to_string(out);
                }
            }

        }
    });

    // FL WheelBoard
    this->register_can_device({
        .id = 0x370,
        .signals = {
            {"FLW_RPM", 0, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    return std::to_string(raw);
                }
            },
            {"FLW_OBJ", 2, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    float out = (((float)raw * 0.02) - 273.15) / (float)1000;
                    return std::to_string(out);                }
            },
            {"FLW_AMB", 4, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    float out = (((float)raw * 0.02) - 273.15) / (float)1000;
                    return std::to_string(out);                }
            }

        }
    });
    // FR Wheel Board
    this->register_can_device({
        .id = 0x380,
        .signals = {
            {"FRW_RPM", 0, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    return std::to_string(raw);
                }
            },
            {"FRW_OBJ", 2, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    float out = (((float)raw * 0.02) - 273.15) / (float)1000;
                    return std::to_string(out);                }
            },
            {"FRW_AMB", 4, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    float out = (((float)raw * 0.02) - 273.15) / (float)1000;
                    return std::to_string(out);                }
            }

        }
    });
    //RR Wheelbaord
    this->register_can_device({
        .id = 0x390,
        .signals = {
            {"RRW_RPM", 0, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    return std::to_string(raw);
                }
            },
            {"RRW_OBJ", 2, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    float out = (((float)raw * 0.02) - 273.15) / (float)1000;
                    return std::to_string(out);                }
            },
            {"RRW_AMB", 4, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    float out = (((float)raw * 0.02) - 273.15) / (float)1000;
                    return std::to_string(out);                }
            }

        }
    });
    //RL Wheelbaord
    this->register_can_device({
        .id = 0x3A0,
        .signals = {
            {"RLW_RPM", 0, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    return std::to_string(raw);
                }
            },
            {"RLW_OBJ", 2, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    float out = (((float)raw * 0.02) - 273.15) / (float)1000;
                    return std::to_string(out);
                }
            },
            {"RLW_AMB", 4, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    float out = (((float)raw * 0.02) - 273.15) / (float)1000;
                    return std::to_string(out);                }
            }

        }
    });

    // Engine CAN Stream 2, muxed by data[0]
    this->register_can_device({
        .id = 0x3E8,
        .matches = mux_frame(0x0),
        .signals = {
            {"ENGINE_ECT", 3, 1},
            {"ENGINE_OIL_PRESS", 5, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    return std::to_string(raw);
                }
            },
        }
    });

    this->register_can_device({
        .id = 0x3E8,
        .matches = mux_frame(0x1),
        .signals = {
            {"ENGINE_TPS", 2, 1},
            {"ENGINE_DRIVEN_WSPD", 4, 2, [] (uint8_t* data, uint8_t /*len*/) -> std::string {
                    uint16_t raw = data[0] << 8 | data[1];
                    return std::to_string(raw);
                }
            },
        }
    });

    this->register_can_device({
        .id = 0x3E8,
        .matches = mux_frame(0x2),
        .signals = {
            {"ENGINE_APS", 1, 1},
        }
    });


    // this->register_can_device({
    //     .id = 0x02,
    //     .signals = {
    //         {"large", 0, 8, [](uint8_t* data) {return nullptr;}},
    //     }
    // });

    this->register_adc_device({
        .name = "FRSHOCK",
        .channel = 0,
        .processing = [](int v) -> std::string {
            float data = ( ((-0.018444) * ( (v) - (1324) ) ));
            return std::to_string(data);
        }
    });
    
    this->register_adc_device({
        .name = "RRSHOCK",
        .channel = 1,
        .processing = [](int v) -> std::string {
            float data = ( (-0.018498) * ( (v) - (1370) ) );
            return std::to_string(data);
        }
    });

    this->register_adc_device({
        .name = "R_BRAKEPRESSURE",
        .channel = 2,
    });

    this->register_adc_device({
        .name = "RLSHOCK",
        .channel = 3,
        .processing = [](int v) -> std::string {
            float data = ( (-0.018498) * ( (v) - (1370) ) );
            return std::to_string(data);
        }
    });
    
    this->register_adc_device({
        .name = "F_BRAKEPRESSURE",
        .channel = 4,
    });

    this->register_adc_device({
        .name = "FLSHOCK",
        .channel = 6,
        .processing = [](int v) -> std::string {
            float data = ( (-0.018586) * ( (v) - (1311) ) );
            return std::to_string(data);
        }
    });
    
    this->register_adc_device({
        .name = "STEERING",
        .channel = 7,
        .processing = [](int v) -> std::string {
            float data =( (0.084769) * ( (v) - (1783) ) );
            return std::to_string(data);
        }
    });


    this->gnss_.init();

    for (int i = 0; i < 5 && !this->gnss_.isInitialized(); i++) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    xTaskCreate(GNSS::uartTask, "GNSS", 4096, nullptr, 5, nullptr);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }
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
        memcpy(s.data, frame->data, frame->len);
        if (frame->len < sizeof(s.data)) {
            memset(s.data + frame->len, 0, sizeof(s.data) - frame->len);
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
        resp.msg_type = 0xA1;
        resp.data = reinterpret_cast<uint8_t *>(payload.data());
        resp.data_len = payload.size();
        resp.source_device = g_module.getId();
        g_module.sendUartResponse(resp);
    }

    ModuleCore::UartResponse resp {};
    resp.msg_type = 0xA1;
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
        case 0x54: {    // CMD_SNAPSHOT
            send_log_snapshot();
            break;
        }
        default:
            break;
    }
}

void Logger::send_log_snapshot() {
    char   line[512];
    size_t pos = snprintf(line, sizeof(line), "timestamp");
    for (auto &s : can_states_)
        for (const auto &sig : s.def.signals)
            pos += snprintf(line + pos, sizeof(line) - pos, ",%s", sig.name);
    for (auto &s : adc_states_)
        pos += snprintf(line + pos, sizeof(line) - pos, ",%s", s.def.name);
    pos += snprintf(line + pos, sizeof(line) - pos, ",Lat,Lon,Speed");
    line[pos++] = '\n';

    pos += snprintf(line + pos, sizeof(line) - pos, "%llu",
                          static_cast<unsigned long long>(esp_timer_get_time() / 1000));

    for (auto &s : can_states_)
        for (const auto &sig : s.def.signals) {
            if (sig.processing != nullptr) {
                pos += snprintf(line + pos, sizeof(line) - pos,
                                ",%s", sig.processing(s.data, s.data_len).c_str());
            } else {
                pos += snprintf(line + pos, sizeof(line) - pos,
                                ",%llu", extract(s.data, s.data_len, sig));
            }
        }

    for (auto &s : adc_states_) {
        // Use cached raw_val — do NOT call adc_driver_.read() here since this runs
        // on the UART task and SPI transactions from that context cause a crash.
        if (s.def.processing != nullptr){
            pos += snprintf(line + pos, sizeof(line) - pos,
                            ",%s", s.def.processing(s.raw_val).c_str());
        } else {
            pos += snprintf(line + pos, sizeof(line) - pos, ",%d", s.raw_val);
        }
    }

    if (gnss_.state.satellites > 0) {
        pos += snprintf(line + pos, sizeof(line) - pos,
            ",%f,%f,%lu", gnss_.state.fLat, gnss_.state.fLon, gnss_.state.gSpeed);
    }

    line[pos++] = '\n';
    ModuleCore::UartResponse resp{};
    resp.msg_type      = 0xA3; // MSG_SNAPSHOT
    resp.source_device = g_module.getId();
    resp.data          = reinterpret_cast<uint8_t *>(line);
    resp.data_len      = pos;
    g_module.sendUartResponse(resp);
}

void Logger::write_header() {
    char   line[512];
    size_t pos = snprintf(line, sizeof(line), "timestamp");
    for (auto &s : can_states_)
        for (const auto &sig : s.def.signals)
            pos += snprintf(line + pos, sizeof(line) - pos, ",%s", sig.name);
    for (auto &s : adc_states_)
        pos += snprintf(line + pos, sizeof(line) - pos, ",%s", s.def.name);

    pos += snprintf(line + pos, sizeof(line) - pos, "Lat,Lon,Speed");
    line[pos++] = '\n';
    write_log(line, pos);
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
            adc_driver_.~AdcDriver();
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
            if (sig.processing != nullptr) {
                pos += snprintf(line + pos, sizeof(line) - pos,
                                ",%s", sig.processing(s.data + sig.offset, sig.len).c_str());
            } else {
                pos += snprintf(line + pos, sizeof(line) - pos,
                                ",%llu", extract(s.data, s.data_len, sig));
            }
        }

    for (auto &s : adc_states_) {
        if (adc_ready_) {
            esp_err_t ret = adc_driver_.read(s.def.channel, &s.raw_val);
            if (ret != ESP_OK) {
                ModuleCoreLogger::error("ADC read failed for channel %u: %s",
                                        static_cast<unsigned>(s.def.channel), esp_err_to_name(ret));
                s.raw_val = 0;
            }
        }
        if (s.def.processing != nullptr){
            pos += snprintf(line + pos, sizeof(line) - pos, 
                            ",%s", s.def.processing(s.raw_val).c_str());
        } else {
            pos += snprintf(line + pos, sizeof(line) - pos, ",%d", s.raw_val);

        }
    }

    if (gnss_.state.satellites > 0) {
        pos += snprintf(line + pos, sizeof(line) - pos,
            ",%f,%f,%lu", gnss_.state.fLat, gnss_.state.fLon, gnss_.state.gSpeed);
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
