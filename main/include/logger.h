#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "module_core.h"
#include "adc_driver.h"
#include "adc_device.h"
#include "can_device.h"
#include "sd.h"
#include <iostream>
#include "gnss.h"

extern ModuleCore g_module;

enum class ColType : uint8_t {
    UINT8  = 0x01,
    UINT16 = 0x02,
    UINT32 = 0x03,
    UINT64 = 0x04,
    INT32  = 0x05,
    FLOAT  = 0x06,
};

class Logger {
public:
    Logger();

    void register_can_device(CanDeviceDef def);
    void register_adc_device(AdcDeviceDef def);
    esp_err_t init_adc(const AdcDriver::Config &config);

    void on_can_frame(const CanFrame *frame);

    void list_logs();
    void dump_log(const std::string& filename = "");

    void on_uart_rx(const uint8_t *data, size_t len);

    void send_log_snapshot();

    std::expected<void, ModuleCoreError> main();

private:
    int hz_ = 25;

    std::vector<CanDeviceState> can_states_;
    std::vector<AdcDeviceState> adc_states_;
    AdcDriver adc_driver_;
    bool adc_ready_ = false;

    GNSS& gnss_;

    SDCard sd_ {};

    static ColType col_type_for_len(uint8_t len) {
        switch (len) {
            case 1:  return ColType::UINT8;
            case 2:  return ColType::UINT16;
            case 4:  return ColType::UINT32;
            default: return ColType::UINT64;
        }
    }

    std::vector<uint8_t> build_schema_bytes() const;

    void write_header();
    void send_schema();

    esp_err_t apply_adc_channel_config(const AdcDeviceDef &def);

    // If update_adc is true, reads fresh ADC values (only safe from the timer task).
    size_t fill_data_row(uint8_t* row, bool update_adc);

    void write_log(const void* buf, size_t len) {
        esp_err_t ret = sd_.write(buf, len);
        if (ret != ESP_OK) {
            ModuleCoreLogger::error("Failed to write to SD card: %s", esp_err_to_name(ret));
        }
    }

    void log_sample();
};
