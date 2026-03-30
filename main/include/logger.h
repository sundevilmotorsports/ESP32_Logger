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

class Logger {
public:
    Logger();

    void register_can_device(CanDeviceDef def);
    void register_adc_device(AdcDeviceDef def);
    esp_err_t init_adc(const AdcDriver::Config &config);
    void set_file_name(std::string name) { this->name_ = std::move(name); }

    void on_can_frame(const CanFrame *frame);

    std::expected<void, ModuleCoreError> main();

private:
    int hz_ = 25;

    std::string name_;

    std::vector<CanDeviceState> can_states_;
    std::vector<AdcDeviceState> adc_states_;
    AdcDriver adc_driver_;
    bool adc_ready_ = false;

    GNSS& gnss_;

    SDCard sd_;

    void write_header();

    esp_err_t apply_adc_channel_config(const AdcDeviceDef &def);

    uint64_t extract(const uint8_t *data, uint8_t data_len, const SignalSlice &sig);

    void write_log(const char *buf, size_t len) {
        /* sd_.write(buf, len); */
        fwrite(buf, 1, len, stdout);
    }

    void log_sample();
};
