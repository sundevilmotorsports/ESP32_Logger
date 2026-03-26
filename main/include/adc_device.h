#pragma once

#include "esp_adc/adc_oneshot.h"

struct AdcDeviceDef {
    const char    *name;
    adc_channel_t  channel;
    adc_unit_t     unit;
};

struct AdcDeviceState {
    AdcDeviceDef def;
    int          value = 0;
};
