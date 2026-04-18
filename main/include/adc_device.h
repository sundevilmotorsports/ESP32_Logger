#pragma once

#include <cstdint>
#include <functional>
#include <string>

struct AdcDeviceDef {
    const char *name = nullptr;
    std::uint8_t channel = 0;
    int command_msb = -1;
    std::uint8_t command_lsb = 0;
    std::function<float(int)> processing = nullptr;

};

struct AdcDeviceState {
    AdcDeviceDef def;
    int raw_val = 0;
};
