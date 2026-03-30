#pragma once

#include <cstdint>

struct AdcDeviceDef {
    const char *name = nullptr;
    std::uint8_t channel = 0;
    int command_msb = -1;
    std::uint8_t command_lsb = 0;
};

struct AdcDeviceState {
    AdcDeviceDef def;
    int value = 0;
};
