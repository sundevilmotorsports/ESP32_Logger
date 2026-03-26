#pragma once

struct SignalSlice {
    const char *name;
    uint8_t     offset;
    uint8_t     len;
};

struct CanDeviceDef {
    uint8_t                  id;
    std::vector<SignalSlice> signals;
};

struct CanDeviceState {
    CanDeviceDef def;
    uint8_t      data[8] = {};
    uint8_t      data_len = 0;
};