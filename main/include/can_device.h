#pragma once

struct SignalSlice {
    const char *name;
    uint8_t     offset;
    uint8_t     len;

    std::function<std::string(uint8_t*, uint8_t)> processing = nullptr;
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