#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

struct SignalSlice {
    const char *name;
    std::uint8_t offset;
    std::uint8_t len;

    std::function<std::string(std::uint8_t*, std::uint8_t)> processing = nullptr;
};

struct CanDeviceDef {
    std::uint32_t id = 0;
    bool extended = false;
    std::function<bool(const std::uint8_t*, std::uint8_t)> matches = nullptr;
    std::vector<SignalSlice> signals;
};

struct CanDeviceState {
    CanDeviceDef def;
    std::uint8_t data[8] = {};
    std::uint8_t data_len = 0;
};
