#pragma once

#include <cstdint>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define GNSS_DMA_BUF_SIZE 2048
#define GNSS_PATTERN_CHR  '\n'

struct GNSSState {
    uint8_t  uniqueID[5]          = {};
    uint8_t  uartWorkingBuffer[101] = {};
    uint16_t year                 = 0;
    uint8_t  yearBytes[2]         = {};
    uint8_t  month                = 0;
    uint8_t  day                  = 0;
    uint8_t  hour                 = 0;
    uint8_t  min                  = 0;
    uint8_t  sec                  = 0;
    uint8_t  fixType              = 0;
    int32_t  lon                  = 0;
    uint8_t  lonBytes[4]          = {};
    int32_t  lat                  = 0;
    uint8_t  latBytes[4]          = {};
    float    fLon                 = 0.0f;
    float    fLat                 = 0.0f;
    int32_t  height               = 0;
    int32_t  hMSL                 = 0;
    uint8_t  hMSLBytes[4]         = {};
    uint32_t hAcc                 = 0;
    uint32_t vAcc                 = 0;
    int32_t  gSpeed               = 0;
    uint8_t  gSpeedBytes[4]       = {};
    int32_t  headMot              = 0;
};

class GNSS {
public:
    static GNSS& instance();

    GNSS(const GNSS&) = delete;
    GNSS& operator=(const GNSS&) = delete;
    ~GNSS();

    void init();

    bool isInitialized() const;
    bool queryUniqueID();

    static void uartTask(void* pvParameters);

    GNSSState state{};
    TaskHandle_t taskHandle = nullptr;

private:
    static constexpr const char* TAG = "GNSS";

    GNSS() = default;

    void runUartTask();

    bool parseGNGGA(const char* sentence);
    bool parseGNRMC(const char* sentence);
    void parseGSV(const char* sentence);
    void processNMEA(const char* sentence, size_t len);

    QueueHandle_t eventQueue_ = nullptr;
    uint8_t*      dmaBuf_     = nullptr;
    bool          ready_      = false;
};
