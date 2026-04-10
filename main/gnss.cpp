#include "gnss.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static constexpr uart_port_t  NEO_UART_PORT = UART_NUM_1;
static constexpr gpio_num_t   NEO_TX_PIN    = GPIO_NUM_19;
static constexpr gpio_num_t   NEO_RX_PIN    = GPIO_NUM_20;

void GNSS::init() {
    ready_ = false;

    const uart_config_t cfg = {
        .baud_rate  = 38400,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "Installing UART driver...");

    esp_err_t err = uart_driver_install(NEO_UART_PORT, GNSS_DMA_BUF_SIZE, 0, 20, &eventQueue_, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err));
        return;
    }

    err = uart_param_config(NEO_UART_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(err));
        uart_driver_delete(NEO_UART_PORT);
        return;
    }

    err = uart_set_pin(NEO_UART_PORT, NEO_TX_PIN, NEO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_pin: %s (continuing)", esp_err_to_name(err));
    }

    err = uart_enable_pattern_det_baud_intr(NEO_UART_PORT, GNSS_PATTERN_CHR, 1, 9, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_enable_pattern_det_baud_intr: %s", esp_err_to_name(err));
        uart_driver_delete(NEO_UART_PORT);
        return;
    }

    uart_pattern_queue_reset(NEO_UART_PORT, 64);

    dmaBuf_ = static_cast<uint8_t*>(heap_caps_malloc(GNSS_DMA_BUF_SIZE, MALLOC_CAP_DMA));
    if (!dmaBuf_) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer");
        uart_driver_delete(NEO_UART_PORT);
        return;
    }

    // Need to configure input via desktop application
    // if (!queryUniqueID()) {
    //     ESP_LOGE(TAG, "Failed to fetch ID from GNSS");
    //     return;
    // }

    ready_ = true;
    ESP_LOGI(TAG, "GNSS init complete");
}

GNSS::~GNSS() {
    ready_ = false;

    if (taskHandle) {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }

    if (eventQueue_) {
        uart_driver_delete(NEO_UART_PORT);
        eventQueue_ = nullptr;
    }

    if (dmaBuf_) {
        free(dmaBuf_);
        dmaBuf_ = nullptr;
    }
}

bool GNSS::isInitialized() const {
    return ready_;
}

void GNSS::uartTask(void* args) {
    instance().runUartTask();
}

void GNSS::runUartTask() {
    ESP_LOGI(TAG, "UART task started");

    if (!ready_ || !eventQueue_ || !dmaBuf_) {
        ESP_LOGE(TAG, "Not initialized — task exiting");
        vTaskDelete(nullptr);
        return;
    }

    uart_event_t event;

    while (true) {
        if (!xQueueReceive(eventQueue_, &event, portMAX_DELAY)) continue;

        switch (event.type) {

        case UART_DATA:
            ESP_LOGD(TAG, "UART_DATA: %d bytes", event.size);
            break;

        case UART_PATTERN_DET: {
            size_t buffered = 0;
            uart_get_buffered_data_len(NEO_UART_PORT, &buffered);
            ESP_LOGD(TAG, "Pattern detected, buffered=%d", buffered);

            if (buffered == 0) {
                ESP_LOGW(TAG, "Pattern with no buffered data");
                break;
            }

            int pos = uart_pattern_pop_pos(NEO_UART_PORT);
            if (pos < 0 || pos >= GNSS_DMA_BUF_SIZE - 1) {
                uart_flush_input(NEO_UART_PORT);
                ESP_LOGW(TAG, "Bad pattern pos (%d), flushing", pos);
                break;
            }

            memset(dmaBuf_, 0, GNSS_DMA_BUF_SIZE);
            int nread = uart_read_bytes(NEO_UART_PORT, dmaBuf_, pos + 1,
                                        pdMS_TO_TICKS(100));
            if (nread > 0) {
                processNMEA(reinterpret_cast<char*>(dmaBuf_), nread - 1);
            } else {
                ESP_LOGW(TAG, "Read failed after pattern detection");
            }
            break;
        }

        case UART_FIFO_OVF:
            ESP_LOGW(TAG, "FIFO overflow — flushing");
            uart_flush_input(NEO_UART_PORT);
            xQueueReset(eventQueue_);
            break;

        case UART_BUFFER_FULL:
            ESP_LOGW(TAG, "Ring buffer full — flushing");
            uart_flush_input(NEO_UART_PORT);
            xQueueReset(eventQueue_);
            break;

        case UART_BREAK:
            ESP_LOGW(TAG, "Break detected (RX=%d TX=%d)",
                     gpio_get_level(NEO_RX_PIN), gpio_get_level(NEO_TX_PIN));
            uart_flush_input(NEO_UART_PORT);
            xQueueReset(eventQueue_);
            vTaskDelay(pdMS_TO_TICKS(10));
            break;

        case UART_PARITY_ERR:
            ESP_LOGE(TAG, "Parity error");
            break;

        case UART_FRAME_ERR:
            ESP_LOGE(TAG, "Frame error");
            break;

        default:
            ESP_LOGD(TAG, "Unhandled UART event: %d", event.type);
            break;
        }
    }
}

void GNSS::processNMEA(const char* data, size_t len) {
    if (!data || len == 0) return;

    char buf[512];
    if (len >= sizeof(buf)) {
        ESP_LOGW(TAG, "Sentence too long (%d), truncating", len);
        len = sizeof(buf) - 1;
    }

    memcpy(buf, data, len);
    buf[len] = '\0';

    // Strip trailing whitespace / CR
    while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n' || buf[len - 1] == ' ')) {
        buf[--len] = '\0';
    }

    if (len == 0 || buf[0] != '$') {
        ESP_LOGD(TAG, "Non-NMEA data: %.*s", static_cast<int>(len), buf);
        return;
    }

    ESP_LOGD(TAG, "NMEA: %s", buf);

    if      (strncmp(buf, "$GNGGA", 6) == 0) { parseGNGGA(buf); }
    else if (strncmp(buf, "$GNRMC", 6) == 0) { parseGNRMC(buf); }
    else if (strstr (buf, "GSV")      != nullptr) { parseGSV(buf); }
    else if (strncmp(buf, "$GNGSA", 6) == 0) { ESP_LOGD(TAG, "GSA (DOP/active sats)"); }
    else if (strncmp(buf, "$GNVTG", 6) == 0) { ESP_LOGD(TAG, "VTG (track/ground speed)"); }
    else if (strncmp(buf, "$GNGLL", 6) == 0) { ESP_LOGD(TAG, "GLL (lat/lon)"); }
}

bool GNSS::parseGNGGA(const char* sentence) {
    // $GNGGA,hhmmss.ss,ddmm.mmmmm,N/S,dddmm.mmmmm,E/W,q,nn,h.h,a.a,M,...
    char time_str[16]{}, lat_str[16]{}, lon_str[16]{};
    char lat_ns{}, lon_ew{};
    int  quality{}, satellites{};
    float hdop{}, altitude{};

    int n = sscanf(sentence, "$GNGGA,%[^,],%[^,],%c,%[^,],%c,%d,%d,%f,%f,M",
                   time_str, lat_str, &lat_ns, lon_str, &lon_ew,
                   &quality, &satellites, &hdop, &altitude);

    if (n < 6) return false;

    float fLat = 0.0f, fLon = 0.0f;
    if (strlen(lat_str) > 0 && lat_ns) {
        float raw = atof(lat_str);
        int   deg = static_cast<int>(raw / 100);
        fLat = deg + (raw - deg * 100.0f) / 60.0f;
        if (lat_ns == 'S') fLat = -fLat;
    }
    if (strlen(lon_str) > 0 && lon_ew) {
        float raw = atof(lon_str);
        int   deg = static_cast<int>(raw / 100);
        fLon = deg + (raw - deg * 100.0f) / 60.0f;
        if (lon_ew == 'W') fLon = -fLon;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    state.fixType   = static_cast<uint8_t>(quality);
    state.satellites = satellites;

    if (strlen(time_str) >= 6) {
        state.hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
        state.min  = (time_str[2] - '0') * 10 + (time_str[3] - '0');
        state.sec  = (time_str[4] - '0') * 10 + (time_str[5] - '0');
    }

    if (strlen(lat_str) > 0 && lat_ns) {
        state.fLat = fLat;
        state.lat  = static_cast<int32_t>(fLat * 10000000);
    }
    if (strlen(lon_str) > 0 && lon_ew) {
        state.fLon = fLon;
        state.lon  = static_cast<int32_t>(fLon * 10000000);
    }
    state.hMSL = static_cast<int32_t>(altitude);

    return true;
}

bool GNSS::parseGNRMC(const char* sentence) {
    // $GNRMC,hhmmss.ss,A/V,ddmm.mmmmm,N/S,dddmm.mmmmm,E/W,s.s,c.c,ddmmyy,...
    char time_str[16]{}, lat_str[16]{}, lon_str[16]{}, date_str[16]{};
    char status{}, lat_ns{}, lon_ew{};
    float speed{}, course{};

    int n = sscanf(sentence, "$GNRMC,%[^,],%c,%[^,],%c,%[^,],%c,%f,%f,%[^,]",
                   time_str, &status, lat_str, &lat_ns, lon_str, &lon_ew,
                   &speed, &course, date_str);

    if (n < 9) return false;

    bool active = (status == 'A');

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (strlen(date_str) >= 6) {
        state.day   = (date_str[0] - '0') * 10 + (date_str[1] - '0');
        state.month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
        state.year  = 2000u + (date_str[4] - '0') * 10 + (date_str[5] - '0');
    }

    state.gSpeed  = static_cast<int32_t>(speed * 1.151f); // knots → mph
    state.headMot = static_cast<int32_t>(course);

    ESP_LOGI(TAG, "RMC status=%c speed=%.1fkn course=%.1f° date=%02d/%02d/%04d",
             status, speed, course, state.day, state.month, state.year);

    if (active) {
        ESP_LOGI(TAG, "Fix active: %.6f° %.6f°", state.fLat, state.fLon);
        ESP_LOGI(TAG, "Time: %02d:%02d:%02d  Date: %02d/%02d/%04d",
                 state.hour, state.min, state.sec,
                 state.day, state.month, state.year);
    }

    return active;
}

void GNSS::parseGSV(const char* sentence) {
    int total_msgs{}, msg_num{}, total_sats{};
    if (sscanf(sentence, "$%*2cGSV,%d,%d,%d", &total_msgs, &msg_num, &total_sats) == 3) {
        ESP_LOGD(TAG, "GSV total_sats=%d", total_sats);
    }
}

bool GNSS::queryUniqueID() {
    static const uint8_t poll_msg[] = { 0xB5, 0x62, 0x27, 0x03, 0x00, 0x00, 0x2A, 0x98 };

    uart_flush_input(NEO_UART_PORT);
    uart_write_bytes(NEO_UART_PORT, poll_msg, sizeof(poll_msg));

    // Read bytes until we find 0xB5 0x62 sync or timeout
    uint8_t resp[17] = {};
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    int filled = 0;

    while (xTaskGetTickCount() < deadline) {
        uint8_t b;
        if (uart_read_bytes(NEO_UART_PORT, &b, 1, pdMS_TO_TICKS(10)) != 1) continue;

        if (filled == 0 && b != 0xB5) continue; // scan for sync1
        if (filled == 1 && b != 0x62) { filled = 0; continue; } // sync2

        resp[filled++] = b;
        if (filled == 17) break;
    }

    if (filled < 17) {
        ESP_LOGE(TAG, "UBX-SEC-UNIQID: timeout (%d bytes)", filled);
        return false;
    }

    if (resp[2] != 0x27 || resp[3] != 0x03 || resp[4] != 0x09 || resp[5] != 0x00) {
        ESP_LOGE(TAG, "UBX-SEC-UNIQID: wrong class/id/len");
        return false;
    }

    // resp[6] = version (0x01), resp[7..11] = 5-byte UID
    memcpy(state.uniqueID, &resp[7], 5);
    ESP_LOGI(TAG, "F9P UID: %02X %02X %02X %02X %02X",
             state.uniqueID[0], state.uniqueID[1], state.uniqueID[2],
             state.uniqueID[3], state.uniqueID[4]);
    return true;
}

GNSS& GNSS::instance() {
    static GNSS instance;
    return instance;
}
