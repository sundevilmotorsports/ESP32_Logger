#include "gnss.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"
#include "driver/gpio.h"

#define NEO_UART_PORT UART_NUM_1
#define NEO_TX_PIN    GPIO_NUM_19
#define NEO_RX_PIN    GPIO_NUM_20
#define NEO_UART_BUF_SIZE 1024
#define NEO_RD_BUF_SIZE 256

static const char *TAG = "GNSS_DMA";
static QueueHandle_t neo_uart_event_queue = NULL;
static TaskHandle_t gnss_task_handle = NULL;
static uint8_t *dma_buffer = NULL;

GNSS_StateHandle GNSS_Handle = {0};

void gnss_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 38400,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "Installing UART driver...");
    // Install UART driver with DMA support
    ESP_ERROR_CHECK(uart_driver_install(NEO_UART_PORT, GNSS_DMA_BUF_SIZE, 0, 20, &neo_uart_event_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(NEO_UART_PORT, &uart_config));

    esp_err_t pin_result = uart_set_pin(NEO_UART_PORT, NEO_TX_PIN, NEO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (pin_result != ESP_OK) {
        ESP_LOGW(TAG, "UART pin config returned error: %s", esp_err_to_name(pin_result));
        ESP_LOGW(TAG, "Continuing anyway - UART may still work on some boards");
    }

    ESP_LOGI(TAG, "UART configured successfully at 38400 baud");

    // Enable pattern detection for NMEA sentence end ('\n')
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(NEO_UART_PORT, GNSS_PATTERN_CHR, 1, 9, 0, 0));

    dma_buffer = (uint8_t*)heap_caps_malloc(GNSS_DMA_BUF_SIZE, MALLOC_CAP_DMA);
    if (!dma_buffer) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer");
        return;
    }

    ESP_LOGI(TAG, "GPS UART initialization complete");
}

static bool parse_gngga(const char* sentence, GNSS_StateHandle* gps) {
    // $GNGGA,hhmmss.ss,ddmm.mmmmm,N/S,dddmm.mmmmm,E/W,q,nn,h.h,a.a,M,g.g,M,d.d,nnnn*hh
    char time_str[16] = {0};
    char lat_str[16] = {0}, lat_ns = 0;
    char lon_str[16] = {0}, lon_ew = 0;
    int quality = 0, satellites = 0;
    float hdop = 0.0, altitude = 0.0;

    int parsed = sscanf(sentence, "$GNGGA,%[^,],%[^,],%c,%[^,],%c,%d,%d,%f,%f,M",
                       time_str, lat_str, &lat_ns, lon_str, &lon_ew,
                       &quality, &satellites, &hdop, &altitude);

    if (parsed >= 6) {
        gps->fixType = quality;

        // Parse time (HHMMSS format)
        if (strlen(time_str) >= 6) {
            gps->hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
            gps->min = (time_str[2] - '0') * 10 + (time_str[3] - '0');
            gps->sec = (time_str[4] - '0') * 10 + (time_str[5] - '0');
        }

        // Parse latitude (DDMM.MMMMM format)
        if (strlen(lat_str) > 0 && lat_ns != 0) {
            float lat_deg = atof(lat_str);
            int degrees = (int)(lat_deg / 100);
            float minutes = lat_deg - (degrees * 100);
            gps->fLat = degrees + (minutes / 60.0);
            if (lat_ns == 'S') gps->fLat = -gps->fLat;
            gps->lat = (signed long)(gps->fLat * 10000000); // Convert to micro-degrees
        }

        // Parse longitude (DDDMM.MMMMM format)
        if (strlen(lon_str) > 0 && lon_ew != 0) {
            float lon_deg = atof(lon_str);
            int degrees = (int)(lon_deg / 100);
            float minutes = lon_deg - (degrees * 100);
            gps->fLon = degrees + (minutes / 60.0);
            if (lon_ew == 'W') gps->fLon = -gps->fLon;
            gps->lon = (signed long)(gps->fLon * 10000000); // Convert to micro-degrees
        }

        gps->hMSL = altitude;

        ESP_LOGI(TAG, "GGA: Fix=%d, Sats=%d, Lat=%.6f, Lon=%.6f, Alt=%.1fm",
                 quality, satellites, gps->fLat, gps->fLon, altitude);
        return true;
    }
    return false;
}

static bool parse_gnrmc(const char* sentence, GNSS_StateHandle* gps) {
    // $GNRMC,hhmmss.ss,A/V,ddmm.mmmmm,N/S,dddmm.mmmmm,E/W,s.s,c.c,ddmmyy,d.d,E/W,m*hh
    char time_str[16] = {0};
    char status = 0;
    char lat_str[16] = {0}, lat_ns = 0;
    char lon_str[16] = {0}, lon_ew = 0;
    float speed = 0.0, course = 0.0;
    char date_str[16] = {0};

    int parsed = sscanf(sentence, "$GNRMC,%[^,],%c,%[^,],%c,%[^,],%c,%f,%f,%[^,]",
                       time_str, &status, lat_str, &lat_ns, lon_str, &lon_ew,
                       &speed, &course, date_str);

    if (parsed >= 9) {
        // Parse date (DDMMYY format)
        if (strlen(date_str) >= 6) {
            gps->day = (date_str[0] - '0') * 10 + (date_str[1] - '0');
            gps->month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
            gps->year = 2000 + (date_str[4] - '0') * 10 + (date_str[5] - '0');
        }

        gps->gSpeed = (signed long)(speed * 1.151); // Convert knots to mph
        gps->headMot = course;

        ESP_LOGI(TAG, "RMC: Status=%c, Speed=%.1fkn, Course=%.1f°, Date=%02d/%02d/%04d",
                 status, speed, course, gps->day, gps->month, gps->year);
        return (status == 'A'); // Return true if fix is active
    }
    return false;
}

static void parse_gsv_satellites(const char* sentence) {
    // $GPGSV,total_msgs,msg_num,total_sats,sat1_prn,sat1_elev,sat1_azim,sat1_snr,...*hh
    int total_msgs, msg_num, total_sats;

    int parsed = sscanf(sentence, "$%*2cGSV,%d,%d,%d", &total_msgs, &msg_num, &total_sats);

    if (parsed == 3) {
        char constellation[3] = {0};
        strncpy(constellation, sentence + 1, 2);
        // ESP_LOGI(TAG, "GSV %s: Total satellites in view: %d", constellation, total_sats);
    }
}

static void process_nmea_sentence(const char* sentence, size_t len) {
    if (!sentence || len == 0) {
        ESP_LOGW(TAG, "Invalid NMEA sentence: null or empty");
        return;
    }

    char nmea_line[512];
    if (len >= sizeof(nmea_line)) {
        ESP_LOGW(TAG, "NMEA sentence too long (%d bytes), truncating", len);
        len = sizeof(nmea_line) - 1;
    }
    memcpy(nmea_line, sentence, len);
    nmea_line[len] = '\0';

    while (len > 0 && (nmea_line[len-1] == '\r' || nmea_line[len-1] == '\n' || nmea_line[len-1] == ' ')) {
        nmea_line[--len] = '\0';
    }

    // Validate NMEA format (should start with $)
    if (len > 0 && nmea_line[0] == '$') {
        ESP_LOGD(TAG, "GPS: %s", nmea_line);

        if (strncmp(nmea_line, "$GNGGA", 6) == 0) {
            parse_gngga(nmea_line, &GNSS_Handle);
        } else if (strncmp(nmea_line, "$GNRMC", 6) == 0) {
            bool fix_active = parse_gnrmc(nmea_line, &GNSS_Handle);
            if (fix_active) {
                ESP_LOGI(TAG, "gps fix");
                ESP_LOGI(TAG, "Location: %.6f°, %.6f°", GNSS_Handle.fLat, GNSS_Handle.fLon);
                ESP_LOGI(TAG, "Time: %02d:%02d:%02d Date: %02d/%02d/%04d",
                         GNSS_Handle.hour, GNSS_Handle.min, GNSS_Handle.sec,
                         GNSS_Handle.day, GNSS_Handle.month, GNSS_Handle.year);
            }
        } else if (strstr(nmea_line, "GSV") != NULL) {
            parse_gsv_satellites(nmea_line);
        } else if (strncmp(nmea_line, "$GNGSA", 6) == 0) {
            ESP_LOGD(TAG, "GSA: DOP and active satellites info");
        } else if (strncmp(nmea_line, "$GNVTG", 6) == 0) {
            ESP_LOGD(TAG, "VTG: Track made good and ground speed");
        } else if (strncmp(nmea_line, "$GNGLL", 6) == 0) {
            ESP_LOGD(TAG, "GLL: Geographic position - latitude/longitude");
        }
    } else {
        ESP_LOGD(TAG, "Non-NMEA data (%d bytes): %.*s", len, (int)len, nmea_line);
    }
}

static void neo_uart_task(void *pvParameters) {
    uart_event_t event;
    size_t buffered_size;

    ESP_LOGI(TAG, "NEO-F9P DMA UART task started");

    while (1) {
        if (xQueueReceive(neo_uart_event_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    ESP_LOGD(TAG, "Received %d bytes via UART", event.size);
                    break;

                case UART_PATTERN_DET:
                    // Pattern detected - complete NMEA sentence received
                    uart_get_buffered_data_len(NEO_UART_PORT, &buffered_size);
                    ESP_LOGD(TAG, "Pattern detected, buffered size: %d", buffered_size);

                    if (buffered_size > 0) {
                        int pos = uart_pattern_pop_pos(NEO_UART_PORT);
                        if (pos != -1 && pos < GNSS_DMA_BUF_SIZE - 1) {
                            // Clear buffer before reading
                            memset(dma_buffer, 0, GNSS_DMA_BUF_SIZE);

                            // Read up to the pattern position + 1 (including the newline)
                            int read_len = uart_read_bytes(NEO_UART_PORT, dma_buffer, pos + 1, 100 / portTICK_PERIOD_MS);
                            if (read_len > 0) {
                                ESP_LOGD(TAG, "Read %d bytes, processing NMEA sentence", read_len);
                                // Process the complete NMEA sentence
                                process_nmea_sentence((char*)dma_buffer, read_len - 1); // -1 to exclude newline
                            } else {
                                ESP_LOGW(TAG, "Failed to read data after pattern detection");
                            }
                        } else {
                            uart_flush_input(NEO_UART_PORT);
                            ESP_LOGW(TAG, "Pattern queue full or invalid position (%d), flushing buffer", pos);
                        }
                    } else {
                        ESP_LOGW(TAG, "Pattern detected but no buffered data");
                    }
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow - flushing");
                    uart_flush_input(NEO_UART_PORT);
                    xQueueReset(neo_uart_event_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART ring buffer full - flushing");
                    uart_flush_input(NEO_UART_PORT);
                    xQueueReset(neo_uart_event_queue);
                    break;

                case UART_BREAK:
                    ESP_LOGW(TAG, "UART RX break detected");

                    int rx_level = gpio_get_level(NEO_RX_PIN);
                    int tx_level = gpio_get_level(NEO_TX_PIN);
                    ESP_LOGW(TAG, "GPIO levels: RX=%d, TX=%d", rx_level, tx_level);

                    uart_flush_input(NEO_UART_PORT);
                    xQueueReset(neo_uart_event_queue);
                    vTaskDelay(pdMS_TO_TICKS(10));
                    break;

                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "UART parity error");
                    break;

                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "UART frame error");
                    break;

                default:
                    ESP_LOGD(TAG, "Other UART event: %d", event.type);
                    break;
            }
        }
    }
}

void gnss_start_task(void) {
    if (gnss_task_handle == NULL) {
        xTaskCreate(neo_uart_task, "gnss_uart_task", 4096, NULL, 10, &gnss_task_handle);
    }
}

void gnss_stop(void) {
    if (gnss_task_handle != NULL) {
        vTaskDelete(gnss_task_handle);
        gnss_task_handle = NULL;
    }

    if (neo_uart_event_queue != NULL) {
        uart_driver_delete(NEO_UART_PORT);
        neo_uart_event_queue = NULL;
    }

    if (dma_buffer != NULL) {
        free(dma_buffer);
        dma_buffer = NULL;
    }
}
