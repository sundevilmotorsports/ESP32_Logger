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

static void neo_uart_init(void) {
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
        ESP_LOGI(TAG, "GPS: %s", nmea_line);
        printf("GPS: %s\n", nmea_line);
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

void gnss_simple_test(void) {
    ESP_LOGI(TAG, "Starting simple GPS polling test...");

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(NEO_UART_PORT, &uart_config);
    uart_set_pin(NEO_UART_PORT, NEO_TX_PIN, NEO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    char data[128];
    int len;

    ESP_LOGI(TAG, "Listening for GPS data (polling mode)...");
    for (int i = 0; i < 100; i++) { // Try for 10 seconds
        len = uart_read_bytes(NEO_UART_PORT, data, sizeof(data) - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "GPS Data: %s", data);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "GPS polling test completed");
}

void gnss_init(void) {
    neo_uart_init();
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
