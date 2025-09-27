#include "gnss.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"

#define NEO_UART_PORT UART_NUM_1
#define NEO_TX_PIN    17
#define NEO_RX_PIN    16
#define NEO_UART_BUF_SIZE 1024
#define NEO_RD_BUF_SIZE 256

static const char *TAG = "GNSS_DMA";
static QueueHandle_t neo_uart_event_queue = NULL;
static TaskHandle_t gnss_task_handle = NULL;
static uint8_t *dma_buffer = NULL;

GNSS_StateHandle GNSS_Handle = {0};

static void neo_uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200, // NEO-F9P default
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver with DMA support
    // RX buffer size > 0 enables DMA, TX buffer = 0 disables TX DMA
    ESP_ERROR_CHECK(uart_driver_install(NEO_UART_PORT, GNSS_DMA_BUF_SIZE, 0, 20, &neo_uart_event_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(NEO_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(NEO_UART_PORT, NEO_TX_PIN, NEO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Enable pattern detection for NMEA sentence end ('\n')
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(NEO_UART_PORT, GNSS_PATTERN_CHR, 1, 9, 0, 0));

    dma_buffer = (uint8_t*)heap_caps_malloc(GNSS_DMA_BUF_SIZE, MALLOC_CAP_DMA);
    if (!dma_buffer) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer");
        return;
    }
}

static void process_nmea_sentence(const char* sentence, size_t len) {
    char nmea_line[512];
    if (len >= sizeof(nmea_line)) {
        len = sizeof(nmea_line) - 1;
    }
    memcpy(nmea_line, sentence, len);
    nmea_line[len] = '\0';

    ESP_LOGD(TAG, "NMEA: %s", nmea_line);

    printf("%s\n", nmea_line);
}

static void neo_uart_task(void *pvParameters) {
    uart_event_t event;
    size_t buffered_size;

    ESP_LOGI(TAG, "NEO-F9P DMA UART task started");

    while (1) {
        // Wait for UART event
        if (xQueueReceive(neo_uart_event_queue, &event, portMAX_DELAY)) {
            bzero(dma_buffer, GNSS_DMA_BUF_SIZE);

            switch (event.type) {
                case UART_DATA:
                    // Read available data
                    uart_read_bytes(NEO_UART_PORT, dma_buffer, event.size, portMAX_DELAY);
                    ESP_LOGD(TAG, "Received %d bytes via DMA", event.size);
                    break;

                case UART_PATTERN_DET:
                    // Pattern detected - complete NMEA sentence received
                    uart_get_buffered_data_len(NEO_UART_PORT, &buffered_size);

                    if (buffered_size > 0) {
                        int pos = uart_pattern_pop_pos(NEO_UART_PORT);
                        if (pos != -1) {
                            // Read up to the pattern position + 1 (including the newline)
                            int read_len = uart_read_bytes(NEO_UART_PORT, dma_buffer, pos + 1, 100 / portTICK_PERIOD_MS);
                            if (read_len > 0) {
                                // Process the complete NMEA sentence
                                process_nmea_sentence((char*)dma_buffer, read_len - 1); // -1 to exclude newline
                            }
                        } else {
                            // Pattern queue full, flush it
                            uart_flush_input(NEO_UART_PORT);
                            ESP_LOGW(TAG, "Pattern queue full, flushing buffer");
                        }
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
