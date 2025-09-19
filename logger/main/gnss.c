#include "gnss.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define NEO_UART_PORT UART_NUM_1
#define NEO_TX_PIN    17
#define NEO_RX_PIN    16
#define NEO_UART_BUF_SIZE 1024
#define NEO_RD_BUF_SIZE 256

static QueueHandle_t neo_uart_event_queue = NULL;

static void neo_uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200, // NEO-F9P default
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    // Enable DMA by setting rx_buffer_size > 0
    uart_driver_install(NEO_UART_PORT, NEO_UART_BUF_SIZE, 0, 20, &neo_uart_event_queue, 0);
    uart_param_config(NEO_UART_PORT, &uart_config);
    uart_set_pin(NEO_UART_PORT, NEO_TX_PIN, NEO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI("NEO_UART", "NEO-F9P UART initialized (DMA, interrupt mode)");
}

static void neo_uart_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data[NEO_RD_BUF_SIZE];

    ESP_LOGI("NEO_UART", "NEO-F9P UART event task started");
    while (1) {
        // Wait for UART event (interrupt-driven)
        if (xQueueReceive(neo_uart_event_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    int len = uart_read_bytes(NEO_UART_PORT, data, event.size, portMAX_DELAY);
                    if (len > 0) {
                        // Print received NMEA sentences
                        fwrite(data, 1, len, stdout);
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    ESP_LOGW("NEO_UART", "UART FIFO overflow");
                    uart_flush_input(NEO_UART_PORT);
                    xQueueReset(neo_uart_event_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW("NEO_UART", "UART buffer full");
                    uart_flush_input(NEO_UART_PORT);
                    xQueueReset(neo_uart_event_queue);
                    break;
                default:
                    ESP_LOGD("NEO_UART", "Other UART event: %d", event.type);
                    break;
            }
        }
    }
}