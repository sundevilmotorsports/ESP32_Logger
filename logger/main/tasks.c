#include "tasks.h"

#include "can.h"
#include "dtc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gnss.h"
#include "uart.h"

static const char *TAG = "TASKS";

#define TASK_STACK_CAN_RX 4096
#define TASK_STACK_DTC_CHECK 2048
#define TASK_STACK_GNSS_UART 4096
#define TASK_STACK_UART_INPUT 4096
#define TASK_STACK_UART_OUTPUT 8192
#define TASK_STACK_LOG_BUFFER 4096

#define TASK_PRIO_CAN_RX 12
#define TASK_PRIO_LOG_BUFFER 9
#define TASK_PRIO_GNSS_UART 8
#define TASK_PRIO_UART_INPUT 7
#define TASK_PRIO_UART_OUTPUT 6
#define TASK_PRIO_DTC_CHECK 5

extern void logBuffer_task(void *pvParameters);

esp_err_t tasks_start_all(void) {
    esp_err_t status = ESP_OK;
    BaseType_t result;

    result = xTaskCreate(can_receive_task, "can_rx", TASK_STACK_CAN_RX, NULL, TASK_PRIO_CAN_RX, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create can_receive_task");
        status = ESP_FAIL;
    }

    result = xTaskCreate(logBuffer_task, "log buffer", TASK_STACK_LOG_BUFFER, NULL, TASK_PRIO_LOG_BUFFER, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create logBuffer_task");
        status = ESP_FAIL;
    }

    if (gnss_task_handle == NULL) {
        result = xTaskCreate(gnss_uart_task, "gnss_uart_task", TASK_STACK_GNSS_UART, NULL, TASK_PRIO_GNSS_UART, &gnss_task_handle);
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create gnss_uart_task");
            status = ESP_FAIL;
        }
    }

    result = xTaskCreate(uart_input_task, "uart_input", TASK_STACK_UART_INPUT, NULL, TASK_PRIO_UART_INPUT, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create uart_input_task");
        status = ESP_FAIL;
    }

    result = xTaskCreate(uart_output_task, "uart_output", TASK_STACK_UART_OUTPUT, NULL, TASK_PRIO_UART_OUTPUT, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create uart_output_task");
        status = ESP_FAIL;
    }

    result = xTaskCreate(dtc_task, "dtc_check", TASK_STACK_DTC_CHECK, NULL, TASK_PRIO_DTC_CHECK, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create dtc_task");
        status = ESP_FAIL;
    }

    if (status == ESP_OK) {
        ESP_LOGI(TAG, "All tasks created successfully");
    }

    return status;
}
