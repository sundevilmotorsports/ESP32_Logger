#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// UART Configuration
#define UART_PORT UART_NUM_0
#define UART_BUF_SIZE 1024
#define RD_BUF_SIZE 128

// External variables
extern SemaphoreHandle_t char_mutex;
extern QueueHandle_t uart_event_queue;

// Function declarations
esp_err_t uart_init(void);
void uart_input_task(void *pvParameters);
void uart_output_task(void *param);
esp_err_t uart_create_tasks(void);
void uart_deinit(void);

// Utility functions
char uart_get_last_char(void);
void uart_clear_last_char(void);

#ifdef __cplusplus
}
#endif

#endif // UART_H