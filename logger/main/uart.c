#include "uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include <string.h>
#include <inttypes.h>
#include "dtc.h"

static const char *TAG = "UART_MODULE";

// Global variables
SemaphoreHandle_t char_mutex = NULL;
QueueHandle_t uart_event_queue = NULL;

// Private variables
static char last_char = '\0';
static TaskHandle_t dtc_info_task_handle = NULL;
static bool dtc_info_running = false;

// Private function declarations
static void print_dtc_info(void *pvParameters);
static void print_cpu_usage(void);

esp_err_t uart_init(void) {
    ESP_LOGI(TAG, "Initializing UART module");
    
    // Configure UART parameters
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Install UART driver with event queue
    esp_err_t ret = uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 20, &uart_event_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_param_config(UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create mutex for character sharing between tasks
    char_mutex = xSemaphoreCreateMutex();
    if (char_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create character mutex");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "UART initialized successfully");
    return ESP_OK;
}

esp_err_t uart_create_tasks(void) {
    BaseType_t result;

    result = xTaskCreate(uart_input_task, "uart_input", 4096, NULL, 10, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create uart_input_task");
        return ESP_FAIL;
    }

    result = xTaskCreate(uart_output_task, "uart_output", 4096, NULL, 5, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create uart_output_task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UART tasks created successfully");
    return ESP_OK;
}

void uart_input_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data[RD_BUF_SIZE];
    
    ESP_LOGI(TAG, "UART event task started");
    
    while (1) {
        // Wait for UART event
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    int len = uart_read_bytes(UART_PORT, data, event.size, portMAX_DELAY);
                    if (len > 0) {
                        // Store the first character for processing
                        xSemaphoreTake(char_mutex, portMAX_DELAY);
                        last_char = data[0];
                        xSemaphoreGive(char_mutex);
                        
                        // Echo the character back (optional)
                        // uart_write_bytes(UART_PORT, (const char*)data, 1);
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow");
                    uart_flush_input(UART_PORT);
                    xQueueReset(uart_event_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART buffer full");
                    uart_flush_input(UART_PORT);
                    xQueueReset(uart_event_queue);
                    break;
                default:
                    ESP_LOGD(TAG, "Other UART event: %d", event.type);
                    break;
            }
        }
    }
}

void uart_output_task(void *param) {
    ESP_LOGI(TAG, "Command processor task started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
        
        // Get the last received character
        xSemaphoreTake(char_mutex, portMAX_DELAY);
        char input = last_char;
        last_char = '\0';  // Clear after consuming
        xSemaphoreGive(char_mutex);
        
        if (input != '\0') {
            ESP_LOGI(TAG, "Processing input: %c", input);

            // Stop DTC info task if it's running and a new command is issued (except 'd'/'D')
            if (dtc_info_running && (input != 'd' && input != 'D')) {
                dtc_info_running = false;
                if (dtc_info_task_handle != NULL) {
                    vTaskDelay(pdMS_TO_TICKS(150)); // Give task time to stop
                }
                printf("\033[2J\033[H"); // Clear screen
            }
            
            switch (input) {
                case '1':
                    printf("=== Option 1: System Status ===\n");
                    printf("System: Running\n");
                    printf("Free heap: %ld bytes\n", esp_get_free_heap_size());
                    printf("Uptime: %lld ms\n", esp_timer_get_time() / 1000);
                    break;
                    
                case '2':
                    printf("=== Option 2: Toggle LED ===\n");
                    // Add your LED toggle code here
                    printf("LED toggled!\n");
                    break;
                    
                case '3':
                    printf("=== Option 3: Start CAN Logging ===\n");
                    // Uncomment your twai_init() in app_main if needed
                    printf("CAN logging would start here\n");
                    break;
                    
                case '4':
                    printf("=== Option 4: Memory Info ===\n");
                    printf("Total heap size: %d bytes\n", heap_caps_get_total_size(MALLOC_CAP_8BIT));
                    printf("Free heap size: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
                    printf("Largest free block: %d bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
                    break;
                    
                case '5':  // New option for CPU usage
                    printf("=== Option 5: CPU Usage ===\n");
                    print_cpu_usage();
                    break;
                    
                case 'r':
                case 'R':
                    printf("=== Restarting ESP32 ===\n");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                    break;
                    
                case 'd':
                case 'D':
                    if(!dtc_info_running && dtc_info_task_handle == NULL) {
                        printf("=== Starting DTC Information Display ===\n");
                        printf("Press any other key to stop...\n");
                        vTaskDelay(pdMS_TO_TICKS(500));
                        
                        dtc_info_running = true;
                        BaseType_t result = xTaskCreate(print_dtc_info, "dtc_info_display", 4096, NULL, 7, &dtc_info_task_handle);
                        if (result != pdPASS) {
                            ESP_LOGE(TAG, "Failed to create DTC info task");
                            dtc_info_running = false;
                        }
                    } else if (dtc_info_running) {
                        printf("=== Stopping DTC Information Display ===\n");
                        dtc_info_running = false;
                        // Wait for task to clean up
                        while (dtc_info_task_handle != NULL) {
                            vTaskDelay(pdMS_TO_TICKS(50));
                        }
                    } else {
                        printf("DTC display is already starting/stopping...\n");
                    }
                    break;
                    
                case 'h':
                case 'H':
                case '?':
                    printf("\n=== ESP32 UART Command Menu ===\n");
                    printf("1 - Show system status\n");
                    printf("2 - Toggle LED\n");
                    printf("3 - Start CAN logging\n");
                    printf("4 - Show memory info\n");
                    printf("5 - Show CPU usage\n");
                    printf("D - Toggle DTC info display\n");
                    printf("R - Restart system\n");
                    printf("H - Show this help menu\n");
                    printf("ESC - Clear screen\n");
                    printf("================================\n");
                    break;
                    
                case 27: // ESC key
                    printf("\033[2J\033[H"); // Clear screen and move cursor to top
                    printf("Screen cleared!\n");
                    break;
                    
                case '\r':
                case '\n':
                    // Ignore carriage return and newline
                    break;
                    
                default:
                    if (input >= 32 && input <= 126) { // Printable characters
                        printf("Unknown command: '%c' (0x%02X)\n", input, input);
                        printf("Type 'H' for help menu\n");
                    }
                    break;
            }
        }
    }
}

static void print_dtc_info(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 500 ms
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    char output_buffer[1000];
    char temp[100];

    while(1){
        // Check if task should continue running
        if (!dtc_info_running) {
            break; // Exit the task loop
        }
        
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        printf("\033[2J\033[H"); // Clear screen and move cursor to top
        size_t offset = snprintf(output_buffer, sizeof(output_buffer),
            "\n=== DTC Information ===\n\n%-20s %-10s %10s\n===============================================\n",
            "Device", "Status", "Time (ms)");

        for (int i = 0; i < DTC_COUNT; i++) {
            int written = snprintf(temp, sizeof(temp), "%-20s %-10s %10llu\n",
                dtc_device_names[i] ? dtc_device_names[i] : "UNKNOWN",
                dtc_devices[i] ? (dtc_devices[i]->errState ? "OK" : "ERROR") : "N/A",
                dtc_devices[i] ? (pdMS_TO_TICKS(xTaskGetTickCount()) - dtc_devices[i]->prevTime) : 0);

            if (offset + written < sizeof(output_buffer)) {
                strncat(output_buffer, temp, sizeof(output_buffer) - offset - 1);
                offset += written;
            }
        }
        strcat(output_buffer, "===============================================\n");
        printf("%s", output_buffer);
    }

    // Task is ending, clean up
    dtc_info_task_handle = NULL;
    vTaskDelete(NULL);
}

static void print_cpu_usage(void) {
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint64_t ulTotalTime, ulStatsAsPercentage;
    
    // Get number of tasks
    uxArraySize = uxTaskGetNumberOfTasks();
    
    // Allocate array for task status
    pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
    
    if (pxTaskStatusArray != NULL) {
        // Generate the status information
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalTime);
        
        // Check if we have valid timing data
        if (ulTotalTime == 0) {
            printf("Error: No timing data available. Enable FreeRTOS runtime stats.\n");
            vPortFree(pxTaskStatusArray);
            return;
        }
        
        printf("\nTask Name\t\tRun Time\t\tPercentage\n");
        printf("==============================================\n");
        
        // Find IDLE task runtime (CPU downtime)
        uint64_t idle_time = 0;
        
        for (x = 0; x < uxArraySize; x++) {
            if (strstr(pxTaskStatusArray[x].pcTaskName, "IDLE") != NULL) {
                idle_time = pxTaskStatusArray[x].ulRunTimeCounter;
            }
            
            // Calculate percentage with safety check
            ulStatsAsPercentage = (pxTaskStatusArray[x].ulRunTimeCounter * 100ULL) / ulTotalTime;
            
            printf("%s\t\t%llu\t\t%llu%%\n", 
                   pxTaskStatusArray[x].pcTaskName, 
                   pxTaskStatusArray[x].ulRunTimeCounter, 
                   ulStatsAsPercentage);
        }

        uint64_t idle_percentage = (idle_time * 100ULL) / ulTotalTime;
        printf("\nTotal CPU Idle Time: %" PRIu64 " ticks (%" PRIu64 "%%)\n", idle_time, idle_percentage);
        printf("Total Runtime: %llu ticks\n", ulTotalTime);
        
        vPortFree(pxTaskStatusArray);
    } else {
        printf("Error: Failed to allocate memory for task status array\n");
    }
}

// Utility functions
char uart_get_last_char(void) {
    char ch = '\0';
    if (char_mutex != NULL) {
        xSemaphoreTake(char_mutex, portMAX_DELAY);
        ch = last_char;
        xSemaphoreGive(char_mutex);
    }
    return ch;
}

void uart_clear_last_char(void) {
    if (char_mutex != NULL) {
        xSemaphoreTake(char_mutex, portMAX_DELAY);
        last_char = '\0';
        xSemaphoreGive(char_mutex);
    }
}

void uart_deinit(void) {
    if (char_mutex != NULL) {
        vSemaphoreDelete(char_mutex);
        char_mutex = NULL;
    }
    
    if (uart_event_queue != NULL) {
        uart_driver_delete(UART_PORT);
        uart_event_queue = NULL;
    }
    
    ESP_LOGI(TAG, "UART module deinitialized");
}