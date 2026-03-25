#include "uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include <string.h>
#include <inttypes.h>
#include "dtc.h"
#include "sdcard.h"
#include "esp_task_wdt.h"
#include "log_chnl.h"
#include "can.h"


static const char *TAG = "UART_MODULE";

// Global variables
SemaphoreHandle_t char_mutex = NULL;
QueueHandle_t uart_event_queue = NULL;

// Private variables
static char last_char = '\0';
static TaskHandle_t dtc_info_task_handle = NULL;
static bool dtc_info_running = false;

static bool uart_input_mode = false;
static QueueHandle_t uart_cmd_queue = NULL;
static QueueHandle_t uart_input_queue = NULL;

#define UART_CMD_QUEUE_LEN 64
#define UART_INPUT_QUEUE_LEN 128

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

    uart_cmd_queue = xQueueCreate(UART_CMD_QUEUE_LEN, sizeof(char));
    if (uart_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create UART command queue");
        return ESP_FAIL;
    }

    uart_input_queue = xQueueCreate(UART_INPUT_QUEUE_LEN, sizeof(char));
    if (uart_input_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create UART input queue");
        vQueueDelete(uart_cmd_queue);
        uart_cmd_queue = NULL;
        return ESP_FAIL;
    }

    
    ESP_LOGI(TAG, "UART initialized successfully");
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
                        bool input_mode = false;
                        if (char_mutex != NULL) {
                            xSemaphoreTake(char_mutex, portMAX_DELAY);
                            input_mode = uart_input_mode;
                            if (!input_mode) {
                                last_char = data[0];
                            }
                            xSemaphoreGive(char_mutex);
                        }

                        QueueHandle_t target_queue = input_mode ? uart_input_queue : uart_cmd_queue;
                        if (target_queue != NULL) {
                            for (int i = 0; i < len; i++) {
                                char ch = (char)data[i];
                                (void)xQueueSend(target_queue, &ch, 0);
                            }
                        }
                        
                        // Echo characters back in normal mode (optional)
                        if (!input_mode) {
                            // uart_write_bytes(UART_PORT, (const char*)data, 1);
                        }
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
        if (uart_cmd_queue == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        char input = '\0';
        if (xQueueReceive(uart_cmd_queue, &input, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (input != '\0') {
            // ESP_LOGI(TAG, "Processing input: %c", input);

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
                    
                case 'a':
                case 'A':
                    printf("=== Option A: Report Analog ===\n");
                    printf("Front Brake Pressure: %u\n", (logBuffer[F_BRAKEPRESSURE1] << 8) | logBuffer[F_BRAKEPRESSURE]);
                    printf("Rear Brake Pressure:  %u\n", (logBuffer[R_BRAKEPRESSURE1] << 8) | logBuffer[R_BRAKEPRESSURE]);
                    printf("Steering Position:    %u\n", (logBuffer[STEERING1] << 8) | logBuffer[STEERING]);
                    printf("Front Left Shock:     %u\n", (logBuffer[FLSHOCK1] << 8) | logBuffer[FLSHOCK]);
                    printf("Front Right Shock:    %u\n", (logBuffer[FRSHOCK1] << 8) | logBuffer[FRSHOCK]);
                    printf("Rear Left Shock:      %u\n", (logBuffer[RLSHOCK1] << 8) | logBuffer[RLSHOCK]);
                    printf("Rear Right Shock:     %u\n", (logBuffer[RRSHOCK1] << 8) | logBuffer[RRSHOCK]);
                    break;
                
                case 'c':
                case 'C':
                    printf("=== Option C: CAN Message Debug ===\n");
                    printf("# of msgs: %lu\n", can_msg_count);
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

                case 'f':
                case 'F':
                    printf("=== Option F: Change Log File Name ===\n");
                    printf("Type your input and press Enter to submit, or ESC to cancel.\n");
                    
                    
                    char user_input[MAX_FILE_NAME_LENGTH >> 1];
                    esp_err_t input_result = uart_get_user_input(user_input, sizeof(user_input), "Input: ", 30000, true);
                    
                    switch (input_result) {
                        case ESP_OK:
                            nvs_set_log_name(user_input);
                            nvs_set_testno(0);
                            sdcard_create_numbered_log_file(user_input);
                            printf("Starting new logfile: '%s'\n", sdcard_get_current_log_filepath());
                            break;
                            
                        case ESP_ERR_TIMEOUT:
                            printf("Input timed out after 30 seconds.\n");
                            break;
                            
                        case ESP_ERR_INVALID_RESPONSE:
                            printf("Input cancelled by user (ESC pressed).\n");
                            break;
                            
                        default:
                            printf("Input failed with error: %s\n", esp_err_to_name(input_result));
                            break;
                    }
                    
                    printf("Returning to command mode...\n");
                    break;
                case 'i':
                case 'I':
                    printf("=== Option I: Increment Log File ===\n");
                    char buffer[32];
                    nvs_get_log_name(buffer, sizeof(buffer));
                    sdcard_create_numbered_log_file(buffer);
                    printf("Starting new logfile: '%s'\n", sdcard_get_current_log_filepath());
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
                    printf("A - Report analog values\n");
                    printf("C - CAN message debug\n");
                    printf("D - Toggle DTC info display\n");
                    printf("F - Change log file name\n");
                    printf("G - Display GPS info\n");
                    printf("I - Increment log file number\n");
                    printf("R - Restart system\n");
                    printf("S - Display steering info\n");
                    printf("T - Display IMU data\n");
                    printf("W - Display wheel board info\n");
                    printf("X - Display strain gauge info\n");
                    printf("H - Show this help menu\n");
                    printf("ESC - Clear screen\n");
                    printf("================================\n");
                    break;
                case 'g':
                case 'G':
                    // Reconstruct little-endian 32-bit signed values from logBuffer
                    int32_t gps_lon = (int32_t)(
                        (uint32_t)logBuffer[GPS_LON] |
                        ((uint32_t)logBuffer[GPS_LON1] << 8) |
                        ((uint32_t)logBuffer[GPS_LON2] << 16) |
                        ((uint32_t)logBuffer[GPS_LON3] << 24)
                        );
                    int32_t gps_lat = (int32_t)(
                        (uint32_t)logBuffer[GPS_LAT] |
                        ((uint32_t)logBuffer[GPS_LAT1] << 8) |
                        ((uint32_t)logBuffer[GPS_LAT2] << 16) |
                        ((uint32_t)logBuffer[GPS_LAT3] << 24)
                    );
                    printf("=== Display GPS Info ===\n");
                    printf("GPS -> LON: %ld\tLAT: %ld\tFIX: %d\n", gps_lon, gps_lat, logBuffer[GPS_FIX]);
                    break;
                case 'r':
                case 'R':
                    printf("=== Restarting ESP32 ===\n");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                    break;
                case 's':
                case 'S':
                    float angle = (0.084769f) * (float)((uint16_t)(logBuffer[STEERING1] << 8 | logBuffer[STEERING])) - (152.846451f);                  
                    printf("=== Display Steering Info ===\n");
                    printf("Steer Angle: %.2f deg\r\n", angle);
                    break;

                case 't':
                case 'T':
                    printf("=== Display IMU Data ===\n");
                    printf("IMU Accel -> X: %.3f\tY: %.3f\tZ: %.3f\n", (float)((imu_accel.x* 0.122) / 1000), (float)((imu_accel.y* 0.122) / 1000), (float)((imu_accel.z* 0.122) / 1000));
                    printf("IMU Gyro  -> X: %d\tY: %d\tZ: %d\n", imu_gyro.x, imu_gyro.y, imu_gyro.z);
                    break;
                case 'w':
                case 'W':
                    printf("=== Display Wheel Board Info ===\n");
                    /* Recombine 16-bit values stored as two bytes in logBuffer:
                       high byte at *_1 index, low byte at base index (see main.c usage) */
                    uint16_t flw_rpm = (uint16_t)((logBuffer[FLW_RPM1] << 8) | logBuffer[FLW_RPM]);
                    float flw_obj = (float)(((uint16_t)((logBuffer[FLW_OBJ1] << 8) | logBuffer[FLW_OBJ]))*0.02 - 273.15);
                    float flw_amb = (float)(((uint16_t)((logBuffer[FLW_AMB1] << 8) | logBuffer[FLW_AMB]))*0.02 - 273.15);

                    uint16_t frw_rpm = (uint16_t)((logBuffer[FRW_RPM1] << 8) | logBuffer[FRW_RPM]);
                    float frw_obj = (float)(((uint16_t)((logBuffer[FRW_OBJ1] << 8) | logBuffer[FRW_OBJ]))*0.02 - 273.15);
                    float frw_amb = (float)(((uint16_t)((logBuffer[FRW_AMB1] << 8) | logBuffer[FRW_AMB]))*0.02 - 273.15);

                    uint16_t rrw_rpm = (uint16_t)((logBuffer[RRW_RPM1] << 8) | logBuffer[RRW_RPM]);
                    float rrw_obj = (float)(((uint16_t)((logBuffer[RRW_OBJ1] << 8) | logBuffer[RRW_OBJ]))*0.02 - 273.15);
                    float rrw_amb = (float)(((uint16_t)((logBuffer[RRW_AMB1] << 8) | logBuffer[RRW_AMB]))*0.02 - 273.15);

                    uint16_t rlw_rpm = (uint16_t)((logBuffer[RLW_RPM1] << 8) | logBuffer[RLW_RPM]);
                    float rlw_obj = (float)(((uint16_t)((logBuffer[RLW_OBJ1] << 8) | logBuffer[RLW_OBJ]))*0.02 - 273.15);
                    float rlw_amb = (float)(((uint16_t)((logBuffer[RLW_AMB1] << 8) | logBuffer[RLW_AMB]))*0.02 - 273.15);

                    printf("FLW -> RPM: %u\tAMB: %.2f\tOBJ: %.2f\n", flw_rpm, flw_amb, flw_obj);
                    printf("FRW -> RPM: %u\tAMB: %.2f\tOBJ: %.2f\n", frw_rpm, frw_amb, frw_obj);
                    printf("RRW -> RPM: %u\tAMB: %.2f\tOBJ: %.2f\n", rrw_rpm, rrw_amb, rrw_obj);
                    printf("RLW -> RPM: %u\tAMB: %.2f\tOBJ: %.2f\n", rlw_rpm, rlw_amb, rlw_obj);
                    break;
                case 'X':
                case 'x':
                    int16_t flsg = logBuffer[FL_SG1] << 8 | logBuffer[FL_SG];
                    int16_t frsg = logBuffer[FR_SG1] << 8 | logBuffer[FR_SG];
                    int16_t rlsg = logBuffer[RL_SG1] << 8 | logBuffer[RL_SG];
                    int16_t rrsg = logBuffer[RR_SG1] << 8 | logBuffer[RR_SG];
                    double pr1 = (double)(2.4152 * frsg) + (double)(-10973.1053);
                    double pr2 = (double)(1.9029 * frsg) + (double)(-3305.5943);
                    printf(" FRSG: %d\tFLSG: %d\tRRSG: %d\tRLSG: %d\tPR1: %.3f\tPR2: %.3f\r\n", frsg, flsg, rrsg, rlsg, pr1, pr2);
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

esp_err_t uart_get_user_input(char *buffer, size_t buffer_size, const char *prompt, uint32_t timeout_ms, bool echo) {
    if (buffer == NULL || buffer_size == 0) {
        ESP_LOGE(TAG, "Invalid buffer parameters");
        return ESP_ERR_INVALID_ARG;
    }
        // Subscribe current task to TWDT for this function
    bool was_subscribed = (esp_task_wdt_status(NULL) == ESP_OK);
    if (!was_subscribed) {
        esp_task_wdt_add(NULL);  // Add current task
    }
    
    // Clear the buffer
    memset(buffer, 0, buffer_size);
    
    // Show prompt if provided
    if (prompt != NULL) {
        printf("%s", prompt);
        fflush(stdout);
    }
    
    size_t index = 0;
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : 0;
    const TickType_t wdt_feed_interval = pdMS_TO_TICKS(500);
    
    // Enable input mode for better character handling
    if (char_mutex != NULL) {
        xSemaphoreTake(char_mutex, portMAX_DELAY);
        uart_input_mode = true;
        last_char = '\0';  // Clear last_char to prevent interference
        xSemaphoreGive(char_mutex);
    }

    if (uart_input_queue != NULL) {
        char discard = '\0';
        while (xQueueReceive(uart_input_queue, &discard, 0) == pdTRUE) {
        }
    }

    if (uart_cmd_queue != NULL) {
        char discard = '\0';
        while (xQueueReceive(uart_cmd_queue, &discard, 0) == pdTRUE) {
        }
    }
    
    while (1) {
        esp_task_wdt_reset();
        TickType_t wait_ticks = wdt_feed_interval;
        if (timeout_ms > 0) {
            TickType_t elapsed = xTaskGetTickCount() - start_time;
            if (elapsed >= timeout_ticks) {
                ESP_LOGW(TAG, "User input timeout");
                if (char_mutex != NULL) {
                    xSemaphoreTake(char_mutex, portMAX_DELAY);
                    uart_input_mode = false;
                    xSemaphoreGive(char_mutex);
                }
                if (!was_subscribed) {
                    esp_task_wdt_delete(NULL);
                }
                return ESP_ERR_TIMEOUT;
            }
            TickType_t remaining = timeout_ticks - elapsed;
            wait_ticks = (remaining < wdt_feed_interval) ? remaining : wdt_feed_interval;
        }

        char ch = '\0';
        if (uart_input_queue == NULL) {
            ESP_LOGE(TAG, "UART input queue not initialized");
            if (char_mutex != NULL) {
                xSemaphoreTake(char_mutex, portMAX_DELAY);
                uart_input_mode = false;
                xSemaphoreGive(char_mutex);
            }
            if (!was_subscribed) {
                esp_task_wdt_delete(NULL);
            }
            return ESP_FAIL;
        }

        if (xQueueReceive(uart_input_queue, &ch, wait_ticks) == pdTRUE) {
            switch (ch) {
                case '\r':  // Carriage return
                case '\n':  // Line feed (Enter key)
                    if (echo) {
                        printf("\n");
                        fflush(stdout);
                    }
                    buffer[index] = '\0';  // Null terminate
                    ESP_LOGD(TAG, "User input received: '%s' (length: %zu)", buffer, index);
                    
                    // Disable input mode before returning
                    if (char_mutex != NULL) {
                        xSemaphoreTake(char_mutex, portMAX_DELAY);
                        uart_input_mode = false;
                        xSemaphoreGive(char_mutex);
                    }
                    // Cleanup: unsubscribe if we subscribed
                    if (!was_subscribed) {
                        esp_task_wdt_delete(NULL);
                    }
                    return ESP_OK;
                    
                case '\b':  // Backspace
                case 127:   // DEL key
                    if (index > 0) {
                        index--;
                        buffer[index] = '\0';
                        if (echo) {
                            printf("\b \b");  // Move back, print space, move back again
                            fflush(stdout);
                        }
                    }
                    break;
                    
                case 27:    // ESC key - cancel input
                    if (echo) {
                        printf("\n[Input cancelled]\n");
                        fflush(stdout);
                    }
                    buffer[0] = '\0';
                    
                    // Disable input mode before returning
                    if (char_mutex != NULL) {
                        xSemaphoreTake(char_mutex, portMAX_DELAY);
                        uart_input_mode = false;
                        xSemaphoreGive(char_mutex);
                    }
                    // Cleanup: unsubscribe if we subscribed
                    if (!was_subscribed) {
                        esp_task_wdt_delete(NULL);
                    }
                    return ESP_ERR_INVALID_RESPONSE;
                    
                default:
                    // Only accept printable characters
                    if (ch >= 32 && ch <= 126) {
                        if (index < buffer_size - 1) {  // Leave room for null terminator
                            buffer[index] = ch;
                            index++;
                            if (echo) {
                                printf("%c", ch);
                                fflush(stdout);
                            }
                        } else {
                            // Buffer full - beep or indicate error
                            if (echo) {
                                printf("\a");  // Bell character
                                fflush(stdout);
                            }
                            ESP_LOGW(TAG, "Input buffer full");
                        }
                    }
                    break;
            }
        }
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

    if (uart_cmd_queue != NULL) {
        vQueueDelete(uart_cmd_queue);
        uart_cmd_queue = NULL;
    }

    if (uart_input_queue != NULL) {
        vQueueDelete(uart_input_queue);
        uart_input_queue = NULL;
    }

    if (uart_event_queue != NULL) {
        uart_driver_delete(UART_PORT);
        uart_event_queue = NULL;
    }
    
    ESP_LOGI(TAG, "UART module deinitialized");
}
