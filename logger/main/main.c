#include <stdio.h>
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_freertos_hooks.h"
#include <string.h>



#define UART_PORT UART_NUM_0  // or whichever UART port you're using
#define UART_BUF_SIZE 1024
#define RD_BUF_SIZE 128

// These were found in v3.3 of the logger Schematic (basically if this is wrong yell at Kaden)
#define CAN_CTX 11 //GPIO 18
#define CAN_RTX 12 //GPIO 8

// Define your channels using a macro, including some entries with integers at the end
#define LOG_CHANNELS \
    X(TS) \
    X(TS1) \
    X(TS2) \
    X(TS3) \
    X(F_BRAKEPRESSURE) \
    X(F_BRAKEPRESSURE1) \
    X(R_BRAKEPRESSURE) \
    X(R_BRAKEPRESSURE1) \
    X(STEERING) \
    X(STEERING1) \
    X(FLSHOCK) \
    X(FLSHOCK1) \
    X(FRSHOCK) \
    X(FRSHOCK1) \
    X(RRSHOCK) \
    X(RRSHOCK1) \
    X(RLSHOCK) \
    X(RLSHOCK1) \
    X(CURRENT) \
    X(CURRENT1) \
    X(BATTERY) \
    X(BATTERY1) \
    X(IMU_X_ACCEL) \
    X(IMU_X_ACCEL1) \
    X(IMU_X_ACCEL2) \
    X(IMU_X_ACCEL3) \
    X(IMU_Y_ACCEL) \
    X(IMU_Y_ACCEL1) \
    X(IMU_Y_ACCEL2) \
    X(IMU_Y_ACCEL3) \
    X(IMU_Z_ACCEL) \
    X(IMU_Z_ACCEL1) \
    X(IMU_Z_ACCEL2) \
    X(IMU_Z_ACCEL3) \
    X(IMU_X_GYRO) \
    X(IMU_X_GYRO1) \
    X(IMU_X_GYRO2) \
    X(IMU_X_GYRO3) \
    X(IMU_Y_GYRO) \
    X(IMU_Y_GYRO1) \
    X(IMU_Y_GYRO2) \
    X(IMU_Y_GYRO3) \
    X(IMU_Z_GYRO) \
    X(IMU_Z_GYRO1) \
    X(IMU_Z_GYRO2) \
    X(IMU_Z_GYRO3) \
    X(FR_SG) \
    X(FR_SG1) \
    X(FL_SG) \
    X(FL_SG1) \
    X(RL_SG) \
    X(RL_SG1) \
    X(RR_SG) \
    X(RR_SG1) \
    X(FLW_AMB) \
    X(FLW_AMB1) \
    X(FLW_OBJ) \
    X(FLW_OBJ1) \
    X(FLW_RPM) \
    X(FLW_RPM1) \
    X(FRW_AMB) \
    X(FRW_AMB1) \
    X(FRW_OBJ) \
    X(FRW_OBJ1) \
    X(FRW_RPM) \
    X(FRW_RPM1) \
    X(RRW_AMB) \
    X(RRW_AMB1) \
    X(RRW_OBJ) \
    X(RRW_OBJ1) \
    X(RRW_RPM) \
    X(RRW_RPM1) \
    X(RLW_AMB) \
    X(RLW_AMB1) \
    X(RLW_OBJ) \
    X(RLW_OBJ1) \
    X(RLW_RPM) \
    X(RLW_RPM1) \
    X(BRAKE_FLUID) \
    X(BRAKE_FLUID1) \
    X(THROTTLE_LOAD) \
    X(THROTTLE_LOAD1) \
    X(BRAKE_LOAD) \
    X(BRAKE_LOAD1) \
    X(DRS) \
    X(GPS_LON) \
    X(GPS_LON1) \
    X(GPS_LON2) \
    X(GPS_LON3) \
    X(GPS_LAT) \
    X(GPS_LAT1) \
    X(GPS_LAT2) \
    X(GPS_LAT3) \
    X(GPS_SPD) \
    X(GPS_SPD1) \
    X(GPS_SPD2) \
    X(GPS_SPD3) \
    X(GPS_FIX) \
    X(ECT) \
    X(OIL_PSR) \
    X(OIL_PSR1) \
    X(TPS) \
    X(APS) \
    X(DRIVEN_WSPD) \
    X(DRIVEN_WSPD1) \
    X(TESTNO) \
    X(DTC_FLW) \
    X(DTC_FRW) \
    X(DTC_RLW) \
    X(DTC_RRW) \
    X(DTC_FLSG) \
    X(DTC_FRSG) \
    X(DTC_RLSG) \
    X(DTC_RRSG) \
    X(DTC_IMU) \
    X(GPS_0_) \
    X(GPS_1_) \
    X(CH_COUNT)

enum LogChannel {
    #define X(channel) channel,
    LOG_CHANNELS
    #undef X
};

uint8_t logBuffer[CH_COUNT];
uint8_t usbBuffer[64];

// Shared character + mutex
static char last_char = '\0';
static SemaphoreHandle_t char_mutex;
static QueueHandle_t uart_event_queue = NULL;
static const char *TAG = "UART_APP";





// Queue to store received messages
static QueueHandle_t rx_queue;

// Semaphore for synchronization
static SemaphoreHandle_t rx_sem;

static void twai_init(void){

    // Create queue for received messages
    rx_queue = xQueueCreate(10, sizeof(twai_message_t));
    
    // Create semaphore for RX notifications
    rx_sem = xSemaphoreCreateBinary();


    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_CTX, CAN_RTX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK){
        ESP_LOGI(TAG, "CAN Driver Installed!");
    }else{
        ESP_LOGI(TAG, "FAIL: CAN Driver Install");
        return;
    }

    if(twai_start() == ESP_OK){
        ESP_LOGI(TAG, "CAN Start Successful");
    }else{
        ESP_LOGI(TAG, "FAIL: CAN Start");
    }

}

static void process_can_message(twai_message_t *message) {
    switch (message->identifier) {
        case 0x4e3:
            for(int i = TS; i <= TS3; i++){
                logBuffer[i] = message->data[i];
            }

            break;
        default:
            ESP_LOGI(TAG, "Unknown message ID: 0x%lu", message->identifier);
            break;
    }
}

static void twai_receive_task(void *pvParameters) {
    twai_message_t rx_message;
    
    while (1) {
        // Wait for message reception (blocking call)
        if (twai_receive(&rx_message, portMAX_DELAY) == ESP_OK) {
            // Process the received message
            // ESP_LOGI(TAG, "Received CAN message:");
            // ESP_LOGI(TAG, "ID: 0x%lu, DLC: %d", rx_message.identifier, rx_message.data_length_code);
            
            // // Print data bytes
            // printf("Data: ");
            // for (int i = 0; i < rx_message.data_length_code; i++) {
            //     printf("0x%02X ", rx_message.data[i]);
            // }
            // printf("\n");
            
            // Add your message processing logic here
            process_can_message(&rx_message);
        }
    }
}



static void uart_init(void){
    // Configure UART parameters
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Install UART driver with event queue
    uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 20, &uart_event_queue, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    ESP_LOGI(TAG, "UART initialized successfully");
}


void print_cpu_usage(void) {
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
        uint32_t idle_time = 0;
        
        for (x = 0; x < uxArraySize; x++) {
            if (strstr(pxTaskStatusArray[x].pcTaskName, "IDLE") != NULL) {
                idle_time += pxTaskStatusArray[x].ulRunTimeCounter;
            }
            
            // Calculate percentage with safety check
            ulStatsAsPercentage = (pxTaskStatusArray[x].ulRunTimeCounter * 100UL) / ulTotalTime;
            
            printf("%s\t\t%llu\t\t%llu%%\n", 
                   pxTaskStatusArray[x].pcTaskName, 
                   pxTaskStatusArray[x].ulRunTimeCounter, 
                   ulStatsAsPercentage);
        }
        
        uint32_t idle_percentage = (idle_time * 100UL) / ulTotalTime;
        printf("\nTotal CPU Idle Time: %" PRIu32 " ticks (%" PRIu32 "%%)\n", idle_time, idle_percentage);
        printf("Total Runtime: %llu ticks\n", ulTotalTime);
        
        vPortFree(pxTaskStatusArray);
    } else {
        printf("Error: Failed to allocate memory for task status array\n");
    }
}

// Task that processes user input and executes commands
void command_processor_task(void *param) {
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
                    
                case 'h':
                case 'H':
                case '?':
                    printf("\n=== ESP32 UART Command Menu ===\n");
                    printf("1 - Show system status\n");
                    printf("2 - Toggle LED\n");
                    printf("3 - Start CAN logging\n");
                    printf("4 - Show memory info\n");
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

// Task that receives UART events via interrupt
void uart_event_task(void *pvParameters) {
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



void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32 UART Command Application");
    
    // Initialize UART
    uart_init();
    
    // Create mutex for character sharing between tasks
    char_mutex = xSemaphoreCreateMutex();
    if (char_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    
    // Create tasks with error checking
    BaseType_t result;
    
    result = xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create uart_event_task");
        return;
    }
    
    result = xTaskCreate(command_processor_task, "cmd_processor", 4096, NULL, 5, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create command_processor_task");
        return;
    }
    
    ESP_LOGI(TAG, "All tasks created successfully");
    
    // Show welcome message and help
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for tasks to start
    printf("\n" \
           "====================================\n" \
           "    ESP32 UART Command Interface    \n" \
           "====================================\n" \
           "Type 'H' for help menu\n" \
           "Ready for commands...\n\n");
    
    // Main loop - keep system alive
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "System heartbeat - Free heap: %ld bytes", esp_get_free_heap_size());
    }
}


// void app_main(void)
// {

//     // twai_init();
//     uart_init();


//     // // Create mutex with validation
//     // char_mutex = xSemaphoreCreateMutex();
//     // if (char_mutex == NULL) {
//     //     ESP_LOGE("APP", "Failed to create mutex");
//     //     return;
//     // }

//     // // Create tasks with error checking
//     // BaseType_t result;
    
//     // result = xTaskCreate(twai_receive_task, "twai_rx", 4096, NULL, 5, NULL);
//     // if (result != pdPASS) {
//     //     ESP_LOGE("APP", "Failed to create twai_receive_task");
//     //     return;
//     // }
    
//     // result = xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
//     // if (result != pdPASS) {
//     //     ESP_LOGE("APP", "Failed to create uart_event_task");
//     //     return;
//     // }
    
//     // result = xTaskCreate(serial_report, "serial_report", 2048, NULL, 10, NULL);
//     // if (result != pdPASS) {
//     //     ESP_LOGE("APP", "Failed to create serial_report");
//     //     return;
//     // }

//     while(1){

//         vTaskDelay(pdMS_TO_TICKS(1000));

//     }

// }