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
#include "dtc.h"
#include "logger.h"
#include "ina260.h"
#include "adc.h"
#include "gnss.h"
//TODO: Add SD card file storage


#define UART_PORT UART_NUM_0  // or whichever UART port you're using
#define UART_BUF_SIZE 1024
#define RD_BUF_SIZE 128

// These were found in v3.3 of the logger Schematic (basically if this is wrong yell at Kaden)
#define CAN_CTX 11 //GPIO 18
#define CAN_RTX 12 //GPIO 8

// Define all log channels with preprocessor macros for enum and file header generation
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

//Stores the Date and Time of the Latest Compile (21 Bytes)
const char compileDateTime[] = __DATE__ " " __TIME__;


// Shared character + mutex
static char last_char = '\0';
static SemaphoreHandle_t char_mutex;
static QueueHandle_t uart_event_queue = NULL;
static const char *TAG = "UART_APP";


static TaskHandle_t dtc_info_task_handle = NULL;
static bool dtc_info_running = false;

typedef struct {
	uint16_t ambTemp;
	uint16_t objTemp;
	uint16_t rpm;
} wheel_data_s_t;

//Logging variables
uint8_t				  TXDAT[8];
uint32_t count = 0;
uint32_t imuCount = 0;
uint32_t xAccel = 0, yAccel = 0, zAccel = 0;
uint32_t xGyro = 0, yGyro = 0, zGyro = 0;
uint16_t frsg = 0, flsg = 0, rrsg = 0, rlsg = 0;
wheel_data_s_t frw, flw, rlw, rrw;
uint8_t testNo = 0;
uint8_t canFifoFull = 0;
uint8_t drs = 0;
uint16_t brakeFluid = 0, throttleLoad = 0, brakeLoad = 0;
uint16_t oilPress = 0, driven_wspd = 0;
uint8_t ect = 0, tps = 0, aps = 0, shift0 = 0, shift1 = 0, shift2 = 0;


// Queue to store received messages
static QueueHandle_t rx_queue;

// Semaphore for synchronization
static SemaphoreHandle_t rx_sem;

static void can_init(void){

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
    switch(message->identifier) {
        case 0x35F:
            drs = message->data[0];
            break;
            
        case 0x360:
            //IMU Data
            xAccel = message->data[0] << 24 | message->data[1] << 16 | message->data[2] << 8 | message->data[3];
            yAccel = message->data[4] << 24 | message->data[5] << 16 | message->data[6] << 8 | message->data[7];
            imuCount++;
            
            //IMU DTC Check
            break;
            
        case 0x361:
            //IMU Data
            zAccel = message->data[0] << 24 | message->data[1] << 16 | message->data[2] << 8 | message->data[3];
            xGyro = message->data[4] << 24 | message->data[5] << 16 | message->data[6] << 8 | message->data[7];
            imuCount++;

            //IMU DTC Check
            break;
            
        case 0x362:
            //IMU Data
            yGyro = message->data[0] << 24 | message->data[1] << 16 | message->data[2] << 8 | message->data[3];
            zGyro = message->data[4] << 24 | message->data[5] << 16 | message->data[6] << 8 | message->data[7];
            imuCount++;

            //IMU DTC Response Update
            DTC_CAN_Response_Measurement(dtc_devices[imu_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x363:
            //Front Left Wheel Board
            flw.rpm = message->data[0] << 8 | message->data[1];
            flw.objTemp = message->data[2] << 8 | message->data[3];
            flw.ambTemp = message->data[4] << 8 | message->data[5];

            //DTC Response Update
            DTC_CAN_Response_Measurement(dtc_devices[flWheelBoard_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x364:
            //Front Right Wheel Board
            frw.rpm = message->data[0] << 8 | message->data[1];
            frw.objTemp = message->data[2] << 8 | message->data[3];
            frw.ambTemp = message->data[4] << 8 | message->data[5];

            //DTC Response Update
            DTC_CAN_Response_Measurement(dtc_devices[frWheelBoard_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x365:
            //Rear Right Wheel Board
            rrw.rpm = message->data[0] << 8 | message->data[1];
            rrw.objTemp = message->data[2] << 8 | message->data[3];
            rrw.ambTemp = message->data[4] << 8 | message->data[5];

            //DTC Response Update
            DTC_CAN_Response_Measurement(dtc_devices[rrWheelBoard_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x366:
            //Rear Left Wheel Board
            rlw.rpm = message->data[0] << 8 | message->data[1];
            rlw.objTemp = message->data[2] << 8 | message->data[3];
            rlw.ambTemp = message->data[4] << 8 | message->data[5];

            //DTC Response Update
            DTC_CAN_Response_Measurement(dtc_devices[rlWheelBoard_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x4e2:
            //Front Left String Gauge
            flsg = message->data[0] << 8 | message->data[1];

            //String Gauge DTC Check
            DTC_CAN_Response_Measurement(dtc_devices[flStrainGauge_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));

            break;
            
        case 0x4e3:
            //Front Right String Gauge
            frsg = message->data[0] << 8 | message->data[1];

            //String Gauge DTC Check
            DTC_CAN_Response_Measurement(dtc_devices[frStrainGauge_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x4e4:
            //Rear Right String Gauge
            rrsg = message->data[0] << 8 | message->data[1];

            //String Gauge DTC Check
            DTC_CAN_Response_Measurement(dtc_devices[rrStrainGauge_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x4e5:
            //Rear Left String Gauge
            rlsg = message->data[0] << 8 | message->data[1];

            //String Gauge DTC Check
            DTC_CAN_Response_Measurement(dtc_devices[rlStrainGauge_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x3e8:
            //Engine CAN Stream 2
            switch(message->data[0]){
                //Frame 1
                case 0x0:
                    // engine_speed = message->data[1] << 8 | message->data[2];
                    ect = message->data[3];
                    // oilTemp = message->data[4];
                    oilPress = message->data[5] << 8 | message->data[6];
                    //TODO: Could also add Park/Neutral Status (Stored on message->data[7])
                    break;

                case 0x1:
                    tps = message->data[2];
                    driven_wspd = message->data[4] << 8 | message->data[5];
                    break;
                    
                case 0x2:
                    aps = message->data[1];
                    break;
            }
            break;

        case 0x40:
            // Shifter Data
            shift0 = message->data[0];
            shift1 = message->data[1];
            shift2 = message->data[2];
            if((shift1 != 1) | (shift2 != 1)) {
                TXDAT[1] = shift1;
                TXDAT[2] = shift2;
            }
            DTC_CAN_Response_Measurement(dtc_devices[shifter_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
    }
}

static void can_receive_task(void *pvParameters) {
    twai_message_t rx_msg;
    
    while (1) {
        // Use timeout instead of blocking indefinitely
        esp_err_t result = twai_receive(&rx_msg, pdMS_TO_TICKS(100)); // 100ms timeout
        
        if (result == ESP_OK) {
            // Process the received message
            process_can_message(&rx_msg);
            
            // Optional: Add to queue for other tasks
            if (rx_queue != NULL) {
                xQueueSend(rx_queue, &rx_msg, 0);
            }
            
            // Give semaphore to notify other tasks
            if (rx_sem != NULL) {
                xSemaphoreGive(rx_sem);
            }
        } else if (result == ESP_ERR_TIMEOUT) {
            // Timeout occurred - this is normal, just continue
            // This allows the watchdog to be reset
        } else {
            // Handle other errors
            ESP_LOGW(TAG, "TWAI receive error: %s", esp_err_to_name(result));
        }
        
        // Small delay to prevent task from consuming too much CPU
        vTaskDelay(pdMS_TO_TICKS(1));
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

void print_dtc_info(void *pvParameters) {

    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 100 ms
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    char output_buffer[1000];
    char temp[100];

    while(1){
        // Check if task should continue running
        if (!dtc_info_running) {
            break; // Exit the task loop
        }
        
        // Wait for the next cycle (10Hz = every 100ms)
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

// Task that processes user input and executes commands
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

// Task that runs DTC error checking at 50Hz
void dtc_task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz = 20ms period
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "DTC error check task started at 50Hz");
    
    while (1) {
        // Wait for the next cycle (50Hz = every 20ms)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Get current time in ticks for DTC functions
        uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());
        
        // Run the DTC error check
        DTC_Error_Check(current_time);
    }
}

void logBuffer_task(void *pvParamaters){
    
    while(1){
        //Start Analog Listening
        adc_enable();

        // //Log Analog Sensor Data
        loggerEmplaceU16(logBuffer, F_BRAKEPRESSURE, getAnalog(ADC_FBP));
        loggerEmplaceU16(logBuffer, R_BRAKEPRESSURE, getAnalog(ADC_RBP));
        loggerEmplaceU16(logBuffer, STEERING, getAnalog(ADC_STP));
        loggerEmplaceU16(logBuffer, FLSHOCK, getAnalog(ADC_FLS));
        loggerEmplaceU16(logBuffer, FRSHOCK, getAnalog(ADC_FRS));
        loggerEmplaceU16(logBuffer, RRSHOCK, getAnalog(ADC_RRS));
        loggerEmplaceU16(logBuffer, RLSHOCK, getAnalog(ADC_RLS));

        adc_disable();

        // //Report Battery Current and Voltage
        loggerEmplaceU16(logBuffer, CURRENT, getCurrent());
        loggerEmplaceU16(logBuffer, BATTERY, getVoltage());

        //Report IMU Data
        loggerEmplaceU32(logBuffer, IMU_X_ACCEL, xAccel);
        loggerEmplaceU32(logBuffer, IMU_Y_ACCEL, yAccel);
        loggerEmplaceU32(logBuffer, IMU_Z_ACCEL, zAccel);

        loggerEmplaceU32(logBuffer, IMU_X_GYRO, xGyro);
        loggerEmplaceU32(logBuffer, IMU_Y_GYRO, yGyro);
        loggerEmplaceU32(logBuffer, IMU_Z_GYRO, zGyro);

        //Report Wheel Board Sensor Data
        loggerEmplaceU16(logBuffer, FLW_AMB, flw.ambTemp);
        loggerEmplaceU16(logBuffer, FLW_OBJ, flw.objTemp);
        loggerEmplaceU16(logBuffer, FLW_RPM, flw.rpm);

        loggerEmplaceU16(logBuffer, FRW_AMB, frw.ambTemp);
        loggerEmplaceU16(logBuffer, FRW_OBJ, frw.objTemp);
        loggerEmplaceU16(logBuffer, FRW_RPM, frw.rpm);

        loggerEmplaceU16(logBuffer, RRW_AMB, rrw.ambTemp);
        loggerEmplaceU16(logBuffer, RRW_OBJ, rrw.objTemp);
        loggerEmplaceU16(logBuffer, RRW_RPM, rrw.rpm);

        loggerEmplaceU16(logBuffer, RLW_AMB, rlw.ambTemp);
        loggerEmplaceU16(logBuffer, RLW_OBJ, rlw.objTemp);
        loggerEmplaceU16(logBuffer, RLW_RPM, rlw.rpm);

        //Report String Gauge Data
        loggerEmplaceU16(logBuffer, FR_SG, frsg);
        loggerEmplaceU16(logBuffer, FL_SG, flsg);
        loggerEmplaceU16(logBuffer, RR_SG, rrsg);
        loggerEmplaceU16(logBuffer, RL_SG, rlsg);

        //Report Brakes and Throttle
        loggerEmplaceU16(logBuffer, BRAKE_FLUID, brakeFluid);
        loggerEmplaceU16(logBuffer, THROTTLE_LOAD, throttleLoad);
        loggerEmplaceU16(logBuffer, BRAKE_LOAD, brakeLoad);

        //Report ECU Data
        loggerEmplaceU16(logBuffer, DRIVEN_WSPD, driven_wspd);
        loggerEmplaceU16(logBuffer, OIL_PSR, oilPress);
        logBuffer[TPS] = tps;
        logBuffer[ECT] = ect;
        logBuffer[APS] = aps;

        //Report DTC Data
        logBuffer[DTC_FLW]  = dtc_devices[flWheelBoard_DTC]->errState;
        logBuffer[DTC_FRW]  = dtc_devices[frWheelBoard_DTC]->errState;
        logBuffer[DTC_RRW]  = dtc_devices[rrWheelBoard_DTC]->errState;
        logBuffer[DTC_RLW]  = dtc_devices[rlWheelBoard_DTC]->errState;
        logBuffer[DTC_FLSG] = dtc_devices[flStrainGauge_DTC]->errState;
        logBuffer[DTC_FRSG] = dtc_devices[frStrainGauge_DTC]->errState;
        logBuffer[DTC_RLSG] = dtc_devices[flStrainGauge_DTC]->errState;
        logBuffer[DTC_RRSG] = dtc_devices[rrStrainGauge_DTC]->errState;
        logBuffer[DTC_IMU]  = dtc_devices[imu_DTC]->errState;
        logBuffer[GPS_0_]   = dtc_devices[gps_0_DTC]->errState;
        logBuffer[GPS_1_]   = dtc_devices[gps_1_DTC]->errState;

    }
}

void app_main(void)
{
    esp_log_level_set("GNSS_DMA", ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "Starting ESP32 UART Command Application");
    
    // Initialize UART
    uart_init();
    // can_init();
    // DTC_Init(pdTICKS_TO_MS(xTaskGetTickCount()));
    // i2c_master_init();
    // adcInit();
    
    // Create mutex for character sharing between tasks
    char_mutex = xSemaphoreCreateMutex();
    if (char_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    
    // Create tasks with error checking
    // BaseType_t result;
    //
    // result = xTaskCreate(can_receive_task, "can_rx", 4096, NULL, 5, NULL);
    // if (result != pdPASS) {
    //     ESP_LOGE("APP", "Failed to create can_receive_task");
    //     return;
    // }
    //
    // result = xTaskCreate(uart_input_task, "uart_input", 4096, NULL, 10, NULL);
    // if (result != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create uart_input_task");
    //     return;
    // }
    //
    // result = xTaskCreate(uart_output_task, "uart_output", 4096, NULL, 5, NULL);
    // if (result != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create uart_output_task");
    //     return;
    // }
    //
    //     // Add DTC error checking task
    // result = xTaskCreate(dtc_task, "dtc_check", 2048, NULL, 6, NULL);
    // if (result != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create dtc_task");
    //     return;
    // }

    gnss_init();

    gnss_start_task();

    ESP_LOGI(TAG, "All tasks created successfully");
    
    // Show welcome message and help
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for tasks to start
    // Main loop - keep system alive
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "System heartbeat - Free heap: %ld bytes", esp_get_free_heap_size());
    }
}

