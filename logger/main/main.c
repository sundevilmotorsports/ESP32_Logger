#include <stdio.h>
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

static const char *TAG = "example";

// These were found in v3.3 of the logger Schematic (basically if this is wrong yell at Kaden)
#define CAN_CTX 11 //GPIO 18
#define CAN_RTX 12 //GPIO 8



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

static void serial_report(void){
    
    ESP_LOGI(TAG, "Data: %02x, %02x, %02x, %02x\r\n", logBuffer[TS], logBuffer[TS1], logBuffer[TS2], logBuffer[TS3]);
    return;
}


void app_main(void)
{

    twai_init();

    xTaskCreate(twai_receive_task, "twai_rx", 4096, NULL, 5, NULL);

    while(1){
        // Allow other tasks to run by yielding for 1000ms
        serial_report();
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}