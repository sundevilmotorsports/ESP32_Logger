#include <stdio.h>
#include "driver/gpio.h"
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
#include "can.h"
#include "esp_twai.h"
#include "sdcard.h"
#include "log_chnl.h"
#include "uart.h"



uint8_t logBuffer[CH_COUNT];
uint8_t usbBuffer[64];

//Stores the Date and Time of the Latest Compile (21 Bytes)
const char compileDateTime[] = __DATE__ " " __TIME__;


static const char *TAG = "MAIN_APP";


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




static void process_can_message(twai_frame_t *message) {

    uint8_t data[8];
    memcpy(data, message->buffer, message->header.dlc);
    switch(message->header.id) {
        case 0x35F:
            drs = data[0];
            break;
            
        case 0x360:
            //IMU Data
            xAccel = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
            yAccel = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
            imuCount++;
            
            //IMU DTC Check
            break;
            
        case 0x361:
            //IMU Data
            zAccel = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
            xGyro = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
            imuCount++;

            //IMU DTC Check
            break;
            
        case 0x362:
            //IMU Data
            yGyro = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
            zGyro = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
            imuCount++;

            //IMU DTC Response Update
            DTC_CAN_Response_Measurement(dtc_devices[imu_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x363:
            //Front Left Wheel Board
            flw.rpm = data[0] << 8 | data[1];
            flw.objTemp = data[2] << 8 | data[3];
            flw.ambTemp = data[4] << 8 | data[5];

            //DTC Response Update
            DTC_CAN_Response_Measurement(dtc_devices[flWheelBoard_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x364:
            //Front Right Wheel Board
            frw.rpm = data[0] << 8 | data[1];
            frw.objTemp = data[2] << 8 | data[3];
            frw.ambTemp = data[4] << 8 | data[5];

            //DTC Response Update
            DTC_CAN_Response_Measurement(dtc_devices[frWheelBoard_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x365:
            //Rear Right Wheel Board
            rrw.rpm = data[0] << 8 | data[1];
            rrw.objTemp = data[2] << 8 | data[3];
            rrw.ambTemp = data[4] << 8 | data[5];

            //DTC Response Update
            DTC_CAN_Response_Measurement(dtc_devices[rrWheelBoard_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x366:
            //Rear Left Wheel Board
            rlw.rpm = data[0] << 8 | data[1];
            rlw.objTemp = data[2] << 8 | data[3];
            rlw.ambTemp = data[4] << 8 | data[5];

            //DTC Response Update
            DTC_CAN_Response_Measurement(dtc_devices[rlWheelBoard_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x4e2:
            //Front Left String Gauge
            flsg = data[0] << 8 | data[1];

            //String Gauge DTC Check
            DTC_CAN_Response_Measurement(dtc_devices[flStrainGauge_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));

            break;
            
        case 0x4e3:
            //Front Right String Gauge
            frsg = data[0] << 8 | data[1];

            //String Gauge DTC Check
            DTC_CAN_Response_Measurement(dtc_devices[frStrainGauge_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x4e4:
            //Rear Right String Gauge
            rrsg = data[0] << 8 | data[1];

            //String Gauge DTC Check
            DTC_CAN_Response_Measurement(dtc_devices[rrStrainGauge_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x4e5:
            //Rear Left String Gauge
            rlsg = data[0] << 8 | data[1];

            //String Gauge DTC Check
            DTC_CAN_Response_Measurement(dtc_devices[rlStrainGauge_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
            
        case 0x3e8:
            //Engine CAN Stream 2
            switch(data[0]){
                //Frame 1
                case 0x0:
                    // engine_speed = message->data[1] << 8 | message->data[2];
                    ect = data[3];
                    // oilTemp = message->data[4];
                    oilPress = data[5] << 8 | data[6];
                    //TODO: Could also add Park/Neutral Status (Stored on message->data[7])
                    break;

                case 0x1:
                    tps = data[2];
                    driven_wspd = data[4] << 8 | data[5];
                    break;
                    
                case 0x2:
                    aps = data[1];
                    break;
            }
            break;

        case 0x40:
            // Shifter Data
            shift0 = data[0];
            shift1 = data[1];
            shift2 = data[2];
            if((shift1 != 1) | (shift2 != 1)) {
                TXDAT[1] = shift1;
                TXDAT[2] = shift2;
            }
            DTC_CAN_Response_Measurement(dtc_devices[shifter_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
            break;
    }
}



void logBuffer_task(void *pvParamaters){
    //Initialize SD Card / File System?
    
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

        // Write Data to SD Card - mutex handling is internal
        esp_err_t result = fast_log_buffer(logBuffer, CH_COUNT);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Failed to write log buffer to SD card");
        }

    }
}

void app_main(void)
{
    esp_log_level_set("GNSS_DMA", ESP_LOG_DEBUG);

    
    // Initialize UART
    gnss_init();
    ESP_ERROR_CHECK(uart_init());
    DTC_Init(pdTICKS_TO_MS(xTaskGetTickCount()));
    i2c_master_init();
    adcInit();
    can_init(process_can_message);
    

    ESP_ERROR_CHECK(uart_create_tasks());

    if (dtc_start_task() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create dtc_task");
        return;
    }


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

