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
#include "tasks.h"

#define REFRESH_MS 10
#define RING_CAP 500

log_ring_t *log_ring;
TaskHandle_t log_flush_task_handle = NULL;
static uint8_t log_flush_staging[RING_CAP * CH_COUNT];
uint8_t logBuffer[CH_COUNT];
uint8_t usbBuffer[64];

// Stores the Date and Time of the Latest Compile (21 Bytes)
const char compileDateTime[] = __DATE__ " " __TIME__;

static const char *TAG = "MAIN_APP";

typedef struct{
    uint16_t ambTemp;
    uint16_t objTemp;
    uint16_t rpm;
} wheel_data_s_t;

// Logging variables
// CAN Tx buffer - unimplemented
uint8_t TXDAT[8];
uint32_t count = 0;

// # of times IMU send us a message - debug
uint32_t imuCount = 0;

// Strain Gauge Storage - raw data
int16_t frsg = 0, flsg = 0, rrsg = 0, rlsg = 0;

wheel_data_s_t frw, flw, rlw, rrw;
tiretemp_data frt, flt, rlt, rrt;
uint8_t testNo = 0;
uint8_t canFifoFull = 0;
uint8_t drs = 0;
uint16_t brakeFluid = 0, throttleLoad = 0, brakeLoad = 0;
uint16_t oilPress = 0, driven_wspd = 0;
uint8_t ect = 0, tps = 0, aps = 0, shift0 = 0, shift1 = 0, shift2 = 0;

imu_accel_t imu_accel;
imu_gyro_t imu_gyro;

int16_t slipAngles[6] = {0};

LR_A_t LR_A;
LR_B_t LR_B;
LR_C_t LR_C;


static void process_can_message(twai_frame_t *message)
{
    uint8_t data[64] = {0};
    size_t copy_len = message->header.dlc;
    if (copy_len > sizeof(data))
    {
        copy_len = sizeof(data);
    }

    memcpy(data, message->buffer, copy_len);

    switch (message->header.id)
    {
    //Slip Angles
    case 0x01:{
        int16_t raw = (int16_t)((data[1] << 8) | data[0]);
        slipAngles[0] = raw;
        break;
    }
    case 0x02:{
        int16_t raw = (int16_t)((data[1] << 8) | data[0]);
        slipAngles[1] = raw;
        break;
    }
    case 0x03:{
        int16_t raw = (int16_t)((data[1] << 8) | data[0]);
        slipAngles[2] = raw;
        break;
    }
    case 0x04:{
        int16_t raw = (int16_t)((data[1] << 8) | data[0]);
        slipAngles[3] = raw;
        break;
    }
    case 0x05:{
        int16_t raw = (int16_t)((data[1] << 8) | data[0]);
        slipAngles[4] = raw;
        break;
    }
    case 0x06:{
        int16_t raw = (int16_t)((data[1] << 8) | data[0]);
        slipAngles[5] = raw;
        break;
    }

    case 0x40:
        // Shifter Data
        shift0 = data[0];
        shift1 = data[1];
        shift2 = data[2];
        if ((shift1 != 1) | (shift2 != 1))
        {
            TXDAT[1] = shift1;
            TXDAT[2] = shift2;
        }
        DTC_CAN_Response_Measurement(dtc_devices[shifter_DTC], pdMS_TO_TICKS(xTaskGetTickCount()));
        break;

    // WFT Data
    case 0x51:
        memcpy(&LR_A, data, 8);
        break;
    case 0x52:
        memcpy(&LR_B, data, 8);
        break;
    case 0x53:
        memcpy(&LR_C, data, 6);
        break;
        
    case 0x35F:
        drs = data[0];
        break;
    // IMU Data Handling
    case 0x360:
        memcpy(&imu_gyro, data, 6);
        break;
    case 0x361:
        memcpy(&imu_accel, data, 6);
        DTC_CAN_Response_Measurement(dtc_devices[imu_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;

    case 0x370:
        // Front Left Wheel Board
        flw.rpm = data[0] << 8 | data[1];
        flw.objTemp = data[2] << 8 | data[3];
        flw.ambTemp = data[4] << 8 | data[5];

        // DTC Response Update
        DTC_CAN_Response_Measurement(dtc_devices[flWheelBoard_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;
    case 0x371:
        memcpy(data, &flt.tiretemp1, sizeof(flt.tiretemp1));
        break;
    case 0x372:
        memcpy(data, &flt.tiretemp2, sizeof(flt.tiretemp2));
        break;

    case 0x380:
        // Front Right Wheel Board
        frw.rpm = data[0] << 8 | data[1];
        frw.objTemp = data[2] << 8 | data[3];
        frw.ambTemp = data[4] << 8 | data[5];

        // DTC Response Update
        DTC_CAN_Response_Measurement(dtc_devices[frWheelBoard_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;
    case 0x381:
        memcpy(data, &frt.tiretemp1, sizeof(frt.tiretemp1));
        break;
    case 0x382:
        memcpy(data, &frt.tiretemp2, sizeof(frt.tiretemp2));
        break;

    case 0x390:
        // Rear Right Wheel Board
        rrw.rpm = data[0] << 8 | data[1];
        rrw.objTemp = data[2] << 8 | data[3];
        rrw.ambTemp = data[4] << 8 | data[5];

        // DTC Response Update
        DTC_CAN_Response_Measurement(dtc_devices[rrWheelBoard_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;
    case 0x391:
        memcpy(data, &rrt.tiretemp1, sizeof(rrt.tiretemp1));
        break;
    case 0x392:
        memcpy(data, &rrt.tiretemp2, sizeof(rrt.tiretemp2));
        break;

    case 0x3a0:
        // Rear Left Wheel Board
        rlw.rpm = data[0] << 8 | data[1];
        rlw.objTemp = data[2] << 8 | data[3];
        rlw.ambTemp = data[4] << 8 | data[5];
        
        // DTC Response Update
        DTC_CAN_Response_Measurement(dtc_devices[rlWheelBoard_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;

    //Rear Left Tire Temps
    case 0x3a1:
        memcpy(data, &rlt.tiretemp1, sizeof(rlt.tiretemp1));
        break;
    case 0x3a2:
    memcpy(data, &rlt.tiretemp2, sizeof(rlt.tiretemp2));
    break;

    case 0x3e8:
        // Engine CAN Stream 2
        switch (data[0])
        {
        // Frame 1
        case 0x0:
            // engine_speed = message->data[1] << 8 | message->data[2];
            ect = data[3];
            // oilTemp = message->data[4];
            oilPress = data[5] << 8 | data[6];
            // TODO: Could also add Park/Neutral Status (Stored on message->data[7])
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
    
    case 0x4e2:
        // Front Left String Gauge
        flsg = data[0] << 8 | data[1];

        // String Gauge DTC Check
        DTC_CAN_Response_Measurement(dtc_devices[flStrainGauge_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;

    case 0x4e3:
        // Front Right String Gauge
        frsg = data[0] << 8 | data[1];

        // String Gauge DTC Check
        DTC_CAN_Response_Measurement(dtc_devices[frStrainGauge_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;

    case 0x4e4:
        // Rear Right String Gauge
        rrsg = data[0] << 8 | data[1];

        // String Gauge DTC Check
        DTC_CAN_Response_Measurement(dtc_devices[rrStrainGauge_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;

    case 0x4e5:
        // Rear Left String Gauge
        rlsg = data[0] << 8 | data[1];

        // Strain Gauge DTC Check
        DTC_CAN_Response_Measurement(dtc_devices[rlStrainGauge_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;

    // Telemetry File Change Command
    case 0x69e:
        char filename[64];
        memcpy(filename, data, copy_len);
        filename[copy_len] = '\0';

        nvs_set_log_name(filename);
        nvs_set_testno(0);
        sdcard_create_numbered_log_file(filename);
        printf("Starting new logfile: '%s'\n", sdcard_get_current_log_filepath());
        break;

    default:
        // ESP_LOGI(TAG, "CAN Rx\tID: %x\r\n", message->header.id);
        break;
    }
}

void log_flush_task(void *pvParamaters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (log_ring == NULL)
        {
            ESP_LOGW(TAG, "Log ring not initialized; flush skipped");
            continue;
        }

        size_t entries = 0;
        uint8_t *dst = log_flush_staging;
        while (entries < RING_CAP)
        {
            if (log_ring_read(log_ring, dst) != 0)
            {
                break;
            }
            entries++;
            dst += CH_COUNT;
        }

        const size_t target_chunk_bytes = 4096; // Match SD file buffer size for better throughput.
        size_t chunk_entries = target_chunk_bytes / (size_t)CH_COUNT;
        if (chunk_entries == 0)
        {
            chunk_entries = 1;
        }

        size_t entry_index = 0;
        while (entry_index < entries)
        {
            size_t remaining_entries = entries - entry_index;
            size_t write_entries = remaining_entries < chunk_entries ? remaining_entries : chunk_entries;
            size_t write_bytes = write_entries * (size_t)CH_COUNT;

            esp_err_t result = fast_log_buffer(log_flush_staging + (entry_index * (size_t)CH_COUNT), write_bytes);
            if (result != ESP_OK)
            {
                ESP_LOGW(TAG, "Flush write failed at entry %u/%u", (unsigned)(entry_index + 1), (unsigned)entries);
                break;
            }
            entry_index += write_entries;
        }
        sdcard_sync();
    }
}

void logBuffer_task(void *pvParamaters)
{
    adc_values_t adc_vals;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(REFRESH_MS);
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // //Log Analog Sensor Data
        // Get ADC values quickly (no SPI operations here)
        if (adc_read_sync(&adc_vals) == ESP_OK)
        {
            // Log Analog Sensor Data using cached values
            loggerEmplaceU16(logBuffer, F_BRAKEPRESSURE, adc_vals.adc0);
            loggerEmplaceU16(logBuffer, R_BRAKEPRESSURE, adc_vals.adc1);
            loggerEmplaceU16(logBuffer, STEERING, adc_vals.adc2);
            loggerEmplaceU16(logBuffer, FLSHOCK, adc_vals.adc3);
            loggerEmplaceU16(logBuffer, FRSHOCK, adc_vals.adc4);
            loggerEmplaceU16(logBuffer, RRSHOCK, adc_vals.adc6);
            loggerEmplaceU16(logBuffer, RLSHOCK, adc_vals.adc7);
        }
        else
        {
            ESP_LOGW(TAG, "Using previous ADC values due to mutex timeout");
        }

        // //Report Battery Current and Voltage
        loggerEmplaceU16(logBuffer, CURRENT, getCurrent());
        loggerEmplaceU16(logBuffer, BATTERY, getVoltage());

        // Report IMU Data
        loggerEmplaceU16(logBuffer, IMU_X_ACCEL, imu_accel.x);
        loggerEmplaceU16(logBuffer, IMU_Y_ACCEL, imu_accel.y);
        loggerEmplaceU16(logBuffer, IMU_Z_ACCEL, imu_accel.z);

        loggerEmplaceU16(logBuffer, IMU_X_GYRO, imu_gyro.x);
        loggerEmplaceU16(logBuffer, IMU_Y_GYRO, imu_gyro.y);
        loggerEmplaceU16(logBuffer, IMU_Z_GYRO, imu_gyro.z);

        // Report Wheel Board Sensor Data
        loggerEmplaceU16(logBuffer, FLW_AMB, flw.ambTemp);
        loggerEmplaceU16(logBuffer, FLW_OBJ, flw.objTemp);
        loggerEmplaceU16(logBuffer, FLW_RPM, flw.rpm);
        loggerEmplaceU64(logBuffer, FLT_TTA, flt.tiretemp1);
        loggerEmplaceU64(logBuffer, FLT_TTB, flt.tiretemp2);

        loggerEmplaceU16(logBuffer, FRW_AMB, frw.ambTemp);
        loggerEmplaceU16(logBuffer, FRW_OBJ, frw.objTemp);
        loggerEmplaceU16(logBuffer, FRW_RPM, frw.rpm);
        loggerEmplaceU64(logBuffer, FRT_TTA, frt.tiretemp1);
        loggerEmplaceU64(logBuffer, FRT_TTB, frt.tiretemp2);

        loggerEmplaceU16(logBuffer, RRW_AMB, rrw.ambTemp);
        loggerEmplaceU16(logBuffer, RRW_OBJ, rrw.objTemp);
        loggerEmplaceU16(logBuffer, RRW_RPM, rrw.rpm);
        loggerEmplaceU64(logBuffer, RRT_TTA, rrt.tiretemp1);
        loggerEmplaceU64(logBuffer, RRT_TTB, rrt.tiretemp2);

        loggerEmplaceU16(logBuffer, RLW_AMB, rlw.ambTemp);
        loggerEmplaceU16(logBuffer, RLW_OBJ, rlw.objTemp);
        loggerEmplaceU16(logBuffer, RLW_RPM, rlw.rpm);
        loggerEmplaceU64(logBuffer, RLT_TTA, rlt.tiretemp1);
        loggerEmplaceU64(logBuffer, RLT_TTB, rlt.tiretemp2);

        // Report String Gauge Data
        loggerEmplaceU16(logBuffer, FR_SG, frsg);
        loggerEmplaceU16(logBuffer, FL_SG, flsg);
        loggerEmplaceU16(logBuffer, RR_SG, rrsg);
        loggerEmplaceU16(logBuffer, RL_SG, rlsg);

        // Report Brakes and Throttle
        loggerEmplaceU16(logBuffer, BRAKE_FLUID, brakeFluid);
        loggerEmplaceU16(logBuffer, THROTTLE_LOAD, throttleLoad);
        loggerEmplaceU16(logBuffer, BRAKE_LOAD, brakeLoad);

        // Report ECU Data
        loggerEmplaceU16(logBuffer, DRIVEN_WSPD, driven_wspd);
        loggerEmplaceU16(logBuffer, OIL_PSR, oilPress);
        logBuffer[TPS] = tps;
        logBuffer[ECT] = ect;
        logBuffer[APS] = aps;

        // Report DTC Data
        logBuffer[DTC_FLW] = dtc_devices[flWheelBoard_DTC]->errState;
        logBuffer[DTC_FRW] = dtc_devices[frWheelBoard_DTC]->errState;
        logBuffer[DTC_RRW] = dtc_devices[rrWheelBoard_DTC]->errState;
        logBuffer[DTC_RLW] = dtc_devices[rlWheelBoard_DTC]->errState;
        logBuffer[DTC_FLSG] = dtc_devices[flStrainGauge_DTC]->errState;
        logBuffer[DTC_FRSG] = dtc_devices[frStrainGauge_DTC]->errState;
        logBuffer[DTC_RLSG] = dtc_devices[rlStrainGauge_DTC]->errState;
        logBuffer[DTC_RRSG] = dtc_devices[rrStrainGauge_DTC]->errState;
        logBuffer[DTC_IMU] = dtc_devices[imu_DTC]->errState;
        logBuffer[GPS_0_] = dtc_devices[gps_0_DTC]->errState;
        logBuffer[GPS_1_] = dtc_devices[gps_1_DTC]->errState;

        loggerEmplaceU64(logBuffer, TS, (uint64_t)esp_timer_get_time());

        loggerEmplaceU32(logBuffer, GPS_LAT, GNSS_Handle.lat);
        loggerEmplaceU32(logBuffer, GPS_LON, GNSS_Handle.lon);
        loggerEmplaceU32(logBuffer, GPS_SPD, GNSS_Handle.gSpeed);
        logBuffer[GPS_FIX] = GNSS_Handle.fixType;

        //Slip Angle Log Emplace
        loggerEmplaceU16(logBuffer, SLIP_ANG_1_, slipAngles[0]);
        loggerEmplaceU16(logBuffer, SLIP_ANG_2_, slipAngles[1]);
        loggerEmplaceU16(logBuffer, SLIP_ANG_3_, slipAngles[2]);
        loggerEmplaceU16(logBuffer, SLIP_ANG_4_, slipAngles[3]);
        loggerEmplaceU16(logBuffer, SLIP_ANG_5_, slipAngles[4]);
        loggerEmplaceU16(logBuffer, SLIP_ANG_6_, slipAngles[5]);


        loggerEmplaceU16(logBuffer, LR_X_Force, LR_A.LR_X_Force);
        loggerEmplaceU16(logBuffer, LR_Y_Force, LR_A.LR_Y_Force);
        loggerEmplaceU16(logBuffer, LR_Z_Force, LR_A.LR_Z_Force);
        loggerEmplaceU16(logBuffer, LR_MX_Moment, LR_A.LR_MX_Moment);

        loggerEmplaceU16(logBuffer,LR_MY_Force,LR_B.LR_MY_Force);
        loggerEmplaceU16(logBuffer,LR_MZ_Force,LR_B.LR_MZ_Force);
        loggerEmplaceU16(logBuffer,LR_Velocity,LR_B.LR_Velocity);
        loggerEmplaceU16(logBuffer,LR_Position,LR_B.LR_Position);

        loggerEmplaceU16(logBuffer,LR_X_Acceleration,LR_C.LR_X_Acceleration);
        loggerEmplaceU16(logBuffer,LR_Y_Acceleration,LR_C.LR_Y_Acceleration);
        loggerEmplaceU16(logBuffer,LR_Z_Acceleration,LR_C.LR_Z_Acceleration);






        // // Write Data to SD Card - mutex handling is internal
        // esp_err_t result = fast_log_buffer(logBuffer, CH_COUNT);
        // if (result != ESP_OK) {
        //     ESP_LOGW(TAG, "Failed to write log buffer to SD card");
        // }

        if (log_ring_write(log_ring, logBuffer) != 0)
        {
            if (log_flush_task_handle != NULL)
            {
                xTaskNotifyGive(log_flush_task_handle);
            }
        }
        // Call sdcard_sync() every 1 second to flush buffers to persistent storage
        // if (last_sync_tick == 0) {
        //     last_sync_tick = xTaskGetTickCount();
        // } else {
        //     TickType_t now = xTaskGetTickCount();
        //     if ((now - last_sync_tick) >= pdMS_TO_TICKS(1000)) {
        //         sdcard_sync();
        //         last_sync_tick = now;
        //     }
        // }
    }
}

void app_main(void)
{
    esp_log_level_set("GNSS_DMA", ESP_LOG_DEBUG);

    log_ring = log_ring_create((size_t)CH_COUNT, (size_t)RING_CAP);
    if (log_ring == NULL)
    {
        ESP_LOGE(TAG, "Failed to create log ring buffer");
        return;
    }

    // Initialize UART
    sdcard_init();
    gnss_init();
    ESP_ERROR_CHECK(uart_init());
    DTC_Init();
    i2c_master_init();
    adc_init();
    can_init(process_can_message);

    if (tasks_start_all() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start RTOS tasks");
        return;
    }

    // Show welcome message and help
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for tasks to start
    // Main loop - keep system alive
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        // ESP_LOGI(TAG, "System heartbeat - Free heap: %ld bytes", esp_get_free_heap_size());
    }
}
