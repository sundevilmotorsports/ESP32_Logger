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
#include "wifi.h"
#include "server.h"

#define REFRESH_MS 10
#define RING_CAP 256

#define ENGINE_STREAM_ID_2 1000U
#define ENGINE_STREAM_ID_3 64U
#define ENGINE_STREAM_ID_5 1001U
#define ENGINE_STREAM_ID_6 1002U
#define ENGINE_STREAM_ID_7 1003U
#define ENGINE_STREAM_ID_8 1004U

log_ring_t *log_ring;
TaskHandle_t log_flush_task_handle = NULL;
static uint8_t log_flush_staging[RING_CAP * CH_COUNT];
uint8_t logBuffer[CH_COUNT];
uint8_t usbBuffer[64];

// Stores the Date and Time of the Latest Compile (21 Bytes)
const char compileDateTime[] = __DATE__ " " __TIME__;

static const char *TAG = "MAIN_APP";



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

engine_t engine;

imu_accel_t imu_accel;
imu_gyro_t imu_gyro;


// Old WFT Handlers
// SLIP_t SLIP;
// WFT_CAN1_t WFT_1;
// WFT_CAN2_t WFT_2;
// WFT_CAN3_t WFT_3;

static void process_can_message(twai_frame_t *message)
{
    uint8_t data[64] = {0};
    size_t copy_len = message->header.dlc;
    if (copy_len > sizeof(data))
    {
        copy_len = sizeof(data);
    }

    memcpy(data, message->buffer, copy_len);

    switch (message->header.id){
    //Old WFT Handlers
    // //Slip Angles
    // case 0x1:
    //     SLIP.POS1 = data[0] | (data[1] << 8);
    //     break;
    // case 0x2:
    //     SLIP.POS2 = data[0] | (data[1] << 8);
    //     break;
    // case 0x3:
    //     SLIP.POS3 = data[0] | (data[1] << 8);
    //     break;
    // case 0x4:
    //     SLIP.POS4 = data[0] | (data[1] << 8);
    //     break;
    // case 0x5:
    //     SLIP.POS5 = data[0] | (data[1] << 8);
    //     break;
    // case 0x6:
    //     SLIP.POS6 = data[0] | (data[1] << 8);
    //     break;
    // case 0x21:
    //     WFT_1.Fx_Force = data[0] | (data[1] << 8);
    //     WFT_1.Fy_Force = data[2] | (data[3] << 8);
    //     WFT_1.Fz_Force = data[4] | (data[5] << 8);
    //     WFT_1.Mx_Moment = data[6] | (data[7] << 8);
    //     // printf("WFT Fx: %d, Fy: %d\n", Fx, Fy);
    //     break;

    // case 0x22:
    //     WFT_2.My_Moment = data[0] | (data[1] << 8);
    //     WFT_2.Mz_Moment = data[2] | (data[3] << 8);
    //     WFT_2.Wheelspeed = data[4] | (data[5] << 8);
    //     WFT_2.Position = data[6] | (data[7] << 8);
    //     //printf("WFT Fz: %d, Mx: %d\n", Fz, Mx);
    //     break;

    // case 0x23:
    //     WFT_3.X_Acceleration = data[0] | (data[1] << 8);
    //     WFT_3.Y_Acceleration = data[1] | (data[2] << 8);
    //     WFT_3.Z_Acceleration = data[3] | (data[4] << 8);
    //     // printf("WFT My: %d, Mz: %d\n", My, Mz);
    //     break;

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
        // printf("FL WB\n");
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
        // printf("FR WB\n");
        frw.rpm = data[0] << 8 | data[1];
        frw.objTemp = data[2] << 8 | data[3];
        frw.ambTemp = data[4] << 8 | data[5];
        
        // DTC Response Update
        DTC_CAN_Response_Measurement(dtc_devices[frWheelBoard_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;
    case 0x381:
        // printf("FR Tire Temp1\n");
        memcpy(data, &frt.tiretemp1, sizeof(frt.tiretemp1));
        break;
    case 0x382:
        // printf("FR Tire Temp2\n");
        memcpy(data, &frt.tiretemp2, sizeof(frt.tiretemp2));
    break;
    
    case 0x390:
        // Rear Right Wheel Board
        // printf("RR WB\n");
        rrw.rpm = data[0] << 8 | data[1];
        rrw.objTemp = data[2] << 8 | data[3];
        rrw.ambTemp = data[4] << 8 | data[5];

        // DTC Response Update
        DTC_CAN_Response_Measurement(dtc_devices[rrWheelBoard_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
        break;
    case 0x391:
        // printf("RR Tire Temp1\n");
        memcpy(data, &rrt.tiretemp1, sizeof(rrt.tiretemp1));
        break;
    case 0x392:
        // printf("RR Tire Temp2\n");
        memcpy(data, &rrt.tiretemp2, sizeof(rrt.tiretemp2));
        break;

    case 0x3a0:
        // Rear Left Wheel Board
        // printf("RL WB\n");
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

    case ENGINE_STREAM_ID_2:
        // Engine CAN Stream 2
        switch (data[0]){
        // Frame 1
        case 0x0:
            engine.engine_speed = data[1] << 8 | data[2];
            engine.ect = data[3];
            engine.oil_temperature = data[4];
            engine.oil_pressure = data[5] << 8 | data[6];
            break;

        case 0x1:
            engine.lambda1 = data[1];
            engine.tps = data[2];
            engine.gear = data[3];
            engine.gp_speed1 = data[4] << 8 | data[5];
            break;

        case 0x2:
            engine.aps_main = data[1];
            engine.fuel_pressure = data[2] << 8 | data[3];
            DTC_CAN_Response_Measurement(dtc_devices[engine_DTC], pdTICKS_TO_MS(xTaskGetTickCount()));
            break;
        }
        break;

    case ENGINE_STREAM_ID_6:
        switch (data[0]){
            case 0x0:
                engine.map = data[5] << 8 | data[6];
                break;
        }
        break;
    
    case ENGINE_STREAM_ID_7:
        switch (data[0]){
            case 0x0:
                engine.an_temp_3 = data[5];
                break;
        }
        break;
    
    case ENGINE_STREAM_ID_8:
        engine.imu_accel_x = data[0] << 8 | data[1];
        engine.imu_accel_y = data[2] << 8 | data[3];
        engine.imu_accel_z = data[4] << 8 | data[5];
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
        // ESP_LOGI(TAG, "CAN Rx\tID: 0x%x\r\n", message->header.id);
        break;
    }
    // printf("Recived CAN message 0x%lX\n", message->header.id);
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
        if (chunk_entries == 0){
            chunk_entries = 1;
        }

        size_t entry_index = 0;
        while (entry_index < entries) {
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
    while (1){
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // //Log Analog Sensor Data
        // Get ADC values quickly (no SPI operations here)
        if (adc_read_sync(&adc_vals) == ESP_OK)
        {
            // Log Analog Sensor Data using cached values
            loggerEmplaceU16(logBuffer, FRSHOCK, adc_vals.adc0);
            loggerEmplaceU16(logBuffer, RRSHOCK, adc_vals.adc1);
            loggerEmplaceU16(logBuffer, R_BRAKEPRESSURE, adc_vals.adc2); // Might be swapped
            loggerEmplaceU16(logBuffer, RLSHOCK, adc_vals.adc3);
            loggerEmplaceU16(logBuffer, F_BRAKEPRESSURE, adc_vals.adc4); // Might be swapped
            // loggerEmplaceU16(logBuffer, RLSHOCK, adc_vals.adc5); // Unused
            loggerEmplaceU16(logBuffer, FLSHOCK, adc_vals.adc6);
            loggerEmplaceU16(logBuffer, STEERING, adc_vals.adc7);
            // printf("Analog: FR Shock: %d, RR Shock: %d, Rear BSE:%d, RL Shock:%d, Front BSE:%d, Unused:%d, FL Shock:%d, Steering:%d\n", adc_vals.adc0,adc_vals.adc1,adc_vals.adc2,adc_vals.adc3,adc_vals.adc4,adc_vals.adc5,adc_vals.adc6,adc_vals.adc7);

        }else{
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

        // Report Strain Gauge Data
        loggerEmplaceU16(logBuffer, FR_SG, frsg);
        loggerEmplaceU16(logBuffer, FL_SG, flsg);
        loggerEmplaceU16(logBuffer, RR_SG, rrsg);
        loggerEmplaceU16(logBuffer, RL_SG, rlsg);

        // Report Brakes and Throttle
        loggerEmplaceU16(logBuffer, BRAKE_FLUID, brakeFluid);
        loggerEmplaceU16(logBuffer, THROTTLE_LOAD, throttleLoad);
        loggerEmplaceU16(logBuffer, BRAKE_LOAD, brakeLoad);

        // Report ECU Data
        loggerEmplaceU16(logBuffer, ENGINE_SPEED, engine.engine_speed);
        logBuffer[ECT] = engine.ect;
        logBuffer[OIL_TEMP] = engine.oil_temperature;
        loggerEmplaceU16(logBuffer, OIL_PRESS, engine.oil_pressure);
        logBuffer[NEUTRAL_STAT] = engine.neutral_stat;
        logBuffer[LAMBDA] = engine.lambda1;
        logBuffer[TPS] = engine.tps;
        logBuffer[GEAR] = engine.gear;
        loggerEmplaceU16(logBuffer, GP_SPEED, engine.gp_speed1);
        loggerEmplaceU16(logBuffer, APS_MAIN, engine.aps_main);
        loggerEmplaceU16(logBuffer, FUEL_PRESS, engine.fuel_pressure);
        loggerEmplaceU16(logBuffer, ACCEL_FUEL, engine.accel_fuel);
        loggerEmplaceU16(logBuffer, ACCUM_DIST, engine.accumulated_dist);
        loggerEmplaceU16(logBuffer, MAP, engine.map);
        logBuffer[AN_TEMP_3_] = engine.an_temp_3;
        loggerEmplaceU16(logBuffer, ENG_IMU_X, engine.imu_accel_x);
        loggerEmplaceU16(logBuffer, ENG_IMU_Y, engine.imu_accel_y);
        loggerEmplaceU16(logBuffer, ENG_IMU_Z, engine.imu_accel_z);

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

        // Old WFT Handlers
        //Slip Angle Log Emplace
        // loggerEmplaceU16(logBuffer, SLIP_ANG_1_, SLIP.POS1);
        // loggerEmplaceU16(logBuffer, SLIP_ANG_2_, SLIP.POS2);
        // loggerEmplaceU16(logBuffer, SLIP_ANG_3_, SLIP.POS3);
        // loggerEmplaceU16(logBuffer, SLIP_ANG_4_, SLIP.POS4);
        // loggerEmplaceU16(logBuffer, SLIP_ANG_5_, SLIP.POS5);
        // loggerEmplaceU16(logBuffer, SLIP_ANG_6_, SLIP.POS6);


        // loggerEmplaceU16(logBuffer, WFT_FX_Force, WFT_1.Fx_Force);
        // loggerEmplaceU16(logBuffer, WFT_FY_Force, WFT_1.Fx_Force);
        // loggerEmplaceU16(logBuffer, WFT_FZ_Force, WFT_1.Fx_Force);
        // loggerEmplaceU16(logBuffer, WFT_MX_Moment, WFT_1.Mx_Moment);

        // loggerEmplaceU16(logBuffer,WFT_MY_Force,WFT_2.My_Moment);
        // loggerEmplaceU16(logBuffer,WFT_MZ_Force,WFT_2.Mz_Moment);
        // loggerEmplaceU16(logBuffer,WFT_Wheelspeed,WFT_2.Wheelspeed);
        // loggerEmplaceU16(logBuffer,WFT_Position,WFT_2.Position);

        // loggerEmplaceU16(logBuffer,WFT_X_Acceleration,WFT_3.X_Acceleration);
        // loggerEmplaceU16(logBuffer,WFT_Y_Acceleration,WFT_3.Y_Acceleration);
        // loggerEmplaceU16(logBuffer,WFT_Z_Acceleration,WFT_3.Z_Acceleration);

        // // Write Data to SD Card - mutex handling is internal
        // esp_err_t result = fast_log_buffer(logBuffer, CH_COUNT);
        // if (result != ESP_OK) {
        //     ESP_LOGW(TAG, "Failed to write log buffer to SD card");
        // }

        if (log_ring_write(log_ring, logBuffer) != 0){
            if (log_flush_task_handle != NULL){
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

esp_err_t start_server() {
    // start wifi
    wifi_init();

    // start http server
    if(http_server_start() == ESP_OK) {
        ESP_LOGI(TAG, "Server started");
        ESP_LOGI(TAG, "Connect to Wi-Fi 'data-logger' and visit: http://192.168.4.1/api/view");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Failed to start server");
    return ESP_FAIL;
}


void app_main(void){
    bool server_started_once = false;
    esp_log_level_set("GNSS_DMA", ESP_LOG_DEBUG);

    log_ring = log_ring_create((size_t)CH_COUNT, (size_t)RING_CAP);
    if (log_ring == NULL){
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

    if (tasks_start_all() != ESP_OK){
        ESP_LOGE(TAG, "Failed to start RTOS tasks");
        return;
    }

    // Show welcome message and help
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for tasks to start
    // Main loop - keep system alive
    while (1){
        // printf("\033[2J\033[H");
        // printf("WFT CAN1: Fx: %d, Fy: %d, Fz:%d, Mz:%d\n", WFT_1.Fx_Force, WFT_1.Fy_Force, WFT_1.Fz_Force,WFT_1.Mx_Moment);
        // printf("WFT CAN2: My: %d, Mz: %d, Wheelspeed:%d, Position:%d\n", WFT_2.My_Moment, WFT_2.Mz_Moment, WFT_2.Wheelspeed,WFT_2.Position);
        // printf("WFT CAN3: X_Accel: %d, Y_Accel: %d, Z_Accel:%d\n", WFT_3.X_Acceleration, WFT_3.Y_Acceleration, WFT_3.Z_Acceleration);
        // printf("Slip: Ch1: %d, Ch2: %d, Ch3:%d, Ch4:%d, Ch5:%d, Ch6:%d\n", SLIP.POS1, SLIP.POS2, SLIP.POS3, SLIP.POS4, SLIP.POS5, SLIP.POS6);
        // printf("Tire Temp FRT: %lld%lld RRT: %lld%lld\n", frt.tiretemp1, frt.tiretemp2, rrt.tiretemp1, rrt.tiretemp2);
        vTaskDelay(pdMS_TO_TICKS(5000));
        // ESP_LOGI(TAG, "System heartbeat - Free heap: %ld bytes", esp_get_free_heap_size());

        if (can_msg_count == 0 && !http_server_is_running() && !server_started_once) {
            if (start_server() == ESP_OK) {
                server_started_once = true;
            }
        }

        auto_server_stop();
    }
}
