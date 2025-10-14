#include "dtc.h"
#include <stdlib.h>
#include "esp_log.h" 
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


const char* dtc_device_names[] = {
    #define X(device) #device,
    DTC_DEVICES
    #undef X
};

static const char *TAG = "DTC_ERROR";
can_dtc *dtc_devices[DTC_COUNT];

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

// Optional helper function to create and start the DTC task
esp_err_t dtc_start_task(void) {
    BaseType_t result = xTaskCreate(
        dtc_task,           // Function
        "dtc_check",        // Name
        2048,              // Stack size
        NULL,              // Parameters
        6,                 // Priority
        NULL               // Task handle
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create DTC task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "DTC task created successfully");
    return ESP_OK;
}

/**
 * @brief Initialize a CAN DTC (Diagnostic Trouble Code) structure
 * 
 * Sets up the DTC structure with initial values and allocates memory for the 
 * time buffer used to calculate average response times.
 * 
 * @param dtc       Pointer to the can_dtc structure to initialize
 * @param index     Index of the DTC code in the array
 * @param measures  Goal number of measurements to calculate average response time
 * @param threshold Additional time above average before triggering error (in ms)
 * @param start_time Initial time to set for totalTime and prevTime
 * 
 * @note Memory allocation failure will set errState to 1 and log an error
 */
void DTC_CAN_Init_Device(can_dtc *dtc, uint8_t index, uint8_t measures, uint16_t threshold, uint64_t start_time){
    dtc->errState = 0; // Clear error state
    dtc->DTC_Idx = index; // Set DTC index
    dtc->measures = measures; // Set goal number of measurements
    dtc->bufferIndex = 0; // Initialize buffer index
    dtc->totalTime = start_time; // Reset total time
    dtc->prevTime = start_time; // Set previous time to start time
    dtc->threshold = threshold; // Set threshold for error state
    dtc->timeBuffer = (uint64_t *)malloc(measures * sizeof(uint64_t)); // Allocate memory for time buffer

    if (dtc->timeBuffer == NULL) {
        // Handle memory allocation failure
        dtc->errState = 1; // Set error state
        ESP_LOGE(TAG, "Failed to allocate memory for DTC time buffer -> Index: %d", index);
    }
    for (int i = 0; i < measures; i++) {
        dtc->timeBuffer[i] = start_time;
    }
	return;
}

/**
 * @brief Add a new response time measurement to the CAN DTC structure
 * This function updates the time buffer with the latest response time,
 * 
 * @param dtc Pointer to the can_dtc structure to update
 * @param response_time Current response time in milliseconds
 */
void DTC_CAN_Response_Measurement(can_dtc *dtc, uint64_t response_time) {
    if (dtc == NULL || dtc->timeBuffer == NULL) {
        ESP_LOGE(TAG, "Invalid DTC structure or time buffer -> Index: %d", dtc ? dtc->DTC_Idx : -1);
        return;
    }

    // Update the time buffer with the new response time
    dtc->timeBuffer[dtc->bufferIndex] = response_time - dtc->prevTime; // Calculate time since last measurement
    dtc->totalTime += dtc->timeBuffer[dtc->bufferIndex]; // Update total time
    dtc->bufferIndex = (dtc->bufferIndex + 1) % dtc->measures; // Increment buffer index and wrap around if necessary



    dtc->prevTime = response_time; // Update previous time to current response time

    return;
}

/**
 * @brief Update the error state of a CAN DTC based on response time
 * 
 * Checks if the current response time exceeds the average response time by a certain threshold.
 * If it does, sets the error state to indicate an error condition.
 * 
 * @param dtc Pointer to the can_dtc structure to update
 * @param current_time Current time in milliseconds
 */
void DTC_CAN_Update_Error_State(can_dtc *dtc, uint64_t current_time) {
    if (dtc == NULL || dtc->timeBuffer == NULL) {
        ESP_LOGE(TAG, "Invalid DTC structure or time buffer -> Index: %d", dtc ? dtc->DTC_Idx : -1);
        return;
    }

    current_time = current_time - dtc->prevTime; // Calculate time since last measurement

    // Find max in current buffer
    uint64_t max_response = 0;
    for (uint8_t i = 0; i < dtc->measures; i++) {
        if (dtc->timeBuffer[i] > max_response) {
            max_response = dtc->timeBuffer[i];
        }
    }
    
    // Error if current response is much larger than recent maximum
    if (current_time > (max_response + dtc->threshold)) {
        dtc->errState = 0;
    }
    else {
        dtc->errState = 1; // Clear error state if within threshold
    }

    return;
}

void DTC_Init(uint64_t start_time){
    for(int i = 0; i < DTC_COUNT; i++) {
        dtc_devices[i] = (can_dtc *)malloc(sizeof(can_dtc));
        if (dtc_devices[i] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for DTC device %d", i);
            continue;
        }
        DTC_CAN_Init_Device(dtc_devices[i], i, DTC_MEASURES, DTC_THRESHOLD_MS, start_time);
    }

}

void DTC_Error_Check(uint64_t current_time) {
    for (int i = 0; i < DTC_COUNT; i++) {
        if (dtc_devices[i] != NULL) {
            DTC_CAN_Update_Error_State(dtc_devices[i], current_time);
        } else {
            ESP_LOGE(TAG, "DTC device %d is NULL", i);
        }
    }
}