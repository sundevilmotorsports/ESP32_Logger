#include <stdio.h>
#include "driver/gpio.h"
#include "driver/twai.h"
#include "Arduino.h"

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
        case 0x123:
            // Handle specific message ID
            ESP_LOGI(TAG, "Processing sensor data message");
            break;
            
        case 0x456:
            // Handle another message type
            ESP_LOGI(TAG, "Processing control message");
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
            ESP_LOGI(TAG, "Received CAN message:");
            ESP_LOGI(TAG, "ID: 0x%lu, DLC: %d", rx_message.identifier, rx_message.data_length_code);
            
            // Print data bytes
            printf("Data: ");
            for (int i = 0; i < rx_message.data_length_code; i++) {
                printf("0x%02X ", rx_message.data[i]);
            }
            printf("\n");
            
            // Add your message processing logic here
            process_can_message(&rx_message);
        }
    }
}


void app_main(void)
{

    //Start CAN Driver Up
    twai_init();

    //Create Async Recieve Task (FreeRTOS is awesome and a bitch)
    xTaskCreate(twai_receive_task, "twai_rx", 4096, NULL, 5, NULL);

}