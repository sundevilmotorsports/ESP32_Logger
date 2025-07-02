/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "Arduino.h"
#include "driver/twai.h"


static const char *TAG = "example";

// These were found in v3.3 of the logger Schematic (basically if this is wrong yell at Kaden)
#define CAN_CTX 11 //GPIO 18
#define CAN_RTX 12 //GPIO 8

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;


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

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

void app_main(void)
{

    twai_init();

    /* Configure the peripheral according to the LED type */
    configure_led();


    xTaskCreate(twai_receive_task, "twai_rx", 4096, NULL, 5, NULL);

    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
