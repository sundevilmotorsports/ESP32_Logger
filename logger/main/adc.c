#include "adc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "ADC";
uint16_t frontBrakePress = 0, rearBrakePress = 0, steerPos = 0, flShock = 0, frShock = 0, rlShock = 0, rrShock = 0;
static SemaphoreHandle_t adc_data_mutex = NULL;

static uint8_t buffer[2];
static spi_device_handle_t spi_handle;

void adc_reading_task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz sampling rate
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "ADC reading task started at 100Hz");
    
    // Local variables for reading
    uint16_t local_fbp, local_rbp, local_stp;
    uint16_t local_fls, local_frs, local_rrs, local_rls;
    
    while (1) {
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Read all ADC channels (this takes time due to SPI operations)
        local_fbp = adc_get_channel(ADC_FBP);
        local_rbp = adc_get_channel(ADC_RBP);
        local_stp = adc_get_channel(ADC_STP);
        local_fls = adc_get_channel(ADC_FLS);
        local_frs = adc_get_channel(ADC_FRS);
        local_rrs = adc_get_channel(ADC_RRS);
        local_rls = adc_get_channel(ADC_RLS);
        
        // Update global variables atomically
        if (xSemaphoreTake(adc_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            frontBrakePress = local_fbp;
            rearBrakePress = local_rbp;
            steerPos = local_stp;
            flShock = local_fls;
            frShock = local_frs;
            rrShock = local_rrs;
            rlShock = local_rls;
            
            xSemaphoreGive(adc_data_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire ADC mutex for writing");
        }
    }
}

esp_err_t adc_init(void) {
    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = ADC_DOUT,     // MISO pin for SPI3
        .mosi_io_num = ADC_DIN,     // MOSI pin for SPI3
        .sclk_io_num = ADC_CLK,     // Clock pin for SPI3
        .quadwp_io_num = -1,            // Not used
        .quadhd_io_num = -1,            // Not used
        .max_transfer_sz = 4092,        // Maximum transfer size
    };

    // Initialize the SPI3 bus (VSPI)
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        return ret;
    }

    // Configure the ADC device on SPI3 bus
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,      // 1 MHz clock
        .mode = 0,                      // SPI mode 0
        .spics_io_num = ADC_CS,     // CS pin for ADC
        .queue_size = 1,                // Queue only one transaction at a time
        .flags = 0,                     // No special flags
        .pre_cb = NULL,                 // No pre-transaction callback
        .post_cb = NULL,                // No post-transaction callback
    };

    // Add the ADC device to SPI3 bus
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        spi_bus_free(SPI3_HOST);  // Clean up bus if device addition fails
        return ret;
    }


    // Initialize buffers
    buffer[0] = 0;
    buffer[1] = 0;

    adc_data_mutex = xSemaphoreCreateMutex();
    // Create ADC reading task (high priority for consistent sampling)
    BaseType_t result = xTaskCreate(adc_reading_task, "adc_reader", 4096, NULL, 8, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ADC reading task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void adc_deinit(void) {
    if (spi_handle) {
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
    }
    spi_bus_free(SPI3_HOST);
}

uint16_t adc_get_channel(uint8_t channel) {
    spi_transaction_t t = {0};
    uint16_t tx_data = (channel & 0x07) << 11;  // channel select bits in bits [13:11]
    uint16_t rx_data = 0;

    t.length = 16;                     // 16 bits total
    t.tx_buffer = &tx_data;
    t.rx_buffer = &rx_data;

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
        return 0;
    }

    // The ADC returns the 12-bit result in bits [15:4]
    uint16_t result = (rx_data >> 4) & 0x0FFF;
    return result;
}


esp_err_t adc_get_values(uint16_t *fbp, uint16_t *rbp, uint16_t *stp, 
                        uint16_t *fls, uint16_t *frs, uint16_t *rrs, uint16_t *rls) {
    if (xSemaphoreTake(adc_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        *fbp = frontBrakePress;
        *rbp = rearBrakePress;
        *stp = steerPos;
        *fls = flShock;
        *frs = frShock;
        *rrs = rrShock;
        *rls = rlShock;
        
        xSemaphoreGive(adc_data_mutex);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Failed to acquire ADC mutex for reading");
        return ESP_ERR_TIMEOUT;
    }
}