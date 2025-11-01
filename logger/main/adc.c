#include "adc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"


#define ADC_DIN     GPIO_NUM_7
#define ADC_DOUT    GPIO_NUM_6
#define ADC_CLK     GPIO_NUM_5
#define ADC_CS      GPIO_NUM_4


typedef struct {
    uint16_t adc0;
    uint16_t adc1;
    uint16_t adc2;
    uint16_t adc3;
    uint16_t adc4;
    uint16_t adc5;
    uint16_t adc6;
    uint16_t adc7;
} adc_values_t;


static const char *TAG = "ADC";

uint16_t frontBrakePress = 0;
uint16_t rearBrakePress = 0;
uint16_t steerPos = 0;
uint16_t flShock = 0;
uint16_t frShock = 0;
uint16_t rlShock = 0;
uint16_t rrShock = 0;

static SemaphoreHandle_t adc_data_mutex = NULL;

static spi_device_handle_t spi_handle;
static adc_values_t adc_local;


static void read_all_channels( void );
static void adc_reading_task( void *pvParameters );


static void read_all_channels( void )
{
    uint8_t tx_data[ 2 ];
    uint8_t rx_data[ 2 ];

    uint8_t* ptr = ( uint8_t* ) &adc_local;

    spi_transaction_t t = {
        .length = 16,
        .rxlength = 16,                    
        .tx_buffer = &tx_data,
        .rx_buffer = &rx_data
    };

    // Send first message for channel 0
    tx_data[ 0 ] = 0;
    tx_data[ 1 ] = 0;

    spi_device_transmit( spi_handle, &t );

    for( uint8_t ch = 1; ch <= 7; ch++ )
    {
        tx_data[ 0 ] = ch << 3;
        spi_device_transmit( spi_handle, &t );

        // Store xr_data[ 1 ] first because MSB is received first and ESP32 is little endian
        *ptr = rx_data[ 1 ];
        ptr++;
        *ptr = rx_data[ 0 ];
        ptr++;
    }

    // Last message to read data of channel 7
    tx_data[ 0 ] = 0;
    spi_device_transmit( spi_handle, &t );

    *ptr = rx_data[ 1 ];
    ptr++;
    *ptr = rx_data[ 0 ];

    // For debugging
    // ESP_LOGI( TAG, " Channel %d: %f", 0, ( float ) adc_local.adc0 * 5 / 4096 );
    // ESP_LOGI( TAG, " Channel %d: %f", 1, ( float ) adc_local.adc1 * 5 / 4096 );
    // ESP_LOGI( TAG, " Channel %d: %f", 2, ( float ) adc_local.adc2 * 5 / 4096 );
    // ESP_LOGI( TAG, " Channel %d: %f", 3, ( float ) adc_local.adc3 * 5 / 4096 );
    // ESP_LOGI( TAG, " Channel %d: %f", 4, ( float ) adc_local.adc4 * 5 / 4096 );
    // ESP_LOGI( TAG, " Channel %d: %f", 5, ( float ) adc_local.adc5 * 5 / 4096 );
    // ESP_LOGI( TAG, " Channel %d: %f", 6, ( float ) adc_local.adc6 * 5 / 4096 );
    // ESP_LOGI( TAG, " Channel %d: %f", 7, ( float ) adc_local.adc7 * 5 / 4096 );
    // ESP_LOGI( TAG, " ---------------------------" );
}

static void adc_reading_task( void *pvParameters ) 
{
    const TickType_t xFrequency = pdMS_TO_TICKS( 10 ); // 100Hz sampling rate
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI( TAG, "ADC reading task started at 100Hz" );
    
    while (1) {
        // Wait for the next cycle
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        read_all_channels();
        
        // Update global variables atomically
        if ( xSemaphoreTake( adc_data_mutex, pdMS_TO_TICKS( 5 ) ) == pdTRUE ) 
        {
            frontBrakePress = adc_local.adc0;
            rearBrakePress = adc_local.adc1;
            steerPos = adc_local.adc2;
            frShock = adc_local.adc3;
            flShock = adc_local.adc4;
            rlShock = adc_local.adc6;
            rrShock = adc_local.adc7;
            xSemaphoreGive( adc_data_mutex );
        } 
        else 
        {
            ESP_LOGW( TAG, "Failed to acquire ADC mutex for writing" );
        }
    }
}


esp_err_t adc_init( void ) 
{
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

    if (ret == ESP_OK) 
    {
        ESP_LOGI( TAG, "SPI bus initialized successfully" );
    }
    else
    {
        ESP_LOGE( TAG, "Failed to initialize SPI bus" );
        return ret;
    }

    // Configure the ADC device on SPI3 bus
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,                      // No command phase
        .address_bits = 0,                      // No address phase
        .dummy_bits = 0,                        // No dummy bits
        .mode = 3,                              // SPI mode 3
        .clock_speed_hz = SPI_MASTER_FREQ_10M,   // 8 MHz clock
        .spics_io_num = ADC_CS,                 // CS pin for ADC
        .queue_size = 10,                        // Queue only one transaction at a time
        .flags = 0,                             // No special flags
        .pre_cb = NULL,                         // No pre-transaction callback
        .post_cb = NULL,                        // No post-transaction callback
    };

    // Add the ADC device to SPI3 bus
    ret = spi_bus_add_device( SPI3_HOST, &devcfg, &spi_handle );
    
    if ( ret == ESP_OK ) 
    {
        ESP_LOGI( TAG, "ADC device added to SPI bus" );
    }
    else
    {
        ESP_LOGE( TAG, "Failed to add ADC device to SPI bus" );
        spi_bus_free( SPI3_HOST );  // Clean up bus if device addition fails
        return ret;
    }

    adc_data_mutex = xSemaphoreCreateMutex();
    // Create ADC reading task (high priority for consistent sampling)
    BaseType_t result = xTaskCreate( adc_reading_task, "adc_reader", 4096, NULL, 8, NULL );

    if ( result != pdPASS ) {
        ESP_LOGE( TAG, "Failed to create ADC reading task" );
        return ESP_FAIL;
    }

    return ESP_OK;
}


void adc_deinit( void ) {
    if ( spi_handle ) 
    {
        spi_bus_remove_device( spi_handle );
        spi_handle = NULL;
    }
    spi_bus_free( SPI3_HOST );
}


esp_err_t adc_get_values( uint16_t *fbp, uint16_t *rbp, uint16_t *stp, 
                        uint16_t *fls, uint16_t *frs, uint16_t *rrs, uint16_t *rls ) 
{
    if ( xSemaphoreTake( adc_data_mutex, pdMS_TO_TICKS( 1 ) ) == pdTRUE ) 
    {
        *fbp = frontBrakePress;
        *rbp = rearBrakePress;
        *stp = steerPos;
        *fls = flShock;
        *frs = frShock;
        *rrs = rrShock;
        *rls = rlShock;
        
        xSemaphoreGive( adc_data_mutex );
        return ESP_OK;
    } 
    else 
    {
        ESP_LOGW( TAG, "Failed to acquire ADC mutex for reading" );
        return ESP_ERR_TIMEOUT;
    }
}