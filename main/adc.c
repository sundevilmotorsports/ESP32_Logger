#include "adc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"


#define ADC_DIN     GPIO_NUM_7
#define ADC_DOUT    GPIO_NUM_6
#define ADC_CLK     GPIO_NUM_5
#define ADC_CS      GPIO_NUM_4




static const char *TAG = "ADC";


static SemaphoreHandle_t adc_data_mutex = NULL;

static spi_device_handle_t spi_handle;
static adc_values_t adc_local;



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

// NEW: synchronous read API
esp_err_t adc_read_sync(adc_values_t *out)
{
    if (spi_handle == NULL) {
        ESP_LOGE(TAG, "SPI device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // perform a blocking read into adc_local
    read_all_channels();


    if (out) {
        memcpy(out, &adc_local, sizeof(adc_local));
    }

    return ESP_OK;
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

