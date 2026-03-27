#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EXT_SPI_ADC_MAX_CHANNELS 8

typedef uint8_t ext_adc_channel_t;

typedef struct ext_spi_adc_t *ext_spi_adc_handle_t;

typedef struct {
    spi_host_device_t host_id;
    spi_dma_chan_t dma_chan;
    gpio_num_t sclk_io_num;
    gpio_num_t mosi_io_num;
    gpio_num_t miso_io_num;
    int max_transfer_sz;
    bool initialize_bus;
} ext_spi_adc_bus_config_t;

typedef struct {
    gpio_num_t cs_io_num;
    int clock_speed_hz;
    uint8_t spi_mode;
    int queue_size;
} ext_spi_adc_device_config_t;

typedef struct {
    uint8_t channel_count;
    // For the current ADC, the logical channel select bits live in the first
    // transmitted byte and the second byte is zero by default.
    uint8_t channel_command_shift;
    uint16_t sample_mask;
} ext_spi_adc_protocol_config_t;

typedef struct {
    ext_spi_adc_bus_config_t bus;
    ext_spi_adc_device_config_t device;
    ext_spi_adc_protocol_config_t protocol;
} ext_spi_adc_config_t;

typedef struct {
    // Raw 16-bit SPI frame in transmit order: tx_data[0] is shifted out first.
    uint8_t tx_data[2];
} ext_spi_adc_channel_config_t;

typedef struct {
    uint8_t channel_count;
    uint16_t values[EXT_SPI_ADC_MAX_CHANNELS];
} ext_spi_adc_frame_t;

esp_err_t ext_spi_adc_new(const ext_spi_adc_config_t *config, ext_spi_adc_handle_t *ret_handle);
esp_err_t ext_spi_adc_config_channel(ext_spi_adc_handle_t handle,
                                     ext_adc_channel_t channel,
                                     const ext_spi_adc_channel_config_t *config);
esp_err_t ext_spi_adc_read_channel(ext_spi_adc_handle_t handle,
                                   ext_adc_channel_t channel,
                                   int *out_raw);
esp_err_t ext_spi_adc_read_frame(ext_spi_adc_handle_t handle, ext_spi_adc_frame_t *out_frame);
esp_err_t ext_spi_adc_del(ext_spi_adc_handle_t handle);

#ifdef __cplusplus
}
#endif
