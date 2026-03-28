#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

inline constexpr std::size_t EXT_SPI_ADC_MAX_CHANNELS = 8;

using ext_adc_channel_t = std::uint8_t;

struct ext_spi_adc_t;
using ext_spi_adc_handle_t = ext_spi_adc_t *;

struct ext_spi_adc_bus_config_t {
    spi_host_device_t host_id;
    spi_dma_chan_t dma_chan;
    gpio_num_t sclk_io_num;
    gpio_num_t mosi_io_num;
    gpio_num_t miso_io_num;
    int max_transfer_sz;
    bool initialize_bus;
};

struct ext_spi_adc_device_config_t {
    gpio_num_t cs_io_num;
    int clock_speed_hz;
    std::uint8_t spi_mode;
    int queue_size;
};

struct ext_spi_adc_protocol_config_t {
    std::uint8_t channel_count;
    // For the current ADC, the logical channel select bits live in the first
    // transmitted byte and the second byte is zero by default.
    std::uint8_t channel_command_shift;
    std::uint16_t sample_mask;
};

struct ext_spi_adc_config_t {
    ext_spi_adc_bus_config_t bus;
    ext_spi_adc_device_config_t device;
    ext_spi_adc_protocol_config_t protocol;
};

struct ext_spi_adc_channel_config_t {
    // Raw 16-bit SPI frame in transmit order: tx_data[0] is shifted out first.
    std::uint8_t tx_data[2];
};

struct ext_spi_adc_frame_t {
    std::uint8_t channel_count;
    std::uint16_t values[EXT_SPI_ADC_MAX_CHANNELS];
};

esp_err_t ext_spi_adc_new(const ext_spi_adc_config_t *config, ext_spi_adc_handle_t *ret_handle);
esp_err_t ext_spi_adc_config_channel(ext_spi_adc_handle_t handle,
                                     ext_adc_channel_t channel,
                                     const ext_spi_adc_channel_config_t *config);
esp_err_t ext_spi_adc_read_channel(ext_spi_adc_handle_t handle,
                                   ext_adc_channel_t channel,
                                   int *out_raw);
esp_err_t ext_spi_adc_read_frame(ext_spi_adc_handle_t handle, ext_spi_adc_frame_t *out_frame);
esp_err_t ext_spi_adc_del(ext_spi_adc_handle_t handle);
