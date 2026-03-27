#pragma once

#include "ext_spi_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ext_adc_oneshot_unit_t *ext_adc_oneshot_unit_handle_t;

typedef struct {
    ext_spi_adc_config_t driver_config;
} ext_adc_oneshot_unit_init_cfg_t;

typedef struct {
    // Raw 16-bit SPI frame in transmit order: tx_data[0] is shifted out first.
    uint8_t tx_data[2];
} ext_adc_oneshot_chan_cfg_t;

esp_err_t ext_adc_oneshot_new_unit(const ext_adc_oneshot_unit_init_cfg_t *init_config,
                                   ext_adc_oneshot_unit_handle_t *ret_unit);
esp_err_t ext_adc_oneshot_config_channel(ext_adc_oneshot_unit_handle_t unit,
                                         ext_adc_channel_t channel,
                                         const ext_adc_oneshot_chan_cfg_t *config);
esp_err_t ext_adc_oneshot_read(ext_adc_oneshot_unit_handle_t unit,
                               ext_adc_channel_t channel,
                               int *out_raw);
esp_err_t ext_adc_oneshot_read_frame(ext_adc_oneshot_unit_handle_t unit,
                                     ext_spi_adc_frame_t *out_frame);
esp_err_t ext_adc_oneshot_del_unit(ext_adc_oneshot_unit_handle_t unit);

#ifdef __cplusplus
}
#endif
