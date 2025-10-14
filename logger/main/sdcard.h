#ifndef SDCARD_H
#define SDCARD_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdmmc_cmd.h"

#define MOUNT_POINT "/sdcard/"
#define LOG_TYPE ".benji2"

#define PIN_NUM_MISO  5  // D0
#define PIN_NUM_MOSI  15 // D1
#define PIN_NUM_CLK   14 // CLK
#define PIN_NUM_CS    13 // CS

extern FILE *log_file;
extern SemaphoreHandle_t log_file_mutex;
extern char current_log_filename[32];

// Function declarations
void sdcard_init(void);
void sdcard_deinit(void);
bool sdcard_is_initialized(void);
sdmmc_card_t* sdcard_get_card_handle(void);
esp_err_t fast_log_buffer(const uint8_t *data_buffer, uint8_t buffer_len);

#endif