#ifndef SDCARD_H
#define SDCARD_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdmmc_cmd.h"

#define MOUNT_POINT "/sdcard/"
#define LOG_TYPE ".benji2"

#define MAX_FILE_NAME_LENGTH 128

#define PIN_NUM_MISO  GPIO_NUM_10  // D0
#define PIN_NUM_MOSI  GPIO_NUM_9 // D1
#define PIN_NUM_CLK   GPIO_NUM_11 // CLK
#define PIN_NUM_CS    13 // CS

extern FILE *log_file;
extern SemaphoreHandle_t log_file_mutex;

// Function declarations
void sdcard_init(void);
void sdcard_deinit(void);
bool sdcard_is_initialized(void);
sdmmc_card_t* sdcard_get_card_handle(void);
esp_err_t fast_log_buffer(const uint8_t *data_buffer, uint8_t buffer_len);
esp_err_t sdcard_create_numbered_log_file(const char *filename);
esp_err_t nvs_set_log_name(const char *log_name);
esp_err_t nvs_get_log_name(char *buffer, size_t buffer_size);
static esp_err_t validate_filename(const char *filename);
static bool is_valid_fat32_filename_char(char ch);
const char* sdcard_get_current_log_filename(void);
esp_err_t nvs_set_testno(uint8_t testno);
esp_err_t nvs_increment_testno(uint8_t *testno);
#endif