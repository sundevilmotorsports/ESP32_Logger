#ifndef SDCARD_H
#define SDCARD_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"

#define MOUNT_POINT "/sdcard"
#define LOG_TYPE ".benji2"

#define MAX_FILE_NAME_LENGTH 255

#define PIN_NUM_D0  GPIO_NUM_10  // D0
#define PIN_NUM_D1  GPIO_NUM_9 // D1
#define PIN_NUM_D2 GPIO_NUM_14
#define PIN_NUM_D3 GPIO_NUM_13
#define PIN_NUM_CLK   GPIO_NUM_11 // CLK
#define PIN_NUM_CMD GPIO_NUM_12

extern FILE *log_file;
extern SemaphoreHandle_t log_file_mutex;

// ============================================================================
// SD CARD INITIALIZATION AND MANAGEMENT
// ============================================================================

/**
 * @brief Initialize SD card in SPI mode and mount filesystem
 * 
 * This function:
 * - Initializes NVS
 * - Sets up SPI bus for SD card
 * - Mounts SD card filesystem
 * - Creates initial log file
 */
void sdcard_init(void);

/**
 * @brief Deinitialize SD card and cleanup resources
 */
void sdcard_deinit(void);

/**
 * @brief Check if SD card is initialized
 * @return true if initialized, false otherwise
 */
bool sdcard_is_initialized(void);

/**
 * @brief Get handle to SD card structure
 * @return Pointer to sdmmc_card_t or NULL if not initialized
 */
sdmmc_card_t* sdcard_get_card_handle(void);

// ============================================================================
// FILE OPERATIONS
// ============================================================================

/**
 * @brief Fast binary logging function with mutex protection
 * @param data_buffer Pointer to data to write
 * @param buffer_len Number of bytes to write
 * @return ESP_OK on success, error code on failure
 */
esp_err_t fast_log_buffer(const uint8_t *data_buffer, size_t buffer_len);

/**
 * @brief Create a new numbered log file
 * @param filename Base filename (without number and extension)
 * @return ESP_OK on success, error code on failure
 * 
 * Creates files like: /sdcard/{filename}001.benji2
 */
esp_err_t sdcard_create_numbered_log_file(const char *filename);

/**
 * @brief Get current log file path (read-only)
 * @return Pointer to current log filepath string
 */
const char* sdcard_get_current_log_filepath(void);

// ============================================================================
// FILE SYNC
// ============================================================================

/**
 * @brief Flush and sync the active log file to persistent storage
 *
 * This function:
 *  - Attempts to take the internal log file mutex (`log_file_mutex`)
 *  - Calls fflush() on the open FILE* (`log_file`) to flush stdio buffers
 *  - Calls fsync() on the file descriptor returned by fileno(log_file) to
 *    flush OS/device caches to the SD card
 *  - Releases the mutex
 *
 * Use this to reduce data loss risk on unexpected power loss. The call is
 * safe to call from multiple contexts; it will time out if the mutex cannot be
 * acquired.
 */
void sdcard_sync(void);

// ============================================================================
// NVS (NON-VOLATILE STORAGE) FUNCTIONS
// ============================================================================

/**
 * @brief Initialize NVS flash storage
 * 
 * Called automatically by sdcard_init()
 */
void nvs_init(void);

/**
 * @brief Set log filename in NVS storage
 * @param log_name Filename to store (will be validated)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t nvs_set_log_name(const char *log_name);

/**
 * @brief Get log filename from NVS storage
 * @param buffer Buffer to store retrieved filename
 * @param buffer_size Size of buffer
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not set
 */
esp_err_t nvs_get_log_name(char *buffer, size_t buffer_size);

/**
 * @brief Get current test number from NVS
 * @param testno Pointer to store retrieved test number
 * @return ESP_OK on success, error code on failure
 */
esp_err_t nvs_get_testno(uint8_t *testno);

/**
 * @brief Set test number in NVS
 * @param testno Test number to store
 * @return ESP_OK on success, error code on failure
 */
esp_err_t nvs_set_testno(uint8_t testno);

/**
 * @brief Increment test number in NVS (with rollover at 255)
 * @param testno Pointer to store new test number
 * @return ESP_OK on success, error code on failure
 */
esp_err_t nvs_increment_testno(uint8_t *testno);

// ============================================================================
// UTILITY FUNCTIONS (INTERNAL - NOT FOR EXTERNAL USE)
// ============================================================================

// Note: These functions are static in the .c file and should not be 
// declared in the header. They are internal implementation details:
//
// static esp_err_t validate_filename(const char *filename);
// static bool is_valid_fat32_filename_char(char ch);
// static esp_err_t open_log_file(const char *filename);

#endif