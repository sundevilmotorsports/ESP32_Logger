#include "sdcard.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

#define LOG_CHANNEL_NAMES
#include "log_chnl.h"

const char* TAG = "FILE_SYS";



// Global persistent variables - these stay in scope
static sdmmc_card_t* g_card = NULL;
static sdmmc_host_t g_host;
static sdspi_device_config_t g_slot_config;
static bool g_sdcard_initialized = false;

nvs_handle_t hnvs;
FILE *log_file = NULL;
SemaphoreHandle_t log_file_mutex;
char default_log_filename[32] = "logger_data_";
char current_log_filename[64] = "logger_data_";


static esp_err_t write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t append_file(FILE *file_handle, const uint8_t *data_buffer, size_t buffer_size)
{
    if (file_handle == NULL) {
        ESP_LOGE(TAG, "Invalid file handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (data_buffer == NULL) {
        ESP_LOGE(TAG, "Invalid data buffer");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (buffer_size == 0) {
        ESP_LOGW(TAG, "Buffer size is zero, nothing to write");
        return ESP_OK;
    }
    
    // Write all bytes at once (more efficient)
    size_t bytes_written = fwrite(data_buffer, sizeof(uint8_t), buffer_size, file_handle);
    
    if (bytes_written != buffer_size) {
        ESP_LOGE(TAG, "Failed to write all data. Expected: %zu, Written: %zu", buffer_size, bytes_written);
        return ESP_FAIL;
    }
    
    // Flush to ensure data is written to storage
    if (fflush(file_handle) != 0) {
        ESP_LOGE(TAG, "Failed to flush file buffer");
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Successfully appended %zu bytes to file", bytes_written);
    return ESP_OK;
}

// Open a new log file
static esp_err_t open_log_file(const char *filename) {
    xSemaphoreTake(log_file_mutex, portMAX_DELAY);
    
    // Close existing file if open
    if (log_file != NULL) {
        fflush(log_file);
        fclose(log_file);
        log_file = NULL;
        ESP_LOGI(TAG, "Closed previous log file");
    }
    
    // Open new file
    log_file = fopen(filename, "a");
    if (log_file == NULL) {
        ESP_LOGE(TAG, "Failed to open log file: %s", filename);
        xSemaphoreGive(log_file_mutex);
        return ESP_FAIL;
    }
    
    // Set buffer mode for better performance
    setvbuf(log_file, NULL, _IOFBF, 4096);  // Full buffering with 4KB buffer
    
    // Update current filename
    strncpy(current_log_filename, filename, sizeof(current_log_filename) - 1);
    current_log_filename[sizeof(current_log_filename) - 1] = '\0';
    
    ESP_LOGI(TAG, "Opened log file: %s", filename);
    
    // Build CSV header string
    char csv_header[2048] = {0};  // Adjust size as needed
    size_t header_len = 0;
    
    for (int i = 0; i < (sizeof(log_channel_names)/sizeof(log_channel_names[0])) - 1; i++) {
        size_t name_len = strlen(log_channel_names[i]);
        if (header_len + name_len < sizeof(csv_header) - 1) {  // Leave room for \0
            memcpy(csv_header + header_len, log_channel_names[i], name_len);
            header_len += name_len;
        } else {
            ESP_LOGW(TAG, "CSV header buffer too small, truncating");
            break;
        }
    }

    // Write header length as first 4 bytes (little-endian format)
    uint32_t header_len_le = header_len;  // Convert to little-endian if needed
    size_t len_written = fwrite(&header_len_le, sizeof(uint32_t), 1, log_file);
    if (len_written != 1) {
        ESP_LOGE(TAG, "Failed to write header length");
        fclose(log_file);
        log_file = NULL;
        xSemaphoreGive(log_file_mutex);
        return ESP_FAIL;
    }
    
    // Write CSV header data
    size_t written = fwrite(csv_header, 1, header_len, log_file);
    if (written != header_len) {
        ESP_LOGE(TAG, "Failed to write CSV header");
        fclose(log_file);
        log_file = NULL;
        xSemaphoreGive(log_file_mutex);
        return ESP_FAIL;
    }
    
    // Flush to ensure header is written immediately
    fflush(log_file);
    
    
    xSemaphoreGive(log_file_mutex);
    return ESP_OK;
}

esp_err_t fast_log_buffer(const uint8_t *data_buffer, uint8_t buffer_len) {
    if (data_buffer == NULL || buffer_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (log_file_mutex == NULL || log_file == NULL) {
        ESP_LOGW(TAG, "SD card not initialized or file not open");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Take mutex with timeout to avoid indefinite blocking
    if (xSemaphoreTake(log_file_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire log file mutex within timeout");
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t result = ESP_OK;
    
    // Critical section - file operations
    if (log_file != NULL) {
        size_t written = fwrite(data_buffer, sizeof(uint8_t), buffer_len, log_file);
        
        if (written != buffer_len) {
            ESP_LOGE(TAG, "Log write failed: %zu/%zu bytes", written, buffer_len);
            result = ESP_FAIL;
        } else {
            // Only flush periodically for performance
            static uint32_t write_count = 0;
            if (++write_count % 10 == 0) {
                fflush(log_file);
            }
        }
    } else {
        result = ESP_ERR_INVALID_STATE;
    }
    
    // Always release the mutex
    xSemaphoreGive(log_file_mutex);
    
    return result;
}

esp_err_t nvs_get_testno(uint8_t *testno) {
    if (testno == NULL) {
        ESP_LOGE(TAG, "testno pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (hnvs == 0) {
        ESP_LOGE(TAG, "NVS handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t err = nvs_get_u8(hnvs, "testno", testno);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read testno from NVS: %s", esp_err_to_name(err));
        *testno = 0; // Set safe default value
    }
    
    return err;
}

esp_err_t nvs_increment_testno(uint8_t *testno){
    if(hnvs == 0){
        ESP_LOGE(TAG, "NVS handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t err = nvs_get_testno(testno);
    
    // nvs_get_testno already handles errors and sets default to 0
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        return err;  // Only fail on real errors, not "not found"
    }
    
    // Increment with rollover
    *testno = (*testno == 255) ? 0 : *testno + 1;
    
    // Write and commit
    err = nvs_set_u8(hnvs, "testno", *testno);
    if(err == ESP_OK) {
        err = nvs_commit(hnvs);
    }
    
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Failed to save testno: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Incremented testno to %u", testno);
    }
    
    return err;
}

void nvs_init(){
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    ret = nvs_open("storage", NVS_READWRITE, &hnvs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(ret));
        return;
    }
}

void sdcard_init(){
    
    esp_err_t ret;

    // Create mutex if not already created
    if (log_file_mutex == NULL) {
        log_file_mutex = xSemaphoreCreateMutex();
        if (log_file_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create log file mutex");
            return;
        }
    }

    // Prevent double initialization
    if (g_sdcard_initialized) {
        ESP_LOGW(TAG, "SD card already initialized");
        return;
    }

    // Initialize global host configuration (persistent)
    g_host = (sdmmc_host_t)SDSPI_HOST_DEFAULT();
    g_host.max_freq_khz = 400;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(g_host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI initialized");

    gpio_config_t cs_config = {
        .pin_bit_mask = (1ULL << PIN_NUM_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cs_config);
    gpio_set_level(PIN_NUM_CS, 1);  // Set CS high

    vTaskDelay(pdMS_TO_TICKS(500));

    // Initialize global slot configuration (persistent)
    g_slot_config = (sdspi_device_config_t)SDSPI_DEVICE_CONFIG_DEFAULT();
    g_slot_config.gpio_cs = PIN_NUM_CS;
    g_slot_config.host_id = g_host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // Mount and get persistent card handle
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &g_host, &g_slot_config, &mount_config, &g_card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount filesystem: %s", esp_err_to_name(ret));
        spi_bus_free(g_host.slot);
        return;
    }

    g_sdcard_initialized = true;
    ESP_LOGI(TAG, "SD card mounted successfully");

    if (g_card != NULL) {
        sdmmc_card_print_info(stdout, g_card);
    }

    
    uint8_t testno = 0;
    nvs_increment_testno(&testno);

    // Initialize the first log file
    char log_name[64];
    snprintf(log_name, sizeof(log_name), "%s%s%03d%s", MOUNT_POINT, default_log_filename, testno, LOG_TYPE);
    open_log_file(log_name);
}