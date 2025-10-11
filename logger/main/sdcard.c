#include "sdcard.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/semphr.h"

const char* TAG = "FILE_SYS";


FILE *log_file = NULL;
SemaphoreHandle_t log_file_mutex;
char current_log_filename[64] = "/sdcard/logger_data_001.bin";

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
    
    xSemaphoreGive(log_file_mutex);
    return ESP_OK;
}

esp_err_t fast_log_buffer(FILE *file_handle, const uint8_t *data_buffer, ) {
    if (log_file == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Write entire buffer at once - fastest method
    size_t written = fwrite(logBuffer, sizeof(uint8_t), CH_COUNT, log_file);
    
    if (written != CH_COUNT) {
        ESP_LOGE(TAG, "Log write failed: %zu/%d bytes", written, CH_COUNT);
        return ESP_FAIL;
    }
    
    // Only flush periodically, not every write (for performance)
    static uint32_t write_count = 0;
    if (++write_count % 10 == 0) {  // Flush every 10 writes
        fflush(log_file);
    }
    
    return ESP_OK;
}