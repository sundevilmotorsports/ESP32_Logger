#include "sdcard.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#define LOG_CHANNEL_NAMES
#include "log_chnl.h"

static const char* TAG = "FILE_SYS";

//TODO: Finish implementing default_log_filename with nvs, make sure value updates every time default_log_filename is changed by uart function


// Global persistent variables - these stay in scope
static sdmmc_card_t* g_card = NULL;
// static sdmmc_host_t g_host;
// static sdspi_device_config_t g_slot_config;
static bool g_sdcard_initialized = false;

nvs_handle_t hnvs;
FILE *log_file = NULL;
SemaphoreHandle_t log_file_mutex;
static char current_log_filepath[MAX_FILE_NAME_LENGTH];


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
    strncpy(current_log_filepath, filename, sizeof(current_log_filepath) - 1);
    current_log_filepath[sizeof(current_log_filepath) - 1] = '\0';
    
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

esp_err_t nvs_get_log_name(char *buffer, size_t buffer_size) {
    if (buffer == NULL || buffer_size == 0) {
        ESP_LOGE(TAG, "Invalid buffer or buffer size");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (hnvs == 0) {
        ESP_LOGE(TAG, "NVS handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // get the required size for the string
    size_t required_size = 0;
    esp_err_t err = nvs_get_str(hnvs, "log_name", NULL, &required_size);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "log_name not found in NVS, using empty string");
        buffer[0] = '\0';  // Set empty string
        return ESP_ERR_NVS_NOT_FOUND;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get log_name size from NVS: %s", esp_err_to_name(err));
        buffer[0] = '\0';  // Set empty string as fallback
        return err;
    }
    
    // Check if buffer is large enough (including null terminator)
    if (required_size > buffer_size) {
        ESP_LOGE(TAG, "Buffer too small: need %zu bytes, have %zu bytes", required_size, buffer_size);
        buffer[0] = '\0';  // Set empty string
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Read the actual string
    err = nvs_get_str(hnvs, "log_name", buffer, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read log_name string from NVS: %s", esp_err_to_name(err));
        buffer[0] = '\0';  // Set empty string as fallback
        return err;
    }
    
    ESP_LOGD(TAG, "Successfully read log_name: '%s' (%zu bytes)", buffer, required_size - 1);
    return ESP_OK;
}

esp_err_t nvs_set_log_name(const char *log_name) {
    if (log_name == NULL) {
        ESP_LOGE(TAG, "log_name parameter is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (hnvs == 0) {
        ESP_LOGE(TAG, "NVS handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check string length to prevent excessive storage usage
    size_t name_len = strlen(log_name);
    if (name_len == 0) {
        ESP_LOGW(TAG, "Empty log_name provided");
    }
    
    if (name_len >= MAX_FILE_NAME_LENGTH) {
        ESP_LOGE(TAG, "log_name too long: %zu characters (max: %d)", name_len, MAX_FILE_NAME_LENGTH - 1);
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = validate_filename(log_name);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Invalid Filename!");
        return err;
    }
    
    // Write the string to NVS
    err = nvs_set_str(hnvs, "log_name", log_name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write log_name to NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    // Commit the changes to ensure they're saved
    err = nvs_commit(hnvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit log_name to NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    return ESP_OK;
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
        ESP_LOGI(TAG, "Incremented testno to %u", *testno);
    }
    
    return err;
}

esp_err_t nvs_set_testno(uint8_t testno){
    if(hnvs == 0){
        ESP_LOGE(TAG, "NVS handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Write and commit
    esp_err_t err = nvs_set_u8(hnvs, "testno", testno);
    if(err == ESP_OK) {
        err = nvs_commit(hnvs);
    }
    
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Failed to save testno: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Set testno to %u", testno);
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
    nvs_init();

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

    ESP_LOGI(TAG, "Initializing SD card in SPI mode...");

    // Configure SPI bus for SD card
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_12,     // Your CMD pin becomes MOSI
        .miso_io_num = GPIO_NUM_10,     // Your D0 pin becomes MISO
        .sclk_io_num = GPIO_NUM_11,     // Your CLK pin
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    gpio_config_t cs_config = {
        .pin_bit_mask = (1ULL << 13),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    gpio_config(&cs_config);
    gpio_set_level(GPIO_NUM_13, 1);

    vTaskDelay(pdMS_TO_TICKS(500));

    // Configure SPI device for SD card
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_13;      // Use D3 as CS
    slot_config.host_id = SPI2_HOST;

    // Configure SDMMC host for SPI
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 400;  // Start with low frequency

    // Configure mount options
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,  // Don't auto-format
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // Mount using SPI mode
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &g_card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card in SPI mode: %s", esp_err_to_name(ret));
        
        // Try even slower speed
        host.max_freq_khz = 200;
        ESP_LOGW(TAG, "Retrying with 200kHz clock...");
        
        ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &g_card);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SD card mount failed even at 200kHz: %s", esp_err_to_name(ret));
            spi_bus_free(SPI2_HOST);
            return;
        }
    }

    g_sdcard_initialized = true;
    ESP_LOGI(TAG, "SD card mounted successfully in SPI mode");

    // Rest of your existing code for card info and log file creation...
    if (g_card != NULL) {
        sdmmc_card_print_info(stdout, g_card);
        
        const char* card_type = (g_card->ocr & (1 << 30)) ? "SDHC/SDXC" : "SDSC";
        const char* speed_class = (g_card->csd.tr_speed > 25000000) ? "High Speed" : "Default Speed";
        
        ESP_LOGI(TAG, "SD card info: Name: %s, Type: %s, Speed: %s, Size: %lluMB", 
                 g_card->cid.name, card_type, speed_class,
                 ((uint64_t) g_card->csd.capacity) * g_card->csd.sector_size / (1024 * 1024));
    }

    // Initialize the first log file
    char default_log_filename[MAX_FILE_NAME_LENGTH >> 1];
    ret = nvs_get_log_name(default_log_filename, sizeof(default_log_filename));
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Could not receive default log name from NVS, writing data_log_ to NVS");
        strcpy(default_log_filename, "data_log_");
        ret = nvs_set_log_name("data_log_");
        if(ret != ESP_OK){
            ESP_LOGE(TAG, "Unable to write default log name to NVS");
        }
    }
    
    sdcard_create_numbered_log_file(default_log_filename);
}

esp_err_t sdcard_create_numbered_log_file(const char *filename){
    if (filename == NULL) {
        ESP_LOGE(TAG, "Filename parameter is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = validate_filename(filename);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Invalid Filename Format!");
        return err;
    }
    
    // Create internal buffer of adequate size
    char log_path[MAX_FILE_NAME_LENGTH];
    uint8_t testno = 0;
    
    // Increment the test number to get a NEW number
    err = nvs_increment_testno(&testno);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to increment testno: %s", esp_err_to_name(err));
        return err;
    }
    
    // Create the new log filename
    int written = snprintf(log_path, sizeof(log_path), "%s%s%03d%s", 
                          MOUNT_POINT, filename, testno, LOG_TYPE);
    
    // Check if the string was truncated
    if (written >= sizeof(log_path)) {
        ESP_LOGE(TAG, "Log filename too long for buffer");
        return ESP_ERR_INVALID_SIZE;
    }
    
    ESP_LOGI(TAG, "Created new log filename: %s (testno: %u)", log_path, testno);

    //Copy file path into global
    strncpy(current_log_filepath, log_path, sizeof(current_log_filepath));
    current_log_filepath[sizeof(current_log_filepath) - 1] = '\0';

    // Open the new log file
    return open_log_file(log_path);
}

static bool is_valid_fat32_filename_char(char ch) {
    // FAT32 invalid characters: \ / : * ? " < > |
    // Also exclude control characters (0-31) and DEL (127)
    if (ch < 32 || ch == 127) return false;
    
    switch (ch) {
        case '\\':
        case '/':
        case ':':
        case '*':
        case '?':
        case '"':
        case '<':
        case '>':
        case '|':
        case '.':   // Exclude periods as requested
            return false;
        default:
            return true;
    }
}


//TODO: Add checking the file system for already used filenames, possibly
static esp_err_t validate_filename(const char *filename) {
    if (filename == NULL || strlen(filename) == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t len = strlen(filename);
    
    // Check length (FAT32 supports up to 255 characters, but let's be conservative)
    if (len > MAX_FILE_NAME_LENGTH >> 1) {
        printf("Error: Filename too long (max %d characters)\n", MAX_FILE_NAME_LENGTH >> 1);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Check for invalid characters
    for (size_t i = 0; i < len; i++) {
        if (!is_valid_fat32_filename_char(filename[i])) {
            printf("Error: Invalid character '%c' at position %zu\n", filename[i], i);
            printf("Invalid chars: \\ / : * ? \" < > | .\n");
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    // Check for reserved names (case insensitive)
    const char* reserved_names[] = {
        "CON", "PRN", "AUX", "NUL",
        "COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9",
        "LPT1", "LPT2", "LPT3", "LPT4", "LPT5", "LPT6", "LPT7", "LPT8", "LPT9"
    };
    
    for (size_t i = 0; i < sizeof(reserved_names) / sizeof(reserved_names[0]); i++) {
        if (strcasecmp(filename, reserved_names[i]) == 0) {
            printf("Error: '%s' is a reserved filename\n", filename);
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    // Check that filename doesn't start or end with space
    if (filename[0] == ' ' || filename[len - 1] == ' ') {
        printf("Error: Filename cannot start or end with space\n");
        return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

// Add this function to allow read-only access from outside
const char* sdcard_get_current_log_filename(void) {
    return current_log_filepath;
}

