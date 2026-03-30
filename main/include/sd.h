#pragma once

#include <cstdio>
#include <cerrno>
#include <unistd.h>
#include <string>
#include <cstring>

#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class SDCard {
public:
    esp_err_t init() {
        mutex_ = xSemaphoreCreateMutex();
        if (!mutex_) {
            ESP_LOGE(TAG, "failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
        ESP_RETURN_ON_ERROR(mount_card(), TAG, "SD mount failed");
        return open_log();
    }

    esp_err_t write(const char *buf, size_t len) {
        if (!file_) return ESP_ERR_INVALID_STATE;
        if (!mutex_) return ESP_ERR_INVALID_STATE;
        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "failed to take mutex");
            return ESP_ERR_TIMEOUT;
        }
        ESP_LOGD(TAG, "writing %u bytes", (unsigned)len);
        size_t written = fwrite(buf, 1, len, file_);
        if (written != len) {
            int e = errno;
            ESP_LOGE(TAG, "fwrite wrote %u of %u bytes, errno=%d", (unsigned)written, (unsigned)len, e);
            if (ferror(file_)) {
                ESP_LOGE(TAG, "ferror set on file");
                clearerr(file_);
            }
        }
        fflush(file_);
        if (fsync(fileno(file_)) != 0) {
            ESP_LOGE(TAG, "fsync failed, errno=%d", errno);
        }
        xSemaphoreGive(mutex_);
        return written == len ? ESP_OK : ESP_FAIL;
    }

    void sync() {
        if (!file_) return;
        if (!mutex_) return;
        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) return;
        fflush(file_);
        if (fsync(fileno(file_)) != 0) {
            ESP_LOGE(TAG, "fsync failed during sync, errno=%d", errno);
        }
        xSemaphoreGive(mutex_);
    }

    void setName(std::string name) { file_name_ = std::move(name); }

    ~SDCard() {
        if (file_) { fflush(file_); fclose(file_); file_ = nullptr; }
        if (card_) { esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card_); card_ = nullptr; }
        if (mutex_) { vSemaphoreDelete(mutex_); mutex_ = nullptr; }
    }

private:
    static constexpr const char *TAG = "SDCard";

    static constexpr const char *MOUNT_POINT = "/sdcard";

    sdmmc_card_t     *card_        = nullptr;
    FILE             *file_        = nullptr;
    SemaphoreHandle_t mutex_       = nullptr;
    std::string       file_name_;

    esp_err_t mount_card() {
        sdmmc_host_t host        = SDMMC_HOST_DEFAULT();
        host.flags               = SDMMC_HOST_FLAG_1BIT;

        sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
        slot.clk                 = GPIO_NUM_11;
        slot.cmd                 = GPIO_NUM_12;
        slot.d0                  = GPIO_NUM_10;
        slot.width               = 1;
        slot.cd                  = SDMMC_SLOT_NO_CD;
        slot.wp                  = SDMMC_SLOT_NO_WP;

        esp_vfs_fat_sdmmc_mount_config_t mount = {
            .format_if_mount_failed = false,
            .max_files              = 4,
            .allocation_unit_size   = 16 * 1024,
            .disk_status_check_enable = false,
            .use_one_fat            = false,
        };

        esp_err_t err = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot, &mount, &card_);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "SD card mounted at %s", MOUNT_POINT);
        }
        return err;
    }

    esp_err_t open_log() {
        char path[128];
        int n = snprintf(path, sizeof(path), "%s/%s.csv", MOUNT_POINT, file_name_.c_str());
        if (n < 0 || (size_t)n >= sizeof(path)) {
            ESP_LOGE(TAG, "filename too long");
            return ESP_FAIL;
        }
        file_ = fopen(path, "w");
        if (!file_) { ESP_LOGE(TAG, "fopen('%s') failed, errno=%d", path, errno); return ESP_FAIL; }
        setvbuf(file_, nullptr, _IONBF, 0);
        return ESP_OK;
    }
};
