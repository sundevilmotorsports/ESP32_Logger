#pragma once

#include <cstdio>

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
        ESP_RETURN_ON_ERROR(mount_card(), TAG, "SD mount failed");
        return open_log();
    }

    esp_err_t write(const char *buf, size_t len) {
        if (!file_) return ESP_ERR_INVALID_STATE;
        xSemaphoreTake(mutex_, portMAX_DELAY);
        size_t written = fwrite(buf, 1, len, file_);
        if (++write_count_ % 10 == 0) fflush(file_);
        xSemaphoreGive(mutex_);
        return written == len ? ESP_OK : ESP_FAIL;
    }

    void sync() {
        if (!file_) return;
        xSemaphoreTake(mutex_, portMAX_DELAY);
        fflush(file_);
        fsync(fileno(file_));
        xSemaphoreGive(mutex_);
    }

    ~SDCard() {
        if (file_) { fflush(file_); fclose(file_); }
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card_);
    }

private:
    static constexpr const char *TAG = "SDCard";

    static constexpr const char *MOUNT_POINT = "/sdcard";

    sdmmc_card_t     *card_        = nullptr;
    FILE             *file_        = nullptr;
    SemaphoreHandle_t mutex_       = nullptr;
    uint32_t          write_count_ = 0;

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

        return esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot, &mount, &card_);
    }

    esp_err_t open_log() {
        file_ = fopen("/sdcard/log.csv", "w");
        if (!file_) { ESP_LOGE(TAG, "fopen failed"); return ESP_FAIL; }
        setvbuf(file_, nullptr, _IOFBF, 4096);
        return ESP_OK;
    }
};
