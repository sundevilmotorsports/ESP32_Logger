#pragma once

#include <fstream>
#include <functional>
#include <string>
#include <string_view>
#include <mutex>
#include <filesystem>
#include <vector>
#include <array>
#include <cstring>

#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"

namespace fs = std::filesystem;

class SDCard {
public:
    static constexpr size_t SECTOR_SIZE = 512;

    explicit SDCard() = default;

    esp_err_t init() {
        if (auto err = mount_card(); err != ESP_OK) return err;

        int next = get_next_log_index();
        file_name_ = format_log_name(next);

        return open_log();
    }

    esp_err_t next_log() {
        std::lock_guard lock(mutex_);

        if (file_.is_open()) {
            if (buf_len_ > 0) flush_buf();
            file_.flush();
            file_.close();
        }

        int next = get_next_log_index();
        file_name_ = format_log_name(next);

        return open_log();
    }

    esp_err_t write(std::string_view data) {
        if (!file_.is_open()) return ESP_ERR_INVALID_STATE;

        std::lock_guard lock(mutex_);

        while (!data.empty()) {
            const size_t space    = SECTOR_SIZE - buf_len_;
            const size_t to_copy  = std::min(data.size(), space);

            std::memcpy(buf_.data() + buf_len_, data.data(), to_copy);
            buf_len_ += to_copy;
            data      = data.substr(to_copy);

            if (buf_len_ == SECTOR_SIZE) {
                if (auto err = flush_buf(); err != ESP_OK) return err;
            }
        }
        return ESP_OK;
    }

    esp_err_t sync() {
        if (!file_.is_open()) return ESP_ERR_INVALID_STATE;
        std::lock_guard lock(mutex_);
        if (auto err = flush_buf(); err != ESP_OK) return err;
        file_.flush();
        return ESP_OK;
    }

    ~SDCard() {
        if (file_.is_open()) {
            if (buf_len_ > 0) flush_buf();
            file_.flush();
            file_.close();
        }
        if (card_) esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card_);
    }

    std::vector<std::string> get_logs_list() {
        std::vector<std::string> result;
        std::error_code ec;
        for (const auto &entry : fs::directory_iterator(MOUNT_POINT, ec)) {
            if (!ec && entry.is_regular_file(ec))
                result.push_back(entry.path().filename().string());
            ec.clear();
        }
        return result;
    }

    uint64_t get_file_size(const std::string& name) {
        std::error_code ec;
        auto sz = fs::file_size(fs::path(MOUNT_POINT) / name, ec);
        return ec ? 0 : static_cast<uint64_t>(sz);
    }

    // Returns the filename (with .csv) of the currently open log.
    std::string get_current_name() const { return file_name_ + ".csv"; }

    // Flush pending writes then call cb for each chunk of the current log file.
    void stream_current(size_t chunk_size, std::function<void(const uint8_t *, size_t)> cb) {
        sync();
        std::ifstream f(fs::path(MOUNT_POINT) / (file_name_ + ".csv"), std::ios::binary);
        if (!f) return;
        std::vector<char> buf(chunk_size);
        while (true) {
            f.read(buf.data(), static_cast<std::streamsize>(chunk_size));
            auto n = f.gcount();
            if (n <= 0) break;
            cb(reinterpret_cast<const uint8_t *>(buf.data()), static_cast<size_t>(n));
        }
    }

    // Call cb for each chunk of a named file (read-only, no sync needed).
    void stream_file(const std::string& name, size_t chunk_size, std::function<void(const uint8_t *, size_t)> cb) {
        std::ifstream f(fs::path(MOUNT_POINT) / name, std::ios::binary);
        if (!f) return;
        std::vector<char> buf(chunk_size);
        while (true) {
            f.read(buf.data(), static_cast<std::streamsize>(chunk_size));
            auto n = f.gcount();
            if (n <= 0) break;
            cb(reinterpret_cast<const uint8_t *>(buf.data()), static_cast<size_t>(n));
        }
    }

private:
    static constexpr const char *TAG         = "SDCard";
    static constexpr const char *MOUNT_POINT = "/sdcard";

    sdmmc_card_t *card_ = nullptr;
    std::ofstream file_;
    std::mutex    mutex_;
    std::string   file_name_;

    std::array<char, SECTOR_SIZE> buf_    {};
    size_t                        buf_len_ = 0;

    int extract_log_index(const std::string& name) {
        // logXXXX.csv
        if (name.size() != 11) return -1;
        if (name[0] != 'l' || name[1] != 'o' || name[2] != 'g') return -1;
        if (name[7] != '.' || name[8] != 'c' || name[9] != 's' || name[10] != 'v') return -1;

        int value = 0;

        for (int i = 3; i < 7; i++) {
            if (name[i] < '0' || name[i] > '9') return -1;
            value = value * 10 + (name[i] - '0');
        }

        return value;
    }

    int get_next_log_index() {
        auto logs = get_logs_list();

        int max_index = 0;
        for (const auto& log : logs) {
            int idx = extract_log_index(log);
            if (idx > max_index) max_index = idx;
        }

        return max_index + 1;
    }

    std::string format_log_name(int index) {
        char buf[16];
        snprintf(buf, sizeof(buf), "log%04d", index);
        return std::string(buf);
    }

    esp_err_t flush_buf() {
        if (buf_len_ == 0) return ESP_OK;
        if (!file_.write(buf_.data(), buf_len_)) {
            ESP_LOGE(TAG, "write failed for %zu bytes", buf_len_);
            return ESP_FAIL;
        }
        buf_len_ = 0;
        return ESP_OK;
    }

    esp_err_t mount_card() {
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.flags        = SDMMC_HOST_FLAG_1BIT;

        sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
        slot.clk   = GPIO_NUM_11;
        slot.cmd   = GPIO_NUM_12;
        slot.d0    = GPIO_NUM_10;
        slot.width = 1;
        slot.cd    = SDMMC_SLOT_NO_CD;
        slot.wp    = SDMMC_SLOT_NO_WP;

        constexpr esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
            .format_if_mount_failed   = false,
            .max_files                = 4,
            .allocation_unit_size     = 16 * 1024,
            .disk_status_check_enable = false,
            .use_one_fat              = false,
        };

        const esp_err_t err = esp_vfs_fat_sdmmc_mount(
            MOUNT_POINT, &host, &slot, &mount_cfg, &card_
        );
        if (err == ESP_OK) ESP_LOGI(TAG, "SD card mounted at %s", MOUNT_POINT);
        return err;
    }

    esp_err_t open_log() {
        if (file_name_.empty()) {
            ESP_LOGE(TAG, "file name not set");
            return ESP_ERR_INVALID_ARG;
        }

        const fs::path path = fs::path(MOUNT_POINT) / (file_name_ + ".csv");

        file_.rdbuf()->pubsetbuf(nullptr, 0);
        file_.open(path, std::ios::out | std::ios::trunc);
        if (!file_.is_open()) {
            ESP_LOGE(TAG, "failed to open '%s'", path.c_str());
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Logging to %s", path.c_str());
        return ESP_OK;
    }
};
