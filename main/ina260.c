//Created by Alex Rumer 9/6/2025
#include "driver/i2c.h"
#include "esp_log.h"
#include "ina260.h"


static uint8_t buffer[2];
static const char *TAG = "INA260";

esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Equivalent to HAL_I2C_Mem_Read
static esp_err_t i2c_read_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Restart and read data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void ina260Init() {
    buffer[0] = 0;
    buffer[1] = 1;
}

uint16_t getCurrent() {
    esp_err_t ret = i2c_read_register(INA260_DEV_ID, INA260_REG_CURR, buffer, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return 0;
    }
    return (buffer[0] << 8) | buffer[1];
}

uint16_t getVoltage() {
    esp_err_t ret = i2c_read_register(INA260_DEV_ID, INA260_REG_VBUS, buffer, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return 0;
    }
    return (buffer[0] << 8) | buffer[1];
}

uint8_t checkMfgID() {
    esp_err_t ret = i2c_read_register(INA260_DEV_ID, INA260_REG_MFG, buffer, 2);
    if (ret == ESP_OK && buffer[0] == 0x54 && buffer[1] == 0x49) {
        return 1;
    }
    return 0;
}

uint8_t checkDieID() {
    esp_err_t ret = i2c_read_register(INA260_DEV_ID, INA260_REG_DIE, buffer, 2);
    if (ret == ESP_OK && buffer[0] == 0x22 && buffer[1] == 0x70) {
        return 1;
    }
    return 0;
}
