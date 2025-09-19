#ifndef INA260_H
#define INA260_H

#include <stdint.h>
#include <stdbool.h>

//TODO: Check I2C configuration and register adresses

// INA260 I2C address (default)
#define INA260_DEFAULT_ADDR 0x40

// INA260 Register Addresses
#define INA260_DEV_ID   0b1000000
#define INA260_REG_CURR 0x01
#define INA260_REG_VBUS 0x02
#define INA260_REG_MFG  0xfe
#define INA260_REG_DIE  0xff

//I2C Config
#define I2C_MASTER_SCL_IO           GPIO_NUM_47      // Set your SCL pin
#define I2C_MASTER_SDA_IO           GPIO_NUM_48      // Set your SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t i2c_addr;
} ina260_t;

//Initialize I2C device
esp_err_t i2c_master_init(void);

// Initialize INA260 device
bool ina260_init(ina260_t *dev, uint8_t i2c_addr);

// Read bus voltage in millivolts
bool ina260_read_bus_voltage(const ina260_t *dev, uint16_t *voltage_mv);

// Read current in milliamps
bool ina260_read_current(const ina260_t *dev, int16_t *current_ma);

// Read power in milliwatts
bool ina260_read_power(const ina260_t *dev, uint16_t *power_mw);

// Write configuration register
bool ina260_write_config(const ina260_t *dev, uint16_t config);

// Read configuration register
bool ina260_read_config(const ina260_t *dev, uint16_t *config);

uint16_t getVoltage(void);
uint16_t getCurrent(void);

#ifdef __cplusplus
}
#endif

#endif // INA260_H