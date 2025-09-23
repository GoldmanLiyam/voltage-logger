#pragma once
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <time.h>
#include <stdint.h>

#define DS3231_I2C_FREQ_STANDARD 100000U   // 100 kHz
#define DS3231_I2C_FREQ_FAST     400000U   // 400 kHz (max for DS3231)

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t  i2c_addr;
    uint32_t scl_speed_hz;
} ds3231_t;

/**
 * @brief Initialize DS3231 on given I2C bus
 * @param rtc   Handle
 * @param bus   Existing I2C master bus
 * @param addr  I2C address (usually 0x68)
 * @param scl_speed_hz  I2C speed in Hz (<= 400000)
 */
esp_err_t ds3231_init(ds3231_t *rtc, i2c_master_bus_handle_t bus, uint8_t addr, uint32_t scl_speed_hz);

esp_err_t ds3231_set_time(ds3231_t *rtc, const struct tm *time);
esp_err_t ds3231_get_time(ds3231_t *rtc, struct tm *time);
