#include "ds3231.h"

static uint8_t bcd2dec(uint8_t v){ return (v>>4)*10 + (v & 0x0F); }
static uint8_t dec2bcd(uint8_t v){ return ((v/10)<<4) | (v%10); }

esp_err_t ds3231_init(ds3231_t *rtc, i2c_master_bus_handle_t bus, uint8_t addr, uint32_t scl_speed_hz) {
    if (!rtc || !bus) return ESP_ERR_INVALID_ARG;

    // Clamp to safe range
    if (scl_speed_hz == 0 || scl_speed_hz > DS3231_I2C_FREQ_FAST) {
        scl_speed_hz = DS3231_I2C_FREQ_STANDARD; // fallback to 100kHz
    }

    rtc->bus = bus;
    rtc->i2c_addr = addr;
    rtc->scl_speed_hz = scl_speed_hz;

    i2c_device_config_t dev_cfg = {
        .device_address = addr,
        .scl_speed_hz   = scl_speed_hz,
    };

    return i2c_master_bus_add_device(bus, &dev_cfg, &rtc->dev);
}

esp_err_t ds3231_set_time(ds3231_t *rtc, const struct tm *time) {
    if (!rtc || !time) return ESP_ERR_INVALID_ARG;

    uint8_t buf[8] = {
        0x00,
        dec2bcd(time->tm_sec),
        dec2bcd(time->tm_min),
        dec2bcd(time->tm_hour),
        dec2bcd(time->tm_wday + 1),       // DS3231: 1=Sunday
        dec2bcd(time->tm_mday),
        dec2bcd(time->tm_mon + 1),
        dec2bcd(time->tm_year - 100)      // tm_year since 1900
    };

    return i2c_master_transmit(rtc->dev, buf, sizeof(buf), -1);
}

esp_err_t ds3231_get_time(ds3231_t *rtc, struct tm *time) {
    if (!rtc || !time) return ESP_ERR_INVALID_ARG;

    uint8_t reg = 0x00, data[7];
    esp_err_t ret;

    ret = i2c_master_transmit(rtc->dev, &reg, 1, -1);
    if (ret != ESP_OK) return ret;

    ret = i2c_master_receive(rtc->dev, data, sizeof(data), -1);
    if (ret != ESP_OK) return ret;

    time->tm_sec  = bcd2dec(data[0] & 0x7F);
    time->tm_min  = bcd2dec(data[1]);
    time->tm_hour = bcd2dec(data[2] & 0x3F); // 24h
    time->tm_wday = bcd2dec(data[3]) - 1;    // 0=Sunday
    time->tm_mday = bcd2dec(data[4]);
    time->tm_mon  = bcd2dec(data[5]) - 1;
    time->tm_year = bcd2dec(data[6]) + 100;

    return ESP_OK;
}
