#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "ina219.h"

#define TAG "INA219_IDF"

// INA219 registers
#define INA219_REG_CONFIG   0x00
#define INA219_REG_SHUNT_V  0x01
#define INA219_REG_BUS_V    0x02
#define INA219_REG_POWER    0x03
#define INA219_REG_CURRENT  0x04
#define INA219_REG_CALIB    0x05

static esp_err_t wr16_be(ina219_t *c, uint8_t reg, uint16_t be)
{
    uint8_t buf[3] = { reg, (uint8_t)(be >> 8), (uint8_t)be };
    return i2c_master_transmit(c->dev, buf, sizeof(buf), 1000 /* ms */);
}

static esp_err_t rd16_be(ina219_t *c, uint8_t reg, uint16_t *out_be)
{
    esp_err_t err = i2c_master_transmit(c->dev, &reg, 1, 1000);
    if (err != ESP_OK) return err;
    uint8_t rx[2] = {0};
    err = i2c_master_receive(c->dev, rx, 2, 1000);
    if (err != ESP_OK) return err;
    *out_be = ((uint16_t)rx[0] << 8) | rx[1];
    return ESP_OK;
}

esp_err_t ina219_init(ina219_t *c, i2c_master_bus_handle_t bus, uint8_t addr, uint32_t scl_hz)
{
    if (!c || !bus) return ESP_ERR_INVALID_ARG;
    memset(c, 0, sizeof(*c));
    c->bus  = bus;
    c->addr = addr;

    i2c_device_config_t dev_cfg = {
        .device_address = addr,
        .scl_speed_hz   = scl_hz ? scl_hz : 400000
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &c->dev), TAG, "add_device failed");

    // Optional: write a default CONFIG register value if required (based on the datasheet and the operating mode you want).
    // wr16_be(c, INA219_REG_CONFIG, 0x019F);

    c->current_lsb = 0.0f;
    c->power_lsb   = 0.0f;
    return ESP_OK;
}

esp_err_t ina219_deinit(ina219_t *c)
{
    if (!c) return ESP_ERR_INVALID_ARG;
    if (c->dev) {
        i2c_master_bus_rm_device(c->dev);
        c->dev = NULL;
    }
    memset(c, 0, sizeof(*c));
    return ESP_OK;
}

esp_err_t ina219_calibrate(ina219_t *c, float r_shunt, float i_max)
{
    if (!c || r_shunt <= 0.0f || i_max <= 0.0f) return ESP_ERR_INVALID_ARG;

    // current_LSB = Imax/32767
    c->current_lsb = i_max / 32767.0f;
    // power_LSB = 20 * current_LSB
    c->power_lsb   = 20.0f * c->current_lsb;

    // calibration = 0.04096 / (current_LSB * Rshunt)
    float calib_f = 0.04096f / (c->current_lsb * r_shunt);
    uint16_t calib = (uint16_t)(calib_f + 0.5f);
    if (calib == 0) calib = 1;

    return wr16_be(c, INA219_REG_CALIB, calib);
}

esp_err_t ina219_read_shunt_voltage(ina219_t *c, float *v)
{
    if (!c || !v) return ESP_ERR_INVALID_ARG;
    uint16_t be;
    ESP_RETURN_ON_ERROR(rd16_be(c, INA219_REG_SHUNT_V, &be), TAG, "read shunt");
    int16_t raw = (int16_t)be;
    *v = raw * 10e-6f; // 10 µV per LSB
    return ESP_OK;
}

esp_err_t ina219_read_bus_voltage(ina219_t *c, float *v)
{
    if (!c || !v) return ESP_ERR_INVALID_ARG;
    uint16_t be;
    ESP_RETURN_ON_ERROR(rd16_be(c, INA219_REG_BUS_V, &be), TAG, "read bus");
    uint16_t raw = (be >> 3) & 0x1FFF; // bits [15:3]
    *v = raw * 0.004f; // 4 mV per LSB
    return ESP_OK;
}

esp_err_t ina219_read_current(ina219_t *c, float *a)
{
    if (!c || !a) return ESP_ERR_INVALID_ARG;
    if (c->current_lsb <= 0.0f) return ESP_ERR_INVALID_STATE;
    uint16_t be;
    ESP_RETURN_ON_ERROR(rd16_be(c, INA219_REG_CURRENT, &be), TAG, "read current");
    int16_t raw = (int16_t)be;
    *a = raw * c->current_lsb;
    return ESP_OK;
}

esp_err_t ina219_read_power(ina219_t *c, float *w)
{
    if (!c || !w) return ESP_ERR_INVALID_ARG;
    if (c->power_lsb <= 0.0f) return ESP_ERR_INVALID_STATE;
    uint16_t be;
    ESP_RETURN_ON_ERROR(rd16_be(c, INA219_REG_POWER, &be), TAG, "read power");
    *w = be * c->power_lsb;
    return ESP_OK;
}
