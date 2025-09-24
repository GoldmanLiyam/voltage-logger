#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t addr;
    float current_lsb;  // A per LSB (set by calibrate)
    float power_lsb;    // W per LSB (= 20 * current_lsb)
} ina219_t;

/**
 * Initialize INA219 device on an existing I2C master bus.
 * @param ctx    Pointer to context (output).
 * @param bus    Created I2C master bus handle (i2c_new_master_bus).
 * @param addr   I2C address (0x40..0x4F typically).
 * @param scl_hz SCL frequency (e.g. 400000). If 0, defaults to 400kHz.
 */
esp_err_t ina219_init(ina219_t *ctx, i2c_master_bus_handle_t bus, uint8_t addr, uint32_t scl_hz);

/** Remove device from bus and reset context. */
esp_err_t ina219_deinit(ina219_t *ctx);

/**
 * Calibrate for accurate current/power according to shunt resistance and max current.
 * current_LSB = Imax / 32767
 * calibration = 0.04096 / (current_LSB * Rshunt)
 * power_LSB   = 20 * current_LSB
 *
 * @param r_shunt_ohms  Shunt resistance in ohms (e.g. 0.1f)
 * @param i_max_amps    Max expected current (e.g. 3.2f)
 */
esp_err_t ina219_calibrate(ina219_t *ctx, float r_shunt_ohms, float i_max_amps);

/** Read bus voltage in volts (LSB=4mV, bits [15:3]). */
esp_err_t ina219_read_bus_voltage(ina219_t *ctx, float *volts);

/** Read shunt voltage in volts (signed, LSB=10µV). */
esp_err_t ina219_read_shunt_voltage(ina219_t *ctx, float *volts);

/** Read current in amperes (requires successful calibrate). */
esp_err_t ina219_read_current(ina219_t *ctx, float *amps);

/** Read power in watts (requires successful calibrate). */
esp_err_t ina219_read_power(ina219_t *ctx, float *watts);

#ifdef __cplusplus
}
#endif
