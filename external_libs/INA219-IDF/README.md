NA219-IDF

Minimal INA219 driver for ESP-IDF 5+ using the modern I²C bus/device API (driver/i2c_master.h).
No Arduino/Wire layer, no Component Manager required — works cleanly via PlatformIO lib_deps.

Features

ESP-IDF 5+ I²C Master (bus/device handles)

Read bus voltage, shunt voltage, current, power

Simple calibration API (set your shunt and max current)

Tiny footprint, header: ina219.h

Requirements

ESP-IDF 5.0+ (PlatformIO framework = espidf)

ESP32 target (tested), should work on other ESP-IDF 5+ devices with I²C

Pull-ups on SDA/SCL (typically 4.7 kΩ to 3.3 V). Internal pull-ups may be insufficient.

Installation (PlatformIO)

Add this repository URL to your project’s platformio.ini:

[env:esp32dev]
platform = espressif32
framework = espidf
board = esp32dev
monitor_speed = 115200

lib_deps =
  https://github.com/<YOUR-USER>/ina219-idf-pio


Replace <YOUR-USER> with your GitHub username (repo must be public).

Quick Start

Wiring (example for ESP32):

INA219 SDA → ESP32 GPIO21

INA219 SCL → ESP32 GPIO22

INA219 VCC → 3.3 V

INA219 GND → GND

INA219 accepts 3–5.5 V supply, but I²C lines must be 3.3 V logic on ESP32.

Example (main.c):

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "ina219.h"

#define SDA       21
#define SCL       22
#define I2C_PORT  I2C_NUM_0
#define INA_ADDR  0x40   // check your module’s address jumpers (0x40..0x4F)

void app_main(void) {
    // 1) Create I2C master bus
    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = { .enable_internal_pullup = true },
        .glitch_ignore_cnt = 7,
        .trans_queue_depth = 0
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // 2) Init INA219 on that bus
    ina219_t ina;
    ESP_ERROR_CHECK(ina219_init(&ina, bus, INA_ADDR, 400000));   // 400 kHz I2C

    // 3) Calibrate (set your real shunt and max current!)
    // Example: Rshunt = 0.1 Ω, Imax = 3.2 A
    ESP_ERROR_CHECK(ina219_calibrate(&ina, 0.1f, 3.2f));

    // 4) Read measurements
    while (1) {
        float vbus, vshunt, i, p;
        ESP_ERROR_CHECK(ina219_read_bus_voltage(&ina, &vbus));     // volts
        ESP_ERROR_CHECK(ina219_read_shunt_voltage(&ina, &vshunt)); // volts
        ESP_ERROR_CHECK(ina219_read_current(&ina, &i));            // amps
        ESP_ERROR_CHECK(ina219_read_power(&ina, &p));              // watts
        printf("Bus=%.3f V | Shunt=%.6f V | I=%.3f A | P=%.3f W\n", vbus, vshunt, i, p);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Optional cleanup (not reached in this loop):
    // ina219_deinit(&ina);
    // i2c_del_master_bus(bus);
}

API Overview

Header: ina219.h

esp_err_t ina219_init(ina219_t *ctx,
                      i2c_master_bus_handle_t bus,
                      uint8_t addr,
                      uint32_t scl_hz);

esp_err_t ina219_deinit(ina219_t *ctx);

esp_err_t ina219_calibrate(ina219_t *ctx,
                           float r_shunt_ohms,
                           float i_max_amps);

esp_err_t ina219_read_bus_voltage(ina219_t *ctx, float *volts);
esp_err_t ina219_read_shunt_voltage(ina219_t *ctx, float *volts);
esp_err_t ina219_read_current(ina219_t *ctx, float *amps);
esp_err_t ina219_read_power(ina219_t *ctx, float *watts);


Notes:

Calibration is required for accurate current/power:

current_LSB = Imax / 32767

calibration = 0.04096 / (current_LSB * Rshunt)

power_LSB = 20 * current_LSB

Bus voltage LSB = 4 mV (bits [15:3])

Shunt voltage LSB = 10 µV (signed)

Tips & Troubleshooting

If #include "ina219.h" isn’t found:

Ensure the repo URL is correct in lib_deps and the repo is public.

Remove .pio/ and rebuild (pio run -t clean && pio run).

Confirm framework = espidf (not Arduino).

If I²C reads fail:

Check wiring and pull-ups.

Try lower I²C speed (e.g., 100 kHz).

Verify the INA219 I²C address (jumpers) with a scanner.

License

MIT (or your preferred license).

Acknowledgements

Based on the INA219 datasheet and ESP-IDF 5+ I²C bus/device API.
