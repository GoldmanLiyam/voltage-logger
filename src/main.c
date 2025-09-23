#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "ina219.h"
#include <ssd1306.h>
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_err.h"
#include "esp_log.h"
#include <time.h>
#include "ds3231.h"
#include "driver/adc.h"
#include "driver/dac.h"

// DEFINES:
#define BTN_PIN                 16              // Button GPIO 16
#define LED_PIN                 4               // LED GPIO 4
#define SCL                     22              // SCL GPIO 22
#define SDA                     21              // SDA GPIO 21
#define MOSI                    23              // MOSI GPIO 23
#define MISO                    19              // MISO GPIO 19
#define SCK                     18              // SCK GPIO 18
#define CS                      5               // CS GPIO 5
#define TAG                     "APP"           // Tag for esp_log
#define GLITCH_IGNORE_COUNT     7               // Glitch ignore count for I2C
#define OLED_ADDR               0x3C            // SSD1306 address
#define OLED_CONTRAST           255             // OLED brightness 
#define INA_ADDR                0x40            // INA219 address
#define R_SHUNT                 0.1             // R shunt value
#define MAX_INA_AMP             3.2             // Max INA219 current value 
#define READ_PERIOD             250             // Timer period (Lowest possible: 250)
#define DS3231_ADDR             0x68            // DS3231 address
#define I2C_MODE                400000          // I2C speed mode - Fast mode
#define SPI_FREQ                20000           // SD reader limit
#define MAX_FILES               1               // Max files to be opened at once - 1
#define ALLOC_UNIT_SIZE         16384           // File size in SD card - 16KB
#define HIGH                    1               // Logic HIGH
#define LOW                     0               // Logic LOW
#define DAC_RANGE               255             // 8 bit PWM
#define ADC_WIDTH               4095            // 12 bit ADC
#define POT_DIFF                5               // Minimum difference to change DAC
#define DEBOUNCE_DELAY          50              // Button debounce
#define BTN_STACK_DEPTH         3400            // Checked btn depth
#define READ_STACK_DEPTH        2600            // Checked read depth
#define CHANGE_VOLT_STACK_DEPTH 2300            // Checked change_volt depth
#define FILE_STACK_DEPTH        4400            // Checked file depth
#define OLED_STACK_DEPTH        2900            // Checked oled depth
#define OLED_WIDTH              128             // 128 pixel wide
#define OLED_HEIGHT_MAP         9               // 32(pixels) / 3.3(V) ~ 9
#define OLED_HEIGHT             32              // 32 pixel high

// STRUCTS:
// Readings data struct:
struct READ_DATA {
    float v;
    struct tm time;
};


// HANDLERS:
// FreeRTOS handlers:
TimerHandle_t read_timer_handle = NULL;         // Reading timer handle
QueueHandle_t oled_q = NULL;                    // Queue to send readings to the OLED
QueueHandle_t file_q = NULL;                    // Queue to send readings to the sd card
SemaphoreHandle_t read_semaphore = NULL;        // Semaphore to control the readings
TaskHandle_t btn_task_handle;                   // Task handler for btn_task         

// I2C handlers:
i2c_master_bus_handle_t master_bus_handle;      // Master bus handle
ssd1306_handle_t display_handle = NULL;         // OLED handle
ina219_t ina_handle;                            // Voltage sensor handle
ds3231_t rtc_handle;                            // RTC handle


// GLOBAL VARIABLES:
struct tm curr_time;                            // Get RTC time data to this struct
const char* mount_path = "/sdcard";             // File mount path
sdmmc_card_t *sd_card;                          // SD card handle
FILE *f;                                        // File pointer


// FUNCTIONS:
// ISR Function:
static void IRAM_ATTR btn_isr(void* arg) {
    static TickType_t last_intr = 0;
    TickType_t curr_intr = xTaskGetTickCountFromISR();
    if ((curr_intr - last_intr) >= pdMS_TO_TICKS(DEBOUNCE_DELAY)) {     // Check debounce
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(btn_task_handle, &hpw);                  // Notify and wake btn_task
        if (hpw) portYIELD_FROM_ISR();                                  // Context switch
    }
    last_intr = curr_intr;
}

// Timer Function:
void ina_read_timer(TimerHandle_t xTimer) {
    xSemaphoreGive(read_semaphore);             // Make readings available
}

// TASKS Declaration:
void btn_task(void* pvParameter);
void read_task(void* pvParameter);
void change_volt_task(void* pvParameter);
void file_task(void* pvParameter);
void oled_task(void* pvParameter);

// Helpful Functions Declaration:
void change_system_status(bool system_status, bool *first_intr);
void spi_set();


// MAIN CODE:
void app_main() {

    // GPIO SET:
    // LED:
    ESP_ERROR_CHECK(gpio_reset_pin(LED_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT));

    // Button:
    ESP_ERROR_CHECK(gpio_reset_pin(BTN_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(BTN_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(BTN_PIN, GPIO_PULLUP_ONLY));


    // I2C MASTER SET:
    i2c_master_bus_config_t master_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
        .glitch_ignore_cnt = GLITCH_IGNORE_COUNT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCL,
        .sda_io_num = SDA,
        .trans_queue_depth = 0
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&master_bus_cfg, &master_bus_handle));


    // I2C ADD DEVICES:
    // I2C Add OLED Device:
    ssd1306_config_t display_cfg = {
        .display_enabled = true,
        .i2c_address = OLED_ADDR,
        .i2c_clock_speed = I2C_MODE
    };
    ESP_ERROR_CHECK(ssd1306_init(master_bus_handle, &display_cfg, &display_handle));
    
    if (display_handle == NULL) {
        ESP_LOGE(TAG, "OLED init failed");
        return;
    }

    ESP_ERROR_CHECK(ssd1306_set_contrast(display_handle, OLED_CONTRAST));
    ESP_ERROR_CHECK(ssd1306_clear_display(display_handle, false));

    // I2C Add INA219 Device:
    ESP_ERROR_CHECK(ina219_init(&ina_handle, master_bus_handle, INA_ADDR, I2C_MODE));
    ESP_ERROR_CHECK(ina219_calibrate(&ina_handle, R_SHUNT, MAX_INA_AMP));

    // I2C Add RTC Device:
    ESP_ERROR_CHECK(ds3231_init(&rtc_handle, master_bus_handle, DS3231_ADDR, I2C_MODE));


    // SPI MASTER SET:
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));


    // SPI ADD SD READER DEVICE:
    sdmmc_host_t host_cfg = SDSPI_HOST_DEFAULT();
    host_cfg.slot = SPI3_HOST;
    host_cfg.max_freq_khz = SPI_FREQ;

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = CS;
    slot_cfg.host_id = host_cfg.slot;
    
    esp_vfs_fat_mount_config_t mount_cfg = {
        .max_files = MAX_FILES,
        .format_if_mount_failed = true,
        .allocation_unit_size = ALLOC_UNIT_SIZE
    };
    ESP_ERROR_CHECK(esp_vfs_fat_sdspi_mount(mount_path, &host_cfg, &slot_cfg, &mount_cfg, &sd_card));
    sdmmc_card_print_info(stdout, sd_card);
    

    // FILE SET:
    f = fopen("/sdcard/voltlog.csv", "w");                             
    if (f == NULL) {
        ESP_LOGE(TAG, "File open on WRITE failed");
        return;
    }

    fprintf(f, "Time[S],Voltage[V]\n");                          // Log file headers 
    fclose(f);
    f = NULL;


    // ADC SET:
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12));
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));


    // DAC SET: 
    ESP_ERROR_CHECK(dac_output_enable(DAC_CHAN_0));


    // QUEUE SET:
    // oled_q set:
    oled_q = xQueueCreate(1, sizeof(float));
    if (oled_q == NULL) {
        ESP_LOGE(TAG, "oled_q creation failed");
        return;
    }

    // file_q set:
    file_q = xQueueCreate(1, sizeof(struct READ_DATA));
    if (file_q == NULL) {
        ESP_LOGE(TAG, "file_q creation failed");
        return;
    }

    
    // SEMAPHORE SET:
    read_semaphore = xSemaphoreCreateBinary();
    if (read_semaphore == NULL) {
        ESP_LOGE(TAG, "Read semaphore creation failed");
        return;
    }


    // TIMER SET:
    read_timer_handle = xTimerCreate("read timer", pdMS_TO_TICKS(READ_PERIOD), pdTRUE, (void*) 0, ina_read_timer);
    if (read_timer_handle == NULL) {
        ESP_LOGE(TAG, "Timer creation failed");
        return;
    }


    // TASKS SET:
    // Btn task:
    if (xTaskCreate(btn_task, "btn task", BTN_STACK_DEPTH, NULL, 5, &btn_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Btn task creation failed");
        return;
    }

    // Read task:
    if (xTaskCreate(read_task, "read task", READ_STACK_DEPTH, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Read task creation failed");
        return;
    }

    // Change volt task:
    if (xTaskCreate(change_volt_task, "change volt task", CHANGE_VOLT_STACK_DEPTH, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Change volt task creation failed");
        return;
    }

    // File task:
    if (xTaskCreate(file_task, "file task", FILE_STACK_DEPTH, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "File task creation failed");
        return;
    }

    // OLED task:
    if (xTaskCreate(oled_task, "OLED task", OLED_STACK_DEPTH, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "OLED task creation failed");
        return;
    }


    // INTERRUPT SET:
    ESP_ERROR_CHECK(gpio_set_intr_type(BTN_PIN, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_PIN, btn_isr, NULL));
    ESP_ERROR_CHECK(gpio_intr_enable(BTN_PIN));
}

// TASKS DEFINITION:
/* Turn system ON & OFF on button press */
void btn_task(void* pvParameter) {
    
    bool system_on = false;                                                             // System status flag
    bool first_intr_flag = true;                                                        // Indicate the first intr 

    while (1) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
            if (gpio_get_level(BTN_PIN)==0) {                                           // Check if pressed
                system_on = !system_on;
                change_system_status(system_on, &first_intr_flag);
                while (gpio_get_level(BTN_PIN) == LOW) vTaskDelay(pdMS_TO_TICKS(5));    // Wait while pressed
                vTaskDelay(pdMS_TO_TICKS(10));          
                while (ulTaskNotifyTake(pdTRUE, 0));                                    // Clear notifications
            }
            //printf("Btn stack left: %d\n", (int)uxTaskGetStackHighWaterMark(NULL));   // Check stack
        }
    }
}

/* 
Read the voltage value of the resistor,
and the time from DS3231. Then forward the information.
*/
void read_task(void* pvParameter) {
    struct READ_DATA data;

    while (1) {
        if (xSemaphoreTake(read_semaphore, portMAX_DELAY) == pdTRUE) {
            ESP_ERROR_CHECK(ds3231_get_time(&rtc_handle, &(data.time)));                // Get time from RTC
            ESP_ERROR_CHECK(ina219_read_bus_voltage(&ina_handle, &(data.v)));           // Get voltage
            if (xQueueOverwrite(file_q, &data) != pdPASS) {                             // Send data to file_q
                ESP_LOGE(TAG, "Data failed to pass to file_q");
            }       
            if (xQueueOverwrite(oled_q, &(data.v)) != pdPASS) {                         // Send data to oled_q
                ESP_LOGE(TAG, "Data failed to pass to oled_q");
            }
            //printf("Read stack left: %d\n", (int)uxTaskGetStackHighWaterMark(NULL));  // Check stack
        }
    }
}

/* Read potentiometer value, and change the resistor's voltage */
void change_volt_task(void* pvParameter) {
    int pot_val = 0;
    int last_pot_val = 0;

    while (1) {
        pot_val = adc1_get_raw(ADC1_CHANNEL_0) * DAC_RANGE / ADC_WIDTH;                     // Get potentiometer value
        if ((pot_val + POT_DIFF) < last_pot_val || (pot_val - POT_DIFF) > last_pot_val) {   // If the potentiometer changed (according to DIFF), update DAC
            ESP_ERROR_CHECK(dac_output_voltage(DAC_CHAN_0, pot_val));
            last_pot_val = pot_val;
        }
        //printf("Change volt stack left: %d\n", (int)uxTaskGetStackHighWaterMark(NULL));   // Check stack
        vTaskDelay(pdMS_TO_TICKS(READ_PERIOD / 2));                                         // Delay shorter than timer to update DAC in no more then one timer delay
    }
}

/* 
Get voltage value and time values, and write it to the SD card.
Data read will be printed after data were read 20 times.
In case of system turning OFF at the middle of fprintf,
data loss will occur.
*/
void file_task(void* pvParameter) {

    struct READ_DATA data;
    struct READ_DATA data_arr[20];                                                      // Data will be stored here
    int idx = 0;                                                                        // data_arr index

    while (1) {
        if (xQueueReceive(file_q, &data, portMAX_DELAY) == pdTRUE) {
            if (idx >= 20) {                                                            // Print data only if idx == 20
                f = fopen("/sdcard/voltlog.csv", "a");
                if (f == NULL) {
                    ESP_LOGE(TAG, "File open on APPEND failed");
                    continue;
                }
                for (idx = 0; idx < 20; idx++) {
                    fprintf(f, "%02d:%02d:%02d,%.3f\n", data_arr[idx].time.tm_hour, 
                        data_arr[idx].time.tm_min , data_arr[idx].time.tm_sec, data_arr[idx].v);
                }
                idx = 0;
                fclose(f);
                f = NULL;
            }
            data_arr[idx++] = data;                                                     // Add data to the array
            //printf("File stack left: %d\n", (int)uxTaskGetStackHighWaterMark(NULL));  // Check stack
        }
    }
}

/* Show the graph of the latest voltage values on the SSD1306 */
void oled_task(void* pvParameter) {

    float volt_arr[OLED_WIDTH] = { 0 };
    int idx = 0;

    while (1) {
        if (xQueueReceive(oled_q, &(volt_arr[idx++]), portMAX_DELAY) == pdTRUE) {
            ESP_ERROR_CHECK(ssd1306_clear_display(display_handle, false));
            if (idx == OLED_WIDTH) idx = 0;                                                                     // Reset cycle
            for (int i = 0; i < OLED_WIDTH; i++) {
                ESP_ERROR_CHECK(ssd1306_set_pixel(display_handle, (uint8_t)i, 
                    (uint8_t)(OLED_HEIGHT - 1 - (uint8_t)(volt_arr[idx] * OLED_HEIGHT_MAP)), false));
                idx++;
                if (idx == OLED_WIDTH) idx = 0;                                                                 // Reset cycle
            }   
            ESP_ERROR_CHECK(ssd1306_display_pages(display_handle));
            //printf("OLED stack left: %d\n", (int)uxTaskGetStackHighWaterMark(NULL));                          // Check stack
        }
    }
}


// HELPFUL FUNCTIONS DEFINITION:
/* 
Get system status and turn it ON/OFF accordingly.
System ON - LED ON, start timer, display loading and SPI reset.
System OFF - LED OFF, stop timer, unmount, SPI bus free and clear display.
First turn ON will not set SPI.
*/
void change_system_status(bool system_status, bool *first_intr) {
    
    // Turn ON:
    if (system_status == true) {
        ESP_ERROR_CHECK(gpio_set_level(LED_PIN, HIGH));                     

        ESP_ERROR_CHECK(ssd1306_display_text(display_handle, 1, "Loading...", false));
        vTaskDelay(pdMS_TO_TICKS(500));
        
        if (*first_intr == true) *first_intr = false;
        else spi_set();

        if (xTimerStart(read_timer_handle, 0) != pdPASS) {
            ESP_LOGE(TAG, "Timer start failed");
            while (1);
        }
    }

    // Turn OFF:
    else {
        ESP_ERROR_CHECK(gpio_set_level(LED_PIN, LOW));

        if (xTimerStop(read_timer_handle, 0) != pdPASS) {
            ESP_LOGE(TAG, "Timer stop failed");
            while (1);
        }

        // SPI & File closing:
        if (f != NULL) {
            fclose(f);
            f = NULL;
        }
        ESP_ERROR_CHECK(esp_vfs_fat_sdcard_unmount(mount_path, sd_card));
        ESP_ERROR_CHECK(spi_bus_free(SPI3_HOST));
        
        xQueueReset(oled_q);                                                        // Reset oled_q to prevent display glitch
        ESP_ERROR_CHECK(ssd1306_clear_display(display_handle, false));    
    }
}

/* Set SPI and add SD_card */
void spi_set() {
    // SPI MASTER SET:
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));


    // SPI ADD SD READER DEVICE:
    sdmmc_host_t host_cfg = SDSPI_HOST_DEFAULT();
    host_cfg.slot = SPI3_HOST;
    host_cfg.max_freq_khz = SPI_FREQ;

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = CS;
    slot_cfg.host_id = host_cfg.slot;
    
    esp_vfs_fat_mount_config_t mount_cfg = {
        .max_files = MAX_FILES,
        .format_if_mount_failed = true,
        .allocation_unit_size = ALLOC_UNIT_SIZE
    };
    ESP_ERROR_CHECK(esp_vfs_fat_sdspi_mount(mount_path, &host_cfg, &slot_cfg, &mount_cfg, &sd_card));
}
