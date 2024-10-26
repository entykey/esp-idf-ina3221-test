
#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ina3221.h>
#include <string.h>
#include <esp_log.h>

#define TAG "INA3221-MAIN"

// Structure to hold I2C pins
typedef struct {
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
} i2c_pins_t;

#define I2C_PORT 0
#define WARNING_CHANNEL 1
#define WARNING_CURRENT (40.0)  

#define SHUNT_RESISTOR_MILLI_OHM    100
#define I2C_ADDR                    0x40
#define MEASURING_MODE_TRIGGER      1   



// #define STRUCT_SETTING 0    
#if defined MEASURING_MODE_TRIGGER
#define MODE false  // true : continuous  measurements // false : trigger measurements
#else
#define MODE true
#endif

// #define MODE true  // true : continuous  measurements 


// Tuanhayho: added retry after reading failure due to fast rate
#define POLLING_DELAY_MS 18
#define MAX_RETRIES 3          // Number of retries for I2C operations
#define MEASURE_TIMEOUT_MS 800 // Maximum wait time for the measurement to complete
#define RETRY_COUNT 5          // Maximum retries for I2C operations


static esp_err_t i2c_dev_read_with_retry(ina3221_t *dev, esp_err_t (*ina3221_fn)(ina3221_t *), const char *error_msg)
{
    esp_err_t res;
    int retry_count = 0;

    do {
        res = ina3221_fn(dev);
        if (res == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "%s. Retrying... (%d/%d)", error_msg, retry_count + 1, MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(50));  // Wait before retrying
        retry_count++;
    } while (retry_count < MAX_RETRIES);

    ESP_LOGE(TAG, "%s failed after %d retries", error_msg, MAX_RETRIES);
    return res;
}

void task(void *pvParameters)
{
    // Check if pvParameters is NULL
    if (pvParameters == NULL) {
        ESP_LOGE(TAG, "Task received NULL as pvParameters!");
        vTaskDelete(NULL);
        return;
    }

    i2c_pins_t *i2c_pins = (i2c_pins_t *)pvParameters;

    ESP_LOGI(TAG, "Task received valid i2c_pins: SDA = %d, SCL = %d", i2c_pins->sda_pin, i2c_pins->scl_pin);

    ina3221_t dev = {
            /* shunt values are 100 mOhm for each channel */
            .shunt = {
                SHUNT_RESISTOR_MILLI_OHM,
                SHUNT_RESISTOR_MILLI_OHM,
                SHUNT_RESISTOR_MILLI_OHM
            },
            .config.config_register = INA3221_DEFAULT_CONFIG,
            .mask.mask_register = INA3221_DEFAULT_MASK
    };
    memset(&dev.i2c_dev, 0, sizeof(i2c_dev_t));

    // ESP_ERROR_CHECK(i2cdev_init());  // cause reboot due to unhandled exception
    esp_err_t res = i2cdev_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "i2cdev_init failed: %s", esp_err_to_name(res));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "i2cdev_init success!");
    ESP_LOGI(TAG, "Initializing I2C with pins SDA = %d, SCL = %d", i2c_pins->sda_pin, i2c_pins->scl_pin);
    
    //  ###  INA3221 initialization and configuration
    //ESP_ERROR_CHECK(ina3221_init_desc(&dev, CONFIG_EXAMPLE_I2C_ADDR, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    res = ina3221_init_desc(&dev, I2C_ADDR, I2C_PORT, i2c_pins->sda_pin, i2c_pins->scl_pin);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "ina3221_init_desc failed: %s", esp_err_to_name(res));
        vTaskDelete(NULL);
        return;
    }

#ifndef STRUCT_SETTING
    printf("STRUCT_SETTING defined\n");
    res = ina3221_set_options(&dev, MODE, true, true);  // Mode selection, bus and shunt activated
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set INA3221 options: %s", esp_err_to_name(res));
        vTaskDelete(NULL);
        return;
    }
    res = ina3221_enable_channel(&dev, true, true, true);   // Enable all channels
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable all INA3221 channels: %s", esp_err_to_name(res));
        vTaskDelete(NULL);  
        return;
    }
    ESP_ERROR_CHECK(ina3221_set_average(&dev, INA3221_AVG_16));      // 16 samples average
    // ESP_ERROR_CHECK(ina3221_set_bus_conversion_time(&dev, INA3221_CT_2116));   // 2ms by channel
    // ESP_ERROR_CHECK(ina3221_set_shunt_conversion_time(&dev, INA3221_CT_2116)); // 2ms by channel
    ESP_ERROR_CHECK(ina3221_set_bus_conversion_time(&dev, INA3221_CT_140)); // Faster conversion (140µs per channel)
    ESP_ERROR_CHECK(ina3221_set_shunt_conversion_time(&dev, INA3221_CT_140)); // Faster conversion (140µs per channel)

#else
    printf("STRUCT_SETTING not defined\n");
    dev.config.mode = MODE; // mode selection
    dev.config.esht = true; // shunt enable
    dev.config.ebus = true; // bus enable
    dev.config.ch1 = true;  // channel 1 enable
    dev.config.ch2 = true;  // channel 2 enable
    dev.config.ch3 = true;  // channel 3 enable
    dev.config.avg = INA3221_AVG_64;   // 64 samples average
    dev.config.vbus = INA3221_CT_2116; // 2ms by channel (bus)
    dev.config.vsht = INA3221_CT_2116; // 2ms by channel (shunt)
    ESP_ERROR_CHECK(ina3221_sync(&dev));
#endif

    ESP_ERROR_CHECK(ina3221_set_warning_alert(&dev, WARNING_CHANNEL - 1, WARNING_CURRENT)); // Set overcurrent security flag

    uint32_t measure_number = 0;
    bool warning = false;
    float bus_voltage;
    float shunt_voltage;
    float shunt_current;

    while (1)
    {
        measure_number++;

#if MEASURING_MODE_TRIGGER
        // printf("in trigger mode");
        // Start a measurement in trigger mode with retries
        ESP_ERROR_CHECK(i2c_dev_read_with_retry(&dev, ina3221_trigger, "Failed to trigger measurement"));

        // printf("trig done, wait: ");
        uint32_t time_waited_ms = 0;
        do
        {
            // printf("X");

            // Check INA3221 status with retries
            ESP_ERROR_CHECK(i2c_dev_read_with_retry(&dev, ina3221_get_status, "Failed to get INA3221 status"));

            if (dev.mask.wf & (1 << (3 - WARNING_CHANNEL)))
                warning = true;

            vTaskDelay(pdMS_TO_TICKS(POLLING_DELAY_MS));
            time_waited_ms += POLLING_DELAY_MS;

            // Timeout check to prevent infinite wait
            if (time_waited_ms >= MEASURE_TIMEOUT_MS)
            {
                ESP_LOGE(TAG, "Measurement timed out after %d ms", MEASURE_TIMEOUT_MS);
                break;
            }

        } while (!(dev.mask.cvrf)); // Wait until measurement is done
#else
        printf("In continuous measuring mode.");
        // ESP_ERROR_CHECK(ina3221_get_status(&dev)); // get mask
        // In continuous mode, just get the status with retries
        ESP_ERROR_CHECK(i2c_dev_read_with_retry(&dev, ina3221_get_status, "Failed to get INA3221 status"));

        if (dev.mask.wf & (1 << (3 - WARNING_CHANNEL)))
            warning = true;
#endif
        // for (uint8_t i = 0; i < INA3221_BUS_NUMBER; i++)
        // for (uint8_t i = 0; i < 1; i++)
        // {
        //     // Get voltage in volts
        //     ESP_ERROR_CHECK(ina3221_get_bus_voltage(&dev, i, &bus_voltage));
        //     // Get voltage in millivolts and current in milliamperes
        //     ESP_ERROR_CHECK(ina3221_get_shunt_value(&dev, i, &shunt_voltage, &shunt_current));

        //     printf("\nC%u:Measure number %" PRIu32 "\n", i + 1, measure_number);
        //     if (warning && (i + 1) == WARNING_CHANNEL)
        //         printf("C%u:Warning Current > %.2f mA !!\n", i + 1, WARNING_CURRENT);
        //     printf("C%u:Bus voltage: %.02f V\n", i + 1, bus_voltage);
        //     printf("C%u:Shunt voltage: %.02f mV\n", i + 1, shunt_voltage);
        //     printf("C%u:Shunt current: %.02f mA\n\n", i + 1, shunt_current);

        // }
        for (uint8_t i = 0; i < INA3221_BUS_NUMBER; i++) {
            bool success = false;
            int retry_count = 0;

            // Retry loop for bus voltage reading
            while (retry_count < RETRY_COUNT) {
                res = ina3221_get_bus_voltage(&dev, i, &bus_voltage);
                if (res == ESP_OK) {
                    success = true;
                    break;
                }
                ESP_LOGW(TAG, "Retry %d/%d: Could not read bus voltage from device [0x%02x]: %s", retry_count + 1, RETRY_COUNT, I2C_ADDR, esp_err_to_name(res));
                retry_count++;
                vTaskDelay(pdMS_TO_TICKS(POLLING_DELAY_MS));
            }

            if (!success) {
                ESP_LOGE(TAG, "Failed to read bus voltage from INA3221 after %d retries. Skipping...", RETRY_COUNT);
                continue;
            }

            // Retry loop for shunt voltage and current reading
            success = false;
            retry_count = 0;

            while (retry_count < RETRY_COUNT) {
                res = ina3221_get_shunt_value(&dev, i, &shunt_voltage, &shunt_current);
                if (res == ESP_OK) {
                    success = true;
                    break;
                }
                ESP_LOGW(TAG, "Retry %d/%d: Could not read shunt voltage/current from device [0x%02x]: %s", retry_count + 1, RETRY_COUNT, I2C_ADDR, esp_err_to_name(res));
                retry_count++;
                vTaskDelay(pdMS_TO_TICKS(POLLING_DELAY_MS));
            }

            if (!success) {
                ESP_LOGE(TAG, "Failed to read shunt voltage/current from INA3221 after %d retries. Skipping...", retry_count);
                continue;
            }

            // printf("\nC%u: Measure number %" PRIu32 "\n", i + 1, measure_number);
            if (warning && (i + 1) == WARNING_CHANNEL) {
                printf("C%u: Warning! Current exceeds %.2f mA!!\n", i + 1, WARNING_CURRENT);
            }
            printf("C%u: Bus voltage: %.02f V\n", i + 1, bus_voltage);
            printf("C%u: Shunt voltage: %.02f mV\n", i + 1, shunt_voltage);
            printf("C%u: Shunt current: %.02f mA\n\n", i + 1, shunt_current);
        }
        warning = false ;

        vTaskDelay(pdMS_TO_TICKS(300)); // 10ms capable, no problem
    }
}

void app_main()
{
    i2c_pins_t i2c_pins;  // Declare this outside the conditional blocks to keep it in scope

#ifdef CONFIG_IDF_TARGET_ESP32
    ESP_LOGI(TAG, "ESP32 detected. Initializing I2C INA3221");

    // Set I2C SDA and SCL pins specifically for ESP32
    i2c_pins.sda_pin = 21;
    i2c_pins.scl_pin = 22;

#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP_LOGI(TAG, "ESP32 S3 detected. Initializing I2C INA3221");

    // Set I2C SDA and SCL pins specifically for ESP32-S2
    i2c_pins.sda_pin = 20;
    i2c_pins.scl_pin = 21;

#else
    ESP_LOGW(TAG, "Unsupported ESP32 Target, using defaults I2C pins...");

    // Set some default or fallback I2C pins if target is unknown
    i2c_pins.sda_pin = 4;   // Default SDA pin
    i2c_pins.scl_pin = 5;    // Default SCL pin
#endif

    vTaskDelay(pdMS_TO_TICKS(100));

    xTaskCreate(task, "ina3221_test", configMINIMAL_STACK_SIZE * 8, (void*)&i2c_pins, 5, NULL);
}