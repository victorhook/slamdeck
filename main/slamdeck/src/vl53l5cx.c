#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "vl53l5cx.h"
#include "i2c.h"
#include "platform.h"

#define LOW  0
#define HIGH 1
#define IS_RANGING     1
#define IS_NOT_RANGING 0

#define VL53L5CX_DEFAULT_I2C_ADDRESS_7_BIT 0x29
#define VL53L5CX_MAX_DISTANCE_MM 4000

#define get_current_time() ((xTaskGetTickCount() / portTICK_PERIOD_MS) * 100)

#define DO_DEBUG

/*
static const VL53L5CX_settings_t sensor_settings_default = {
    .integration_time_ms = 5,
    .sharpener_percent = 5,
    .ranging_frequency_hz = 15,
    .resolution = VL53L5CX_RESOLUTION_8X8,
    .power_mode = VL53L5CX_POWER_MODE_WAKEUP,
    .target_order = VL53L5CX_TARGET_ORDER_STRONGEST,
    .ranging_mode = VL53L5CX_RANGING_MODE_AUTONOMOUS
};
*/


static const char* TAG = "VL53L5CX";


static inline void set_sensor_enable(gpio_num_t pin, const int value)
{
    ESP_ERROR_CHECK(gpio_set_level(pin, value));
}

static inline void init_sensor_gpio_enable_pin(VL53L5CX_t* sensor)
{
    ESP_LOGD(TAG, "Init GPIO: %d", sensor->enable_pin);
	ESP_ERROR_CHECK(gpio_reset_pin(sensor->enable_pin));
	ESP_ERROR_CHECK(gpio_set_direction(sensor->enable_pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(sensor->enable_pin, LOW));
}

static void inline print_sensor(const VL53L5CX_t* sensor)
{
    printf("-------------- Sensor %d -----------------\n", (uint8_t) sensor->enable_pin);
    printf("power_mode: %d\n", sensor->settings.power_mode);
    printf("resolution: %d\n", sensor->settings.resolution);
    printf("ranging_frequency_hz: %d\n", sensor->settings.ranging_frequency_hz);
    printf("integration_time_ms: %d\n", sensor->settings.integration_time_ms);
    printf("sharpener_percent: %d\n", sensor->settings.sharpener_percent);
    printf("target_order: %d\n", sensor->settings.target_order);
    printf("ranging_mode: %d\n", sensor->settings.ranging_mode);
    printf("-------------------------------------------\n");
    fflush(stdout);
}

void VL53L5CX_enable(VL53L5CX_t* sensor)
{
    if (!sensor->enable_pin_initialized)
        ESP_LOGW(TAG, "Sensor pin not initialized!");
    set_sensor_enable(sensor->enable_pin, HIGH);
}

void VL53L5CX_disable(VL53L5CX_t* sensor)
{
    if (!sensor->enable_pin_initialized)
        ESP_LOGW(TAG, "Sensor pin not initialized!");
    set_sensor_enable(sensor->enable_pin, LOW);
}

void VL53L5CX_init_gpio(VL53L5CX_t* sensor)
{
    init_sensor_gpio_enable_pin(sensor);
    sensor->enable_pin_initialized = 1;
}

uint8_t VL53L5CX_is_alive(VL53L5CX_t* sensor)
{
    // First ping the i2c address to check if a sensor is available there.
    //  Note: If this is not done, we need to re-initialize the i2c driver between attempts,
    //  like: i2c_driver_delete(0); i2c_init();
    //  Unfortunately, I don't know why this is the case.
    if (i2c_master_ping_address(sensor->config.platform.address) != ESP_OK) {
        return VL53L5CX_STATUS_TIMEOUT_ERROR;;
    }

    ESP_LOGI(TAG, "PING OK");
    uint8_t is_alive;
    vl53l5cx_is_alive(&sensor->config, &is_alive);
    return VL53L5CX_STATUS_OK;
}

VL53L5CX_status_e VL53L5CX_init(VL53L5CX_t* sensor)
{
    if (!sensor->enable_pin_initialized) {
        ESP_LOGW(TAG, "Must call 'VL53L5CX_init_gpio' enable i2c pin first!");
        sensor->status = VL53L5CX_STATUS_INIT_FAIL;
        return sensor->status;
    }
    if (sensor->i2c_address == 0) {
        ESP_LOGW(TAG, "Requested i2c address is 0! Maybe you have forgotten to set it.");
        sensor->status = VL53L5CX_STATUS_INIT_FAIL;
        return sensor->status;
    }

    // Ensure the sensor is enabled first
    VL53L5CX_enable(sensor);

    // Set i2c address to default and try to communicate
    sensor->config.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS_7_BIT;
    if (VL53L5CX_is_alive(sensor) != VL53L5CX_STATUS_OK) {
        ESP_LOGD(TAG, "Sensor %d not found at %02x. Trying again at 0x%02x", sensor->id, sensor->config.platform.address, sensor->i2c_address);

        sensor->config.platform.address = sensor->i2c_address;
        sensor->status = VL53L5CX_is_alive(sensor);
        if (sensor->status != VL53L5CX_STATUS_OK) {
            ESP_LOGW(TAG, "Sensor %d not found at 0x%02x either. Aborting initialization", sensor->id, sensor->config.platform.address);
            return sensor->status;
        }
    }

    ESP_LOGD(TAG, "Sensor %d found at address 0x%02x, initializing", sensor->id, sensor->config.platform.address);
    sensor->status = vl53l5cx_init(&sensor->config);
    if (sensor->status) {
        ESP_LOGW(TAG, "Failed to initialize sensor %d. error: %d", sensor->id, sensor->status);
        return sensor->status;
    }

    sensor->status = VL53L5CX_set_i2c_address(sensor, sensor->i2c_address);
    if (sensor->status) {
        ESP_LOGW(TAG, "Failed to set i2c address 0x%02x for sensor %d error: %d", sensor->i2c_address, sensor->id, sensor->status);
        return sensor->status;
    }

    sensor->status = VL53L5CX_is_alive(sensor);
    if (sensor->status) {
        ESP_LOGW(TAG, "Failed to recognize sensor %d after i2c address change to 0x%02x error: %d", sensor->id, sensor->i2c_address, sensor->status);
        return sensor->status;
    }

    ESP_LOGD(TAG, "Sensor %d initialized, with i2c address 0x%02x", sensor->enable_pin, sensor->i2c_address);

    sensor->status = VL53L5CX_STATUS_OK;
    sensor->is_ranging = IS_NOT_RANGING;

    return sensor->status;
}

uint16_t VL53L5CX_get_data_size(const VL53L5CX_t* sensor)
{
    return sensor->settings.resolution * 2;
}

uint8_t VL53L5CX_data_ready(VL53L5CX_t* sensor)
{
    sensor->status = vl53l5cx_check_data_ready(&sensor->config, &sensor->data_ready);
    #ifdef DO_DEBUG
        if (sensor->status) {
            ESP_LOGD(TAG, "I2C error checking data ready sensor %d: %02x data is ready: %d", sensor->id, sensor->status, sensor->data_ready);
            vTaskDelay(1000/portTICK_PERIOD_MS);
            i2c_reset_tx_fifo(0);
            i2c_reset_rx_fifo(0);
            //VL53L5CX_stop(sensor);
            //VL53L5CX_start(sensor);
        }
    #endif
    return sensor->data_ready;
}

uint8_t VL53L5CX_collect_data(VL53L5CX_t* sensor)
{
    sensor->status = vl53l5cx_get_ranging_data(&sensor->config, &sensor->result);
    #ifdef DO_DEBUG
        if (sensor->status != VL53L5CX_STATUS_OK)
            ESP_LOGD(TAG, "I2C error getting data: %02x", sensor->status);
    #endif
    if (sensor->status == VL53L5CX_STATUS_OK) {
        sensor->last_sample = get_current_time();
        sensor->samples++;

        // Filter data if needed.
        for (int grid = 0; grid < sensor->settings.resolution; grid++) {
            switch (sensor->result.target_status[grid]) {
                case TARGET_STATUS_TARGET_NOT_DETECTED:
                case TARGET_STATUS_RANGE_VALID_BUT_NO_TARGET:
                    sensor->data_distance_mm[grid] = VL53L5CX_MAX_DISTANCE_MM;
                    break;
                case TARGET_STATUS_NOT_UPDATED:
                    // Don't do anything here, TODO make it nicer.
                    break;
                default:
                    sensor->data_distance_mm[grid] = sensor->result.distance_mm[grid];
                    break;
            }

            // If you dont want to filter anything, uncomment this.
            //sensor->data_distance_mm[grid] = sensor->result.distance_mm[grid];
        }
    }
    return sensor->status;
}

const VL53L5CX_ResultsData* VL53L5CX_get_data(VL53L5CX_t* sensor)
{
    return &sensor->result;
}


uint8_t VL53L5CX_start(VL53L5CX_t* sensor)
{
    sensor->status = vl53l5cx_start_ranging(&sensor->config);
    #ifdef DO_DEBUG
        if (sensor->status)
            ESP_LOGD(TAG, "Sensor %d, failed to start ranging: %d", sensor->id, sensor->status);
    #endif
    sensor->is_ranging = sensor->status == VL53L5CX_STATUS_OK ? IS_RANGING : IS_NOT_RANGING;
    return sensor->status;
}

uint8_t VL53L5CX_stop(VL53L5CX_t* sensor)
{
    sensor->status = vl53l5cx_stop_ranging(&sensor->config);
    #ifdef DO_DEBUG
        if (sensor->status)
            ESP_LOGD(TAG, "Sensor %d failed to stop ranging: %d", sensor->id, sensor->status);
    #endif
    sensor->is_ranging = sensor->status == VL53L5CX_STATUS_OK ? IS_NOT_RANGING : IS_RANGING;
    return sensor->status;
}

/* --- Setters --- */
uint8_t VL53L5CX_set_i2c_address(VL53L5CX_t* sensor, const uint8_t i2c_address)
{
    sensor->status = vl53l5cx_set_i2c_address(&sensor->config, i2c_address);
    return sensor->status;
}
uint8_t VL53L5CX_set_power_mode(VL53L5CX_t* sensor, const VL53L5CX_power_mode_e power_mode)
{
    sensor->status = vl53l5cx_set_power_mode(&sensor->config, power_mode);
    return sensor->status;
}
uint8_t VL53L5CX_set_resolution(VL53L5CX_t* sensor, const VL53L5CX_resolution_e resolution)
{
    sensor->status = vl53l5cx_set_resolution(&sensor->config, resolution);
    return sensor->status;
}
uint8_t VL53L5CX_set_ranging_frequency_hz(VL53L5CX_t* sensor, const uint8_t ranging_frequency_hz)
{
    sensor->status = vl53l5cx_set_ranging_frequency_hz(&sensor->config, ranging_frequency_hz);
    return sensor->status;
}
uint8_t VL53L5CX_set_integration_time_ms(VL53L5CX_t* sensor, const uint32_t integration_time_ms)
{
    sensor->status = vl53l5cx_set_integration_time_ms(&sensor->config, integration_time_ms);
    return sensor->status;
}
uint8_t VL53L5CX_set_sharpener_percent(VL53L5CX_t* sensor, const uint8_t sharpener_percent)
{
    sensor->status = vl53l5cx_set_sharpener_percent(&sensor->config, sharpener_percent);
    return sensor->status;
}
uint8_t VL53L5CX_set_target_order(VL53L5CX_t* sensor, const VL53L5CX_target_order_e target_order)
{
    sensor->status = vl53l5cx_set_target_order(&sensor->config, target_order);
    return sensor->status;
}
uint8_t VL53L5CX_set_ranging_mode(VL53L5CX_t* sensor, const VL53L5CX_ranging_mode_e ranging_mode)
{
    sensor->status = vl53l5cx_set_ranging_mode(&sensor->config, ranging_mode);
    return sensor->status;
}

/* --- Getters --- */
uint8_t VL53L5CX_get_i2c_address(VL53L5CX_t* sensor)
{
    return sensor->i2c_address;
}
uint8_t VL53L5CX_get_power_mode(VL53L5CX_t* sensor)
{
    return sensor->settings.power_mode;
}
uint8_t VL53L5CX_get_resolution(VL53L5CX_t* sensor)
{
    return sensor->settings.resolution;
}
uint8_t VL53L5CX_get_ranging_frequency_hz(VL53L5CX_t* sensor)
{
    return sensor->settings.ranging_frequency_hz;
}
uint32_t VL53L5CX_get_integration_time_ms(VL53L5CX_t* sensor)
{
    return sensor->settings.integration_time_ms;
}
uint8_t VL53L5CX_get_sharpener_percent(VL53L5CX_t* sensor)
{
    return sensor->settings.sharpener_percent;
}
uint8_t VL53L5CX_get_target_order(VL53L5CX_t* sensor)
{
    return sensor->settings.target_order;
}
uint8_t VL53L5CX_get_ranging_mode(VL53L5CX_t* sensor)
{
    return sensor->settings.ranging_mode;
}
