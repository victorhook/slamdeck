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

#define get_current_time() ((xTaskGetTickCount() / portTICK_PERIOD_MS) * 100)

#define DO_DEBUG


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
    printf("power_mode: %d\n", sensor->power_mode);
    printf("resolution: %d\n", sensor->resolution);
    printf("ranging_frequency_hz: %d\n", sensor->ranging_frequency_hz);
    printf("integration_time_ms: %d\n", sensor->integration_time_ms);
    printf("sharpener_percent: %d\n", sensor->sharpener_percent);
    printf("target_order: %d\n", sensor->target_order);
    printf("ranging_mode: %d\n", sensor->ranging_mode);
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

#include "i2c.h"

uint8_t VL53L5CX_is_alive(VL53L5CX_t* sensor)
{
    // First ping the i2c address to check if a sensor is available there.
    //  Note: If this is not done, we need to re-initialize the i2c driver between attempts,
    //  like: i2c_driver_delete(0); i2c_init();
    //  Unfortunately, I don't know why this is the case.
    if (i2c_master_ping_address(sensor->config.platform.address) != ESP_OK) {
        return VL53L5CX_STATUS_TIMEOUT_ERROR;;
    }

    uint8_t is_alive;
    vl53l5cx_is_alive(&sensor->config, &is_alive);
    return VL53L5CX_STATUS_OK;
}

VL53L5CX_status_e VL53L5CX_init(VL53L5CX_t* sensor)
{
    if (!sensor->enable_pin_initialized) {
        ESP_LOGW(TAG, "Must call 'VL53L5CX_init_gpio' enable i2c pin first!");
        return SENSOR_STATUS_FAILED;
    }
    if (sensor->i2c_address == 0) {
        ESP_LOGW(TAG, "Requested i2c address is 0! Maybe you have forgotten to set it.");
    }

    // Ensure the sensor is enabled first
    VL53L5CX_enable(sensor);
    uint8_t result = VL53L5CX_STATUS_OK;
    sensor->config.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS_7_BIT;

    if (VL53L5CX_is_alive(sensor) != VL53L5CX_STATUS_OK) {
        ESP_LOGD(TAG, "Sensor %d not found at %02x. Trying again at 0x%02x", sensor->id, sensor->config.platform.address, sensor->i2c_address);

        sensor->config.platform.address = sensor->i2c_address;
        result = VL53L5CX_is_alive(sensor);
        if (result != VL53L5CX_STATUS_OK) {
            ESP_LOGW(TAG, "Sensor %d not found at 0x%02x either. Aborting initialization", sensor->id, sensor->config.platform.address);
            return result;
        }
    }

    ESP_LOGD(TAG, "Sensor %d found at address 0x%02x, initializing", sensor->id, sensor->config.platform.address);

    result = vl53l5cx_init(&sensor->config);
    if (result) {
        ESP_LOGW(TAG, "Failed to initialize sensor %d. error: %d", sensor->id, result);
        return result;
    }

    result = VL53L5CX_set_i2c_address(sensor, sensor->i2c_address);
    if (result) {
        ESP_LOGW(TAG, "Failed to set i2c address 0x%02x for sensor %d error: %d", sensor->i2c_address, sensor->id, result);
        return result;
    }

    result = VL53L5CX_is_alive(sensor);
    if (result) {
        ESP_LOGW(TAG, "Failed to recognize sensor %d after i2c address change to 0x%02x error: %d", sensor->id, sensor->i2c_address, result);
        return result;
    }

    // Set parameters
    //vl53l5cx_set_resolution(&sensor->config, VL53L5CX_RESOLUTION_8X8);
    //vl53l5cx_set_ranging_frequency_hz(&sensor->config, 60);

    ESP_LOGD(TAG, "Sensor %d initialized, with i2c address 0x%02x", sensor->enable_pin, sensor->i2c_address);

    // Get all parameters for the sensor.
    //vl53l5cx_get_power_mode(&sensor->config, (uint8_t*) &sensor->power_mode);
    //vl53l5cx_get_resolution(&sensor->config, (uint8_t*) &sensor->resolution);
    //vl53l5cx_get_ranging_frequency_hz(&sensor->config, &sensor->ranging_frequency_hz);
    //vl53l5cx_get_integration_time_ms(&sensor->config, &sensor->integration_time_ms);
    //vl53l5cx_get_sharpener_percent(&sensor->config, &sensor->sharpener_percent);
    //vl53l5cx_get_target_order(&sensor->config, (uint8_t*) &sensor->target_order);
    //vl53l5cx_get_ranging_mode(&sensor->config, (uint8_t*) &sensor->ranging_mode);
    //print_sensor(sensor);

    sensor->status = SENSOR_STATUS_OK;
    sensor->is_ranging = IS_NOT_RANGING;

    return RESULT_OK;
}

uint8_t VL53L5CX_data_ready(VL53L5CX_t* sensor)
{
    uint8_t res = vl53l5cx_check_data_ready(&sensor->config, &sensor->data_ready);
    #ifdef DO_DEBUG
        if (res)
            ESP_LOGD(TAG, "I2C error checking data ready sensor %d: %02x", sensor->id, res);
    #endif
    return sensor->data_ready;
}

uint8_t VL53L5CX_collect_data(VL53L5CX_t* sensor)
{
    uint8_t res = vl53l5cx_get_ranging_data(&sensor->config, &sensor->result);
    #ifdef DO_DEBUG
        if (res)
            ESP_LOGD(TAG, "I2C error getting data: %02x", res);
    #endif
    if (res == VL53L5CX_STATUS_OK) {
        sensor->last_sample = get_current_time();
        sensor->samples++;
    }
    return res;
}

const VL53L5CX_ResultsData* VL53L5CX_get_data(VL53L5CX_t* sensor)
{
    return &sensor->result;
}


uint8_t VL53L5CX_start(VL53L5CX_t* sensor)
{
    uint8_t res = vl53l5cx_start_ranging(&sensor->config);
    #ifdef DO_DEBUG
        if (res)
            ESP_LOGD(TAG, "Failed to start ranging with sensor %d", (uint8_t) sensor);
    #endif
    sensor->is_ranging = res == VL53L5CX_STATUS_OK ? IS_RANGING : IS_NOT_RANGING;
    return res;
}

uint8_t VL53L5CX_stop(VL53L5CX_t* sensor)
{
    uint8_t res = vl53l5cx_stop_ranging(&sensor->config);
    #ifdef DO_DEBUG
        if (res)
            ESP_LOGD(TAG, "Failed to stop ranging");
    #endif
    sensor->is_ranging = res == VL53L5CX_STATUS_OK ? IS_NOT_RANGING : IS_RANGING;
    return res;
}


/* --- VL53L5CX API --- */

// Generic setter function that calls the low level vl53l5cx driver setter, which
// tries to set the given value. If this return successfully, we also update the sensor struct model.
#define VL53L5CX_set_generic(sensor, member_ptr, arg, setter) \
    uint8_t require_restart = sensor->is_ranging; \
    if (require_restart) \
        VL53L5CX_stop(sensor); \
    uint8_t res = setter(&sensor->config, arg); \
    if (res == VL53L5CX_STATUS_OK) \
        *member_ptr = arg; \
    if (require_restart) \
        VL53L5CX_start(sensor); \
    return res

uint8_t VL53L5CX_set_i2c_address(VL53L5CX_t* sensor, const uint8_t address)
{
    return vl53l5cx_set_i2c_address(&sensor->config, address);
}
uint8_t VL53L5CX_set_power_mode(VL53L5CX_t* sensor, const VL53L5CX_power_mode_e power_mode)
{
    VL53L5CX_set_generic(sensor, &sensor->power_mode, power_mode, vl53l5cx_set_power_mode);
}

typedef enum {
    UINT8,
    UINT32
} VL53L5CX_setter_type_e;

typedef union {
    uint8_t*  u8;
    uint32_t* u32;
} VL53L5CX_setter_arg_t;

#define VL53L5CX_set_generic2(sensor, setter_type, setter, member, arg) \
    uint8_t require_restart = sensor->is_ranging; \
    uint8_t result; \
    if (require_restart) { \
        result = VL53L5CX_stop(sensor); \
        if (result != VL53L5CX_STATUS_OK) \
            return result; \
    } \
    switch (setter_type) { \
        case UINT8: \
            result = setter(&sensor->config, (uint8_t) arg); \
            if (result == VL53L5CX_STATUS_OK) \
                *((uint8_t*) member) = arg; \
            break; \
        case UINT32: \
            result = setter(&sensor->config, (uint32_t) arg); \
            if (result == VL53L5CX_STATUS_OK) \
                *((uint32_t*) member) = arg; \
            break; \
    } \
 \
    if (result != VL53L5CX_STATUS_OK) {  \
        ESP_LOGW(TAG, "Failed to set config, error: %d", result); \
        return result; \
    } \
 \
    if (require_restart) \
        result = VL53L5CX_start(sensor); \
 \
    return result;

uint8_t VL53L5CX_set_resolution(VL53L5CX_t* sensor, const VL53L5CX_resolution_e resolution)
{
    VL53L5CX_set_generic2(sensor, UINT8, vl53l5cx_set_resolution, &sensor->resolution, resolution);
    /*
    uint8_t require_restart = sensor->is_ranging;
    uint8_t result;
    if (require_restart)
        result = VL53L5CX_stop(sensor);
        if (result != VL53L5CX_STATUS_OK)
            return VL53L5CX_STATUS_ERROR;

    result = vl53l5cx_set_resolution(&sensor->config, resolution);
    if (result == VL53L5CX_STATUS_OK)
        sensor->resolution = resolution;

    if (require_restart)
        result = VL53L5CX_start(sensor);

    return result;
    //VL53L5CX_set_generic(sensor, &sensor->resolution, resolution, vl53l5cx_set_resolution);
    */
}
uint8_t VL53L5CX_set_ranging_frequency_hz(VL53L5CX_t* sensor, const uint8_t ranging_frequency_hz)
{
    VL53L5CX_set_generic2(sensor, UINT8, vl53l5cx_set_ranging_frequency_hz, &sensor->ranging_frequency_hz, ranging_frequency_hz);
    //VL53L5CX_set_generic(sensor, &sensor->ranging_frequency_hz, ranging_frequency_hz, vl53l5cx_set_ranging_frequency_hz);
}
uint8_t VL53L5CX_set_integration_time_ms(VL53L5CX_t* sensor, const uint32_t integration_time_ms)
{
    VL53L5CX_set_generic(sensor, &sensor->integration_time_ms, integration_time_ms, vl53l5cx_set_integration_time_ms);
}
uint8_t VL53L5CX_set_sharpener_percent(VL53L5CX_t* sensor, const uint8_t sharpener_percent)
{
    VL53L5CX_set_generic(sensor, &sensor->sharpener_percent, sharpener_percent, vl53l5cx_set_sharpener_percent);
}
uint8_t VL53L5CX_set_target_order(VL53L5CX_t* sensor, const VL53L5CX_target_order_e target_order)
{
    VL53L5CX_set_generic(sensor, &sensor->target_order, target_order, vl53l5cx_set_target_order);
}
uint8_t VL53L5CX_set_ranging_mode(VL53L5CX_t* sensor, const VL53L5CX_ranging_mode_e ranging_mode)
{
    VL53L5CX_set_generic(sensor, &sensor->ranging_mode, ranging_mode, vl53l5cx_set_ranging_mode);
}
