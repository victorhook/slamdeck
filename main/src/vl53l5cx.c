#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "vl53l5cx.h"
#include "platform.h"


#define LOW  0
#define HIGH 1

#define VL53L5CX_DEFAULT_I2C_ADDRESS 0x29
#define update_status(sensor, status_bit, value) ( sensor->status |= ((1 << status_bit) & value ? 0xff : 0) )
#define clear_status(sensor) (sensor->status = 0)
#define read_status(sensor, status_bit) ( sensor->status & (1 << status_bit) )

#define DO_DEBUG
#ifdef DO_DEBUG
#define return_res(res) \
if (res) \
    ESP_LOGE(TAG, "Error: %s", esp_err_to_name(res)); \
    return res
#else
#define return_res(res) \
    return res
#endif



static const char* TAG = "VL53L5CX";

typedef struct {
    VL53L5CX_id_e           id;
    gpio_num_t              enable_pin;
    uint8_t                 data_ready;
    uint32_t                integration_time_ms;
    uint8_t                 sharpener_percent;
    uint8_t                 ranging_frequency_hz;
    VL53L5CX_status_e       status;
    VL53L5CX_resolution_e   resolution;
    VL53L5CX_power_mode_e   power_mode;
    VL53L5CX_target_order_e target_order;
    VL53L5CX_ranging_mode_e ranging_mode;

    // ST's driver types
    VL53L5CX_Configuration  config;
    VL53L5CX_ResultsData    result;
} VL53L5CX_t;

static VL53L5CX_t sensors[5];
// Sensor id starts at 1, so we need this to index correctly.

static inline VL53L5CX_t* get_sensor(VL53L5CX_id_e sensor)
{
    return &sensors[( (uint8_t) sensor-1 )];
}

// Bit mask to for temporarily disabling sensors.
static uint8_t tmp_disabled_sensors;


static void VL53L5CX_task()
{

}

static inline void set_sensor_pin(gpio_num_t pin, const int value)
{
    ESP_LOGD(TAG, "Setting pin %d to %d", pin, value);
    ESP_ERROR_CHECK(gpio_set_level(pin, value));
}

static inline void init_sensor_gpio_enable_pin(VL53L5CX_id_e sensor)
{
    gpio_num_t pin = get_sensor(sensor)->enable_pin;
	ESP_ERROR_CHECK(gpio_reset_pin(pin));
	ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin, LOW));
}

static inline void set_sensor_enable(VL53L5CX_id_e sensor, const uint8_t value)
{
    set_sensor_pin(get_sensor(sensor)->enable_pin, value);
    update_status(get_sensor(sensor), SENSOR_STATUS_ENABLED, value);
}

static inline void enable_sensor(VL53L5CX_id_e sensor)
{
    set_sensor_enable(sensor, 1);
}

static inline void disable_sensor(VL53L5CX_id_e sensor)
{
    set_sensor_enable(sensor, 0);
}

static inline void disable_all_sensors()
{
    tmp_disabled_sensors = 0;
    for (VL53L5CX_id_e sensor = 1; sensor <= 5; sensor++) {
        if (read_status(get_sensor(sensor), SENSOR_STATUS_ENABLED))
            tmp_disabled_sensors |= (1 << sensor);
        disable_sensor(sensor);
    }
}

static inline void restore_enabled_sensors()
{
    for (VL53L5CX_id_e sensor = 1; sensor <= 5; sensor++) {
        if (tmp_disabled_sensors |= (1 << sensor))
            enable_sensor(sensor);
    }
}

uint8_t VL53L5CX_init(VL53L5CX_id_e sensor, gpio_num_t enable_pin)
{
    VL53L5CX_t* this_sensor = get_sensor(sensor);
    this_sensor->id = sensor;
    this_sensor->enable_pin = enable_pin;
    this_sensor->config.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;

    // Clear status flag
    clear_status(this_sensor);
    VL53L5CX_result_e result;

    // Enable GPIO for pin
    init_sensor_gpio_enable_pin(sensor);
    ESP_LOGD(TAG, "Initializing, sensor id: %d, enable pin: %d", this_sensor->id, this_sensor->enable_pin);

    // Set i2c address.
    //disable_all_sensors();
    enable_sensor(sensor);
    //VL53L5CX_set_i2c_address(sensor, this_sensor.platform.address);
    //restore_enabled_sensors();

    // 2. Initialize the sensor.
    update_status(this_sensor, SENSOR_STATUS_INITIALIZING, 1);
    result = vl53l5cx_init(&this_sensor->config);
    update_status(this_sensor, SENSOR_STATUS_INITIALIZING, 0);

    if (result != RESULT_OK) {
        ESP_LOGE(TAG, "Initialization failed");
        update_status(this_sensor, SENSOR_STATUS_FAILED, 1);
        return RESULT_FAIL;
    }

    ESP_LOGD(TAG, "Initialization complete");
    update_status(this_sensor, SENSOR_STATUS_INITIALIZED, 1);

    return RESULT_OK;

    // 3. Get all parameters for the sensor.
    vl53l5cx_get_power_mode(&this_sensor->config, (uint8_t*) &this_sensor->power_mode);
    vl53l5cx_get_resolution(&this_sensor->config, (uint8_t*) &this_sensor->resolution);
    vl53l5cx_get_ranging_frequency_hz(&this_sensor->config, &this_sensor->ranging_frequency_hz);
    vl53l5cx_get_integration_time_ms(&this_sensor->config, &this_sensor->integration_time_ms);
    vl53l5cx_get_sharpener_percent(&this_sensor->config, &this_sensor->sharpener_percent);
    vl53l5cx_get_target_order(&this_sensor->config, (uint8_t*) &this_sensor->target_order);
    vl53l5cx_get_ranging_mode(&this_sensor->config, (uint8_t*) &this_sensor->ranging_mode);

    update_status(this_sensor, SENSOR_STATUS_INITIALIZED, 1);
    return RESULT_OK;
}

uint8_t VL53L5CX_start(VL53L5CX_id_e sensor)
{
    uint8_t res = vl53l5cx_start_ranging(&get_sensor(sensor)->config);
    #ifdef DO_DEBUG
        if (res)
            ESP_LOGD(TAG, "Failed to start ranging with sensor %d", (uint8_t) sensor);
    #endif
    return res;
}

uint8_t VL53L5CX_data_ready(VL53L5CX_id_e sensor)
{
    VL53L5CX_t* this_sensor = get_sensor(sensor);
    uint8_t res = vl53l5cx_check_data_ready(&this_sensor->config, &this_sensor->data_ready);
    #ifdef DO_DEBUG
        if (res)
            ESP_LOGD(TAG, "I2C error checking data ready");
    #endif
    return this_sensor->data_ready;
}

uint8_t VL53L5CX_collect_data(VL53L5CX_id_e sensor)
{
    VL53L5CX_t* this_sensor = get_sensor(sensor);
    uint8_t res = vl53l5cx_get_ranging_data(&this_sensor->config, &this_sensor->result);
    #ifdef DO_DEBUG
        if (res)
            ESP_LOGD(TAG, "I2C error getting data");
    #endif
    return res;
}

const VL53L5CX_ResultsData* VL53L5CX_get_data(VL53L5CX_id_e sensor)
{
    VL53L5CX_t* this_sensor = get_sensor(sensor);
    return &this_sensor->result;
}

uint8_t VL53L5CX_stop(VL53L5CX_id_e sensor)
{
    uint8_t res = vl53l5cx_stop_ranging(&get_sensor(sensor)->config);
    #ifdef DO_DEBUG
        if (res)
            ESP_LOGD(TAG, "Failed to stop ranging");
    #endif
    return res;
}


/* --- VL53L5CX API --- */
uint8_t VL53L5CX_set_enable(VL53L5CX_id_e sensor, const uint8_t is_enabled)
{
    set_sensor_pin(get_sensor(sensor)->enable_pin, is_enabled);
    update_status(get_sensor(sensor), SENSOR_STATUS_ENABLED, is_enabled);
    return 0;
}
uint8_t VL53L5CX_set_i2c_address(VL53L5CX_id_e sensor, const uint8_t address)
{
    return vl53l5cx_set_i2c_address(&get_sensor(sensor)->config, address);
}
uint8_t VL53L5CX_set_power_mode(VL53L5CX_id_e sensor, const VL53L5CX_power_mode_e power_mode)
{
    return 0;
}
uint8_t VL53L5CX_set_resolution(VL53L5CX_id_e sensor, const VL53L5CX_resolution_e resolution)
{
    return 0;
}
uint8_t VL53L5CX_set_ranging_frequency_hz(VL53L5CX_id_e sensor, const uint8_t frequency_hz)
{
    return 0;
}
uint8_t VL53L5CX_set_integration_time_ms(VL53L5CX_id_e sensor, const uint32_t integration_time_ms)
{
    return 0;
}
uint8_t VL53L5CX_set_sharpener_percent(VL53L5CX_id_e sensor, const uint8_t sharpener)
{
    return 0;
}
uint8_t VL53L5CX_set_target_order(VL53L5CX_id_e sensor, const VL53L5CX_target_order_e target_order)
{
    return 0;
}
uint8_t VL53L5CX_set_ranging_mode(VL53L5CX_id_e sensor, const VL53L5CX_ranging_mode_e ranging_mode)
{
    return 0;
}
