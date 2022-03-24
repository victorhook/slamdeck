#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "slamdeck.h"
#include "vl53l5cx.h"
#include "vl53l5cx_api.h"

#define LOW  0
#define HIGH 1


typedef struct {
    VL53L5CX_Configuration device;
    VL53L5CX_ResultsData   result;
    gpio_num_t             enable_pin;
    uint8_t                is_active;
    uint8_t                number;
    uint8_t                address;
} vl53l5cx_t;

vl53l5cx_t sensors[] = {
    {
        .enable_pin=SLAMDECK_GPIO_SENSOR_1,
        .is_active=1,
        .number=1
    },
    {
        .enable_pin=SLAMDECK_GPIO_SENSOR_2,
        .number=2
    },
    {
        .enable_pin=SLAMDECK_GPIO_SENSOR_3,
        .number=3
    },
    {
        .enable_pin=SLAMDECK_GPIO_SENSOR_4,
        .number=4
    },
    {
        .enable_pin=SLAMDECK_GPIO_SENSOR_5,
        .number=5
    }
};

static const uint8_t total_sensors = sizeof(sensors) / sizeof(vl53l5cx_t);
static const char* TAG = "VL53L5CX";

static void vl53l5cx_task()
{

}


static inline void set_sensor_pin(gpio_num_t pin, const int value)
{
    gpio_set_level(pin, value);
}

static inline void init_sensor_enable_pin(gpio_num_t sensor)
{
	ESP_ERROR_CHECK(gpio_reset_pin(sensor));
	ESP_ERROR_CHECK(gpio_set_direction(sensor, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(sensor, LOW));
}

static inline void disable_all_sensors()
{
    for (size_t i = 0; i < total_sensors; i++)
        ESP_ERROR_CHECK(gpio_set_direction(sensors[i].enable_pin, LOW));
}


static void init_sensors_gpio()
{
    int error;
    for (size_t i = 0; i < total_sensors; i++) {
        vl53l5cx_t sensor = sensors[i];
        if (!sensor.is_active)
            continue;

        // 1. Init enable pins (LPn)
        init_sensor_enable_pin(sensor.enable_pin);

        // 2. Disable all sensors, except this one and change its i2c address.
        disable_all_sensors();
        gpio_set_level(sensor.enable_pin, HIGH);

        vl53l5cx_set_i2c_address(&sensor.device, sensor.address);

        // Initialize I2C.
        error = vl53l5cx_init(&sensor.device);
        if (error) {
            ESP_LOGI(TAG, "Error initializing sensor %d", sensor.number);
        }
    }

}


void init_vl53l5cx()
{
    xTaskCreate(vl53l5cx_task, "VL53L5CX", 2048, NULL, 5, NULL);
}
