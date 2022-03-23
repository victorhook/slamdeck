#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "slamdeck.h"
#include "vl53l5cx.h"
#include "vl53l5cx_api.h"

#define LOW  0
#define HIGH 1


typedef struct {
    VL53L5CX_Configuration device;
    VL53L5CX_ResultsData   result;
    gpio_num_t             enable_pin;
} vl53l5cx_t;

vl53l5cx_t sensors[] = {
    {
        .enable_pin=SLAMDECK_GPIO_SENSOR_1
    },
    {
        .enable_pin=SLAMDECK_GPIO_SENSOR_2
    },
    {
        .enable_pin=SLAMDECK_GPIO_SENSOR_3
    },
    {
        .enable_pin=SLAMDECK_GPIO_SENSOR_4
    },
    {
        .enable_pin=SLAMDECK_GPIO_SENSOR_5
    }
};

static const total_sensors = sizeof(sensors) / sizeof(vl53l5cx_t);


static void vl53l5cx_task()
{

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
    for (size_t i = 0; i < total_sensors; i++) {
        // 1. Init enable pins (LPn)
        init_sensor_enable_pin(sensors[i].enable_pin);
        disable_all_sensors();

        // Initialize I2C.
        
    }

}

static void set_i2c_address(vl53l5cx_t& sensor, const uint8_t new_address)
{
    vl53l5cx_set_i2c_address(&sensor.Dev, new_address);
}

void init_vl53l5cx()
{
    xTaskCreate(vl53l5cx_task, "VL53L5CX", 2048, NULL, 5, NULL);
}
