#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stdio.h"
#include "vl53_test.h"
#include "slamdeck.h"
#include "slamdeck_api.h"
#include "vl53l5cx.h"
#include "esp_timer.h"


static VL53L5CX_t sensor = {
    .id=SLAMDECK_SENSOR_ID_BACK,
	.enable_pin=SLAMDECK_GPIO_SENSOR_BACK,
	.i2c_address=0x29
};

static void disable_sensor(const uint8_t sensor)
{
    gpio_reset_pin(sensor);
    gpio_set_direction(sensor, GPIO_MODE_OUTPUT);
    gpio_set_level(sensor, 0);
}

void disable_all()
{
	disable_sensor(SLAMDECK_GPIO_SENSOR_MAIN);
	disable_sensor(SLAMDECK_GPIO_SENSOR_FRONT);
	disable_sensor(SLAMDECK_GPIO_SENSOR_RIGHT);
	disable_sensor(SLAMDECK_GPIO_SENSOR_BACK);
	disable_sensor(SLAMDECK_GPIO_SENSOR_LEFT);
}

void test_vl53l5cx()
{
	disable_all();
	VL53L5CX_init_gpio(&sensor);
	VL53L5CX_init(&sensor);
	VL53L5CX_start(&sensor);

	VL53L5CX_set_resolution(&sensor, VL53L5CX_RESOLUTION_4X4);
	VL53L5CX_set_ranging_frequency_hz(&sensor, 60);
	uint64_t t0 = esp_timer_get_time();
	uint16_t samples = 0;
	uint16_t fails = 0;

	while (1) {
		uint64_t now = esp_timer_get_time();

		if ((now-t0) > 1000000) {
			printf("%d\n", samples);
			t0 = now;
			fails = 0;
			samples = 0;
		}

		if (VL53L5CX_data_ready(&sensor)) {
			VL53L5CX_collect_data(&sensor);
			samples++;
		}

		vTaskDelay(5/portTICK_PERIOD_MS);
	}

}
