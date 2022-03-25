/***********************************/
/*   VL53L5CX ULD basic example    */
/***********************************/
/*
* This example is the most basic. It initializes the VL53L5CX ULD, and starts
* a ranging to capture 10 frames.
*
* By default, ULD is configured to have the following settings :
* - Resolution 4x4
* - Ranging period 1Hz
*
* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

#include "slamdeck.h"
#include "vl53_test.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l5cx_api.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "i2c.h"
#include "esp_task_wdt.h"


static const char* TAG = "VL53L5CX Test";
static VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
static VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */

static void enable_sensor(gpio_num_t gpio)
{
	gpio_reset_pin(gpio);
	gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
	gpio_set_level(gpio, 1);
}

static void test()
{
	enable_sensor(SLAMDECK_GPIO_SENSOR_4);

	uint8_t 				status, loop, isAlive, isReady, i;

	Dev.platform.address = 0x29;
	ESP_LOGI(TAG, "Testing VL53L5CX, address: %02x", Dev.platform.address);

	// Wait for sensor to wake up.
	vTaskDelay(200/portTICK_RATE_MS);
	status = vl53l5cx_init(&Dev);

	if(status)
	{
		ESP_LOGE(TAG, "VL53L5CX ULD Loading failed\n");
		while (1) {
			ESP_LOGI(TAG, "VL53L5CX ULD Loading failed\n");
			vTaskDelay(200/portTICK_RATE_MS);
		}
	}

	ESP_LOGI(TAG, "VL53L5CX ULD ready ! (Version : %s)\n", VL53L5CX_API_REVISION);

	status = vl53l5cx_start_ranging(&Dev);

	while(1) {
		status = vl53l5cx_check_data_ready(&Dev, &isReady);

		if(isReady) {
			vl53l5cx_get_ranging_data(&Dev, &Results);
			printf("-------------------------------------\n");
			for(i = 0; i < 16; i++)
			{
				printf("Zone : %3d, %4d mm\n",
					Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n-------------------------------------");
			fflush(stdout);
		}

		WaitMs(&(Dev.platform), 10);
	}

	status = vl53l5cx_stop_ranging(&Dev);
}



void test_vl53l5cx()
{
	ESP_LOGI(TAG, "Initialized");
	xTaskCreate(test, TAG, 8192, NULL, 5, NULL);
}