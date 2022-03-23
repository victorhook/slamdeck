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
	enable_sensor(SLAMDECK_GPIO_SENSOR_1);

	uint8_t 				status, loop, isAlive, isReady, i;

	Dev.platform.address = 0x29;
	ESP_LOGI(TAG, "Testing VL53L5CX, address: %02x", Dev.platform.address);

	// Wait for sensor to wake up.
	vTaskDelay(200/portTICK_RATE_MS);
	uint8_t buf[10];

	uint8_t dev, rev;

	uint8_t buff;
	/*
	for (int i = 0; i < 0xff; i++) {
		RdByte(&Dev.platform, i, &buff);
		ESP_LOGI(TAG, "%02x: %02x", i, buff);
		WrByte(&Dev.platform, 0x7fff, 0x00);
		RdByte(&Dev.platform, i, &buff);
		ESP_LOGI(TAG, "%02x: %02x", i, buff);
	}
	*/

	/*
	while (1) {
		WrByte(&Dev.platform, 0x7fff, 0x00);
		RdByte(&Dev.platform, 0, &dev);
		RdByte(&Dev.platform, 1, &rev);
		WrByte(&Dev.platform, 0x7fff, 0x02);
		//RdByte(&Dev.platform, 1, buf);
		//ESP_LOGI(TAG, "Write: %s", esp_err_to_name(RdByte(&Dev.platform, 0, buf)));
		//ESP_LOGI(TAG, "Write: %s", esp_err_to_name(RdMulti(&Dev.platform, 0, buf, 5)));
		//ESP_LOGI(TAG, "Write: %s", esp_err_to_name(WrByte(&Dev.platform, 10, 1)));
		//ESP_LOGI(TAG, "Write: %s", esp_err_to_name(WrMulti(&Dev.platform, 10, buf, 5)));

		// 0xF0) && (revision_id == (uint8_t)0x02))
		ESP_LOGI(TAG, "Dev: %02x, Rev: %02x", dev, rev);
		vTaskDelay(200/portTICK_RATE_MS);
	}
	*/

	/* (Optional) Check if there is a VL53L5CX sensor connected */
	status = vl53l5cx_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		ESP_LOGE(TAG, "VL53L5CX not detected at requested address\n");
		while (1) {
			vTaskDelay(50/portTICK_RATE_MS);
		}
	}

	ESP_LOGI(TAG, "Sensor found, initializing vl53l5cx...");

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

			/* As the sensor is set in 4x4 mode by default, we have a total
			 * of 16 zones to print. For this example, only the data of first zone are
			 * print */
			printf("Print data no : %3u\n", Dev.streamcount);
			for(i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
					i,
					Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n");
		}

		WaitMs(&(Dev.platform), 50);
	}

	status = vl53l5cx_stop_ranging(&Dev);
}



void test_vl53l5cx()
{
	ESP_LOGI(TAG, "Initialized");
	xTaskCreate(test, TAG, 4096, NULL, 5, NULL);
}