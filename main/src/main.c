#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "tests_slamdeck.h"
#include "led.h"
#include "i2c.h"
#include "vl53_test.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "slamdeck.h"

static const char* TAG = "Slamdeck";


void app_main(void)
{
    ESP_LOGI(TAG, "Booting up...");

    led_init();
    i2c_init();
    test_vl53l5cx();

    while (1) {
        //printf("time... %u\n", xTaskGetTickCount());
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

}

/*
    #include "wifi.h"
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init();
*/