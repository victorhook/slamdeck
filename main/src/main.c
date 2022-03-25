#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"


#include "led.h"
#include "i2c.h"
#include "vl53_test.h"
#include "vl53l5cx.h"
#include "slamdeck.h"

#include "wifi.h"
#include "router.h"
#include "com.h"

static const char* TAG = "MAIN";


void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set("VL53L5CX", ESP_LOG_DEBUG);
    esp_log_level_set("SLAMDECK", ESP_LOG_DEBUG);
    esp_log_level_set("I2C", ESP_LOG_DEBUG);
    esp_log_level_set("VL53L5CX Api", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    led_init();
    i2c_init();
    //test_vl53l5cx();
    slamdeck_init();

    //com_init();
    //wifi_init();
    //router_init();
    //test_vl53l5cx();

    while (1) {
        vTaskDelay(1000/portTICK_RATE_MS);
    }

}
