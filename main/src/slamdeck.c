#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "slamdeck.h"
#include "vl53l5cx.h"
//#include "esp"

#include "esp_log.h"
#include "esp_err.h"


typedef struct {
    VL53L5CX_id_e id;
    gpio_num_t    enable_pin;
} vl53l5cx_sensor_t;

static const vl53l5cx_sensor_t enabled_sensors[] = {
    {.id=ID_MAIN, .enable_pin=SLAMDECK_GPIO_SENSOR_1}
};

static const char* TAG = "SLAMDECK";


static void slamdeck_task()
{
    uint8_t sensors = sizeof(enabled_sensors) / sizeof(vl53l5cx_sensor_t);
    ESP_LOGD(TAG, "Initializing sensors. Total of %d enabled sensor(s) found.", sensors);

    for (uint8_t i = 0; i < sensors; i++) {
        vl53l5cx_sensor_t sensor = enabled_sensors[i];
        VL53L5CX_init(sensor.id, sensor.enable_pin);
    }

    while (1) {
        vTaskDelay(50/portTICK_RATE_MS);
    }

}


uint8_t slamdeck_init()
{
    xTaskCreate(slamdeck_task, "Slamdeck", 4096, NULL, 3, NULL);
    return 0;
}


