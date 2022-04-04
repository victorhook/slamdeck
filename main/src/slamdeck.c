#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

// CPX
#include "com.h"

#include "i2c.h"
#include "slamdeck.h"
#include "slamdeck_api.h"
#include "vl53l5cx.h"


static VL53L5CX_t sensors[] = {
    {.id=SLAMDECK_SENSOR_ID_MAIN,  .disabled=0, .enable_pin=SLAMDECK_GPIO_SENSOR_MAIN,  .i2c_address=0x30},  // SLAMDECK_SENSOR_ID_MAIN
    {.id=SLAMDECK_SENSOR_ID_FRONT, .disabled=0, .enable_pin=SLAMDECK_GPIO_SENSOR_FRONT, .i2c_address=0x31},  // SLAMDECK_SENSOR_ID_FRONT
    {.id=SLAMDECK_SENSOR_ID_RIGHT, .disabled=0, .enable_pin=SLAMDECK_GPIO_SENSOR_RIGHT, .i2c_address=0x32},  // SLAMDECK_SENSOR_ID_RIGHT
    {.id=SLAMDECK_SENSOR_ID_BACK,  .disabled=0, .enable_pin=SLAMDECK_GPIO_SENSOR_BACK,  .i2c_address=0x33},  // SLAMDECK_SENSOR_ID_BACK
    {.id=SLAMDECK_SENSOR_ID_LEFT,  .disabled=0, .enable_pin=SLAMDECK_GPIO_SENSOR_LEFT,  .i2c_address=0x34}   // SLAMDECK_SENSOR_ID_LEFT
};


#define NBR_OF_SENSORS sizeof(sensors) / sizeof(VL53L5CX_t)

static VL53L5CX_t* enabled_sensors[NBR_OF_SENSORS];
static uint8_t nbr_of_enabled_sensors = 0;


static EventGroupHandle_t startUpEventGroup;
static const int START_UP_SLAMDECK = BIT0;
static const int START_UP_SLAMDECK_API = START_UP_SLAMDECK_API_BIT;
static const int START_UP_CHECK_DATA_READY = BIT2;

static xQueueHandle queueDataReady;

static const char* TAG = "SLAMDECK";

// vl53l5cx_set_resolution before ranging freq

uint8_t slamdeck_sensor_enabled(const slamdeck_sensor_id_e sensor)
{
    return !slamdeck_get_sensor(sensor)->disabled;
}

VL53L5CX_t* slamdeck_get_sensor(const slamdeck_sensor_id_e sensor)
{
    if ((sensor < 0) || (sensor >= NBR_OF_SENSORS)) {
        ESP_LOGE(TAG, "Sensor id %d out of bounds!", sensor);
        return 0;
    }
    return &sensors[sensor];
}

static uint32_t last;
static uint32_t samples;

typedef struct {
    uint64_t last_check;
    uint64_t backoff_us;
    uint8_t  active;
} data_check_backoff_t;

static void slamdeck_task_check_data_ready()
{
    xEventGroupSetBits(startUpEventGroup, START_UP_CHECK_DATA_READY);
    data_check_backoff_t backoff[5];
    memset(backoff, 0, 2*sizeof(data_check_backoff_t));

    while (1) {

        for (uint8_t i = 0; i < nbr_of_enabled_sensors; i++) {
            VL53L5CX_t* sensor = enabled_sensors[i];

            uint64_t now = esp_timer_get_time();
            data_check_backoff_t* back = &backoff[i];
            back->backoff_us = 2000;

            if (back->active && ( (now - back->last_check) < back->backoff_us)) {
                continue;
            }

            back->last_check = now;

            if (VL53L5CX_data_ready(sensor)) {
                back->active = 0;
                xQueueSend(queueDataReady, &sensor, portMAX_DELAY);
            } else {
                back->active = 1;
            }
        }

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

static uint8_t set_sensor_default_values(VL53L5CX_t* sensor)
{
    uint8_t result = VL53L5CX_STATUS_OK;
    //VL53L5CX_set_power_mode(sensor, VL53L5CX_POWER_MODE_WAKEUP);
    result |= VL53L5CX_set_resolution(sensor, VL53L5CX_RESOLUTION_8X8);
    result |= VL53L5CX_set_ranging_frequency_hz(sensor, 15); // 60 max for 4x4
    result |= VL53L5CX_set_ranging_mode(sensor, VL53L5CX_RANGING_MODE_AUTONOMOUS);
    result |= VL53L5CX_set_sharpener_percent(sensor, 5);
    result |= VL53L5CX_set_target_order(sensor, VL53L5CX_TARGET_ORDER_STRONGEST);
    // Only when raning mode is Autonomous
    //result |= VL53L5CX_set_integration_time_ms(sensor, 1);
    return result;
}

static void slamdeck_task()
{
    // Ensure i2c is initialized before starting
    while (!i2c_is_initialized()) {
        vTaskDelay(50/portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Initializing %d sensors.", NBR_OF_SENSORS);
    VL53L5CX_t* sensor;
    VL53L5CX_status_e result;

    vTaskDelay(10/portTICK_PERIOD_MS);

    // Initialize gpio and disable all sensors
    for (uint8_t i = 0; i < NBR_OF_SENSORS; i++) {
        sensor = &sensors[i];
        VL53L5CX_init_gpio(sensor);
        VL53L5CX_disable(sensor);
    }

    // Initialize all (enabled) sensors
    for (uint8_t i = 0; i < NBR_OF_SENSORS; i++) {
        sensor = &sensors[i];
        if (sensor->disabled)
            continue;

        // Initialize sensor. This include writing ~80kB firmware to it.
        ESP_LOGI(TAG, "Initializing sensor %d", i);
        result = VL53L5CX_init(sensor);

        if (result == VL53L5CX_STATUS_OK) {
            enabled_sensors[nbr_of_enabled_sensors] = sensor;
            nbr_of_enabled_sensors++;
            ESP_LOGI(TAG, "Sensor %d initialized", i);
        } else {
            ESP_LOGI(TAG, "Sensor %d fail to initialize, error: %d", i, result);
        }
    }


    // Start all initialized sensors
    ESP_LOGI(TAG, "Starting sensors...");
    for (uint8_t i = 0; i < nbr_of_enabled_sensors; i++) {
        ESP_LOGI(TAG, "Starting sensor %d", i);
        sensor = enabled_sensors[i];
        result = VL53L5CX_start(sensor);
        if (result == VL53L5CX_STATUS_OK) {
            result = set_sensor_default_values(sensor);
            if (result != VL53L5CX_STATUS_OK) {
                ESP_LOGW(TAG, "Failed to set default values to sensor %d", sensor->id);
            }
        }
    }

    queueDataReady = xQueueCreate(nbr_of_enabled_sensors, sizeof(VL53L5CX_t*));
    xEventGroupSetBits(startUpEventGroup, START_UP_SLAMDECK);

    ESP_LOGD(TAG, "%d Sensors started", nbr_of_enabled_sensors);
    VL53L5CX_t* sensor_ready;

    uint64_t t0 = esp_timer_get_time();
    uint32_t samples[NBR_OF_SENSORS];
    char buf[200];

    while (1) {
        xQueueReceive(queueDataReady, &sensor_ready, portMAX_DELAY);
        VL53L5CX_collect_data(sensor_ready);
        samples[sensor_ready->id]++;

        uint64_t now = esp_timer_get_time();
        uint64_t dt = (now - t0) / 1000;
        uint8_t seconds = 1;

        if (dt > 1000*seconds) {
            t0 = now;
            for (int i = 0; i < nbr_of_enabled_sensors; i++) {
                sensor = enabled_sensors[i];
                printf("%lld: %dC | ", sensor->samples / seconds, sensor->result.silicon_temp_degc);
                /*
                for (int j = 0; j < 16; j+=2) {
                    printf("%d ", (uint16_t) enabled_sensors[i]->result.distance_mm[j]);
                }
                printf("\n");
                */
                enabled_sensors[i]->samples = 0;
            }
            printf("\n");
            //vTaskGetRunTimeStats(buf);
            //printf(buf);
        }

        // Check if there's api requests do be done.
        #ifndef DISABLED_WIFI_API
            if (available_api_request()) {
                execute_api_request();
            }
        #endif
    }

}

void slamdeck_init()
{
    startUpEventGroup = xEventGroupCreate();
    xEventGroupClearBits(startUpEventGroup, START_UP_SLAMDECK | START_UP_SLAMDECK_API | START_UP_CHECK_DATA_READY);
    xTaskCreate(slamdeck_task, "Slamdeck", 8000, NULL, 5, NULL);

    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_SLAMDECK,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    xTaskCreate(slamdeck_task_check_data_ready, "Slamdeck data ready", 5000, NULL, 4, NULL);

    #ifndef DISABLED_WIFI_API
    xTaskCreate(slamdeck_api_task, "Slamdeck API", 6000, NULL, 4, NULL);
    #endif

    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_CHECK_DATA_READY
                        #ifndef DISABLED_WIFI_API
                        | START_UP_SLAMDECK_API
                        #endif
                        ,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "%d sensors initialized", nbr_of_enabled_sensors);
}


