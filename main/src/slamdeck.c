#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

// CPX
#include "com.h"

#include "i2c.h"
#include "slamdeck.h"
#include "slamdeck_api.h"
#include "vl53l5cx.h"


static VL53L5CX_t sensors[] = {
    {.id=SLAMDECK_SENSOR_ID_MAIN,  .disabled=0, .enable_pin=SLAMDECK_GPIO_SENSOR_MAIN,  .i2c_address=0x30},  // SLAMDECK_SENSOR_ID_MAIN
    {.id=SLAMDECK_SENSOR_ID_FRONT, .disabled=0, .enable_pin=SLAMDECK_GPIO_SENSOR_FRONT, .i2c_address=0x31},  // SLAMDECK_SENSOR_ID_FRONT
    {.id=SLAMDECK_SENSOR_ID_RIGHT, .disabled=1, .enable_pin=SLAMDECK_GPIO_SENSOR_RIGHT, .i2c_address=0x32},  // SLAMDECK_SENSOR_ID_RIGHT
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
    return slamdeck_get_sensor(sensor) != 0;
}

VL53L5CX_t* slamdeck_get_sensor(const slamdeck_sensor_id_e sensor)
{
    for (uint8_t i = 0; i < nbr_of_enabled_sensors; i++) {
        if (enabled_sensors[i]->id == sensor)
            return enabled_sensors[i];
    }
    return 0;
}

static uint32_t last;
static uint32_t samples;

static void slamdeck_task_check_data_ready()
{
    xEventGroupSetBits(startUpEventGroup, START_UP_CHECK_DATA_READY);
    last = get_current_time();
    while (1) {

        for (uint8_t id = 0; id < nbr_of_enabled_sensors; id++) {
            VL53L5CX_t* sensor = enabled_sensors[id];
            if (VL53L5CX_data_ready(sensor)) {
                /*
                uint32_t now = get_current_time();
                uint32_t dt = now - last;

                if (dt > 1000) {
                    ESP_LOGI(TAG, "Hz: %d", samples);
                    last = now;
                    samples = 0;
                }
                samples++;
                */
                xQueueSend(queueDataReady, &sensor, portMAX_DELAY);
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

static uint8_t set_sensor_default_values(VL53L5CX_t* sensor)
{
    uint8_t result = VL53L5CX_STATUS_OK;
    //VL53L5CX_set_power_mode(sensor, VL53L5CX_POWER_MODE_WAKEUP);
    result |= VL53L5CX_set_resolution(sensor, VL53L5CX_RESOLUTION_4X4);
    result |= VL53L5CX_set_ranging_frequency_hz(sensor, 60); // 60 max for 4x4
    //VL53L5CX_set_ranging_mode(sensor, VL53L5CX_RANGING_MODE_CONTINUOUS);
    //VL53L5CX_set_sharpener_percent(sensor, 10); // Default is 5
    //VL53L5CX_set_target_order(sensor, VL53L5CX_TARGET_ORDER_STRONGEST);
    // Only when raning mode is Autonomous
    //VL53L5CX_set_integration_time_ms(sensor, 5);
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
    for (uint8_t id = 0; id < NBR_OF_SENSORS; id++) {
        sensor = &sensors[id];
        VL53L5CX_init_gpio(sensor);
        VL53L5CX_disable(sensor);
    }

    // Initialize and start all (enabled) sensors
    for (uint8_t id = 0; id < NBR_OF_SENSORS; id++) {
        sensor = &sensors[id];
        if (sensor->disabled)
            continue;

        // Initialize sensor. This include writing ~80kB firmware to it.
        ESP_LOGI(TAG, "Initializing sensor %d", id);
        result = VL53L5CX_init(sensor);

        if (result == VL53L5CX_STATUS_OK) {
            // If initialization was OK, let's start the sensor.
            result = VL53L5CX_start(sensor);
            if (result == VL53L5CX_STATUS_OK) {
                result = set_sensor_default_values(sensor);
            }
        }

        if (result == VL53L5CX_STATUS_OK) {
            enabled_sensors[nbr_of_enabled_sensors] = sensor;
            nbr_of_enabled_sensors++;
            ESP_LOGI(TAG, "Sensor %d initialized", id);
        } else {
            ESP_LOGI(TAG, "Sensor %d fail to initialize, error: %d", id, result);
        }
    }

    queueDataReady = xQueueCreate(nbr_of_enabled_sensors, sizeof(VL53L5CX_t*));
    xEventGroupSetBits(startUpEventGroup, START_UP_SLAMDECK);


    ESP_LOGD(TAG, "%d Sensors started", nbr_of_enabled_sensors);
    VL53L5CX_t* sensor_ready;

    uint32_t t0 = get_current_time();
    uint32_t sample_times[NBR_OF_SENSORS];
    uint32_t samples[NBR_OF_SENSORS];

    while (1) {
        xQueueReceive(queueDataReady, &sensor_ready, portMAX_DELAY);
        VL53L5CX_collect_data(sensor_ready);
        samples[sensor_ready->id]++;
        uint32_t now = get_current_time();
        /*
        if ((now - sample_times[sensor_ready->id]) > 1000) {
            samples[sensor_ready->id] = 0;
            //sample_times[sensor_ready->id] = now;
        } else {
            //samples[sensor_ready->id]++;
        }
        */

        if ((now - t0) > 1000) {
            ESP_LOGD(TAG, "[%d]: %d  [%d]: %d [%d]: %d [%d]: %d", 0, samples[0], 1, samples[1], 3, samples[3], 4, samples[4]);
            t0 = now;
            samples[0] = 0;
            samples[1] = 0;
            samples[3] = 0;
            samples[4] = 0;
        }
        /*
        const VL53L5CX_ResultsData* p_results = VL53L5CX_get_data(sensor_ready);

        // Check if there's api requests do be done.
        if (available_api_request()) {
            execute_api_request();
        }
        */
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

    xEventGroupClearBits(startUpEventGroup, START_UP_SLAMDECK | START_UP_SLAMDECK_API);
    //xTaskCreate(slamdeck_api_task, "Slamdeck API", 6000, NULL, 5, NULL);
    xTaskCreate(slamdeck_task_check_data_ready, "Slamdeck data ready", 5000, NULL, 5, NULL);


    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_CHECK_DATA_READY,//| START_UP_SLAMDECK_API,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "%d Initialized", nbr_of_enabled_sensors);
}


