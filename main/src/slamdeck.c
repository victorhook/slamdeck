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
    {.id=SLAMDECK_SENSOR_ID_MAIN,  .status=VL53L5CX_STATUS_OK, .enable_pin=SLAMDECK_GPIO_SENSOR_MAIN,  .i2c_address=0x30},  // SLAMDECK_SENSOR_ID_MAIN
    {.id=SLAMDECK_SENSOR_ID_FRONT, .status=VL53L5CX_STATUS_OK, .enable_pin=SLAMDECK_GPIO_SENSOR_FRONT, .i2c_address=0x31},  // SLAMDECK_SENSOR_ID_FRONT
    {.id=SLAMDECK_SENSOR_ID_RIGHT, .status=VL53L5CX_STATUS_OK, .enable_pin=SLAMDECK_GPIO_SENSOR_RIGHT, .i2c_address=0x32},  // SLAMDECK_SENSOR_ID_RIGHT
    {.id=SLAMDECK_SENSOR_ID_BACK,  .status=VL53L5CX_STATUS_OK, .enable_pin=SLAMDECK_GPIO_SENSOR_BACK,  .i2c_address=0x33},  // SLAMDECK_SENSOR_ID_BACK
    {.id=SLAMDECK_SENSOR_ID_LEFT,  .status=VL53L5CX_STATUS_OK, .enable_pin=SLAMDECK_GPIO_SENSOR_LEFT,  .i2c_address=0x34}   // SLAMDECK_SENSOR_ID_LEFT
};

static const VL53L5CX_settings_t sensor_settings_default = {
    .integration_time_ms = 5,
    .sharpener_percent = 5,
    .ranging_frequency_hz = 15,
    .resolution = VL53L5CX_RESOLUTION_8X8,
    .power_mode = VL53L5CX_POWER_MODE_WAKEUP,
    .target_order = VL53L5CX_TARGET_ORDER_STRONGEST,
    .ranging_mode = VL53L5CX_RANGING_MODE_AUTONOMOUS
};

static VL53L5CX_settings_t sensor_settings;

static EventGroupHandle_t startUpEventGroup;
static const int START_UP_SLAMDECK         = BIT0;
static const int START_UP_CHECK_DATA_READY = BIT1;

static xQueueHandle queueDataReady;

static const char* TAG = "SLAMDECK";


/* --- Private --- */
static void print_sensor_settings(const VL53L5CX_settings_t* settings)
{
    printf("-------------- Settings -----------------\n");
    printf("power_mode: %d\n", settings->power_mode);
    printf("resolution: %d\n", settings->resolution);
    printf("ranging_frequency_hz: %d\n", settings->ranging_frequency_hz);
    printf("integration_time_ms: %d\n", settings->integration_time_ms);
    printf("sharpener_percent: %d\n", settings->sharpener_percent);
    printf("target_order: %d\n", settings->target_order);
    printf("ranging_mode: %d\n", settings->ranging_mode);
    //printf("temperature: %d\n", result->silicon_temp_degc);
    printf("-------------------------------------------\n");
    fflush(stdout);
}

static void print_sensor_data(const VL53L5CX_t* sensor)
{
    int grid = 0;
    int row_size = sensor->settings.resolution == VL53L5CX_RESOLUTION_4X4 ? 4 : 8;
    for (int row = row_size-1; row >= 0; row--) {
        for (int col = 0; col < row_size; col++) {
            printf("| %3u  %4d ", sensor->result.target_status[grid], sensor->data_distance_mm[grid]);
            grid++;
        }
        printf("\n");
    }
    printf("\n");
}

static uint16_t get_data_size(const VL53L5CX_settings_t* settings)
{
    return settings->resolution * 2 * VL53L5CX_NB_TARGET_PER_ZONE;
}

static VL53L5CX_status_e set_single_sensor_settings(VL53L5CX_t* sensor, const VL53L5CX_settings_t* settings)
{
    ESP_LOGD(TAG, "Sensor status: %d", sensor->status);
    if (sensor->status != VL53L5CX_STATUS_OK)
        return sensor->status;

    uint8_t require_restart = sensor->is_ranging;

    if (require_restart) {
        VL53L5CX_stop(sensor);
        if (sensor->status != VL53L5CX_STATUS_OK)
            return sensor->status;
    }

    VL53L5CX_set_power_mode(sensor, settings->power_mode);
    VL53L5CX_set_resolution(sensor, settings->resolution);
    VL53L5CX_set_ranging_frequency_hz(sensor, settings->ranging_frequency_hz);
    VL53L5CX_set_ranging_mode(sensor, settings->ranging_mode);
    VL53L5CX_set_sharpener_percent(sensor, settings->sharpener_percent);
    VL53L5CX_set_target_order(sensor, settings->target_order);
    VL53L5CX_set_integration_time_ms(sensor, settings->integration_time_ms);

    if (sensor->status != VL53L5CX_STATUS_OK) {
        ESP_LOGW(TAG, "Failed to set config, error: %d", sensor->status);
        return sensor->status;
    }

    if (require_restart) {
        VL53L5CX_start(sensor);
        if (sensor->status != VL53L5CX_STATUS_OK) {
            return sensor->status;
        }
    }

    memcpy(&sensor->settings, settings, sizeof(VL53L5CX_settings_t));

    return sensor->status;
}

static VL53L5CX_status_e get_single_sensor_data(VL53L5CX_t* sensor, uint8_t* buf, const uint16_t data_size)
{
    if (sensor->status == VL53L5CX_STATUS_NOT_ENABLED)
        return sensor->status;
    memcpy(buf, sensor->data_distance_mm, data_size);
    return sensor->status;
}

/* --- Public --- */
void slamdeck_set_sensor_settings(const VL53L5CX_settings_t* settings, VL53L5CX_status_e status[SLAMDECK_NBR_OF_SENSORS])
{
    memcpy(&sensor_settings, settings, sizeof(VL53L5CX_settings_t));
    for (slamdeck_sensor_id_e sensor_id = 0; sensor_id < SLAMDECK_NBR_OF_SENSORS; sensor_id++) {
        status[sensor_id] = set_single_sensor_settings(&sensors[sensor_id], &sensor_settings);
    }
}

void slamdeck_get_sensor_settings(VL53L5CX_settings_t* settings, VL53L5CX_status_e status[SLAMDECK_NBR_OF_SENSORS])
{
    for (slamdeck_sensor_id_e sensor_id = 0; sensor_id < SLAMDECK_NBR_OF_SENSORS; sensor_id++) {
        status[sensor_id] = sensors[sensor_id].status;
    }
    memcpy(settings, &sensor_settings, sizeof(VL53L5CX_settings_t));
}

uint16_t slamdeck_get_sensor_data(uint8_t* buf, VL53L5CX_status_e status[SLAMDECK_NBR_OF_SENSORS])
{
    uint16_t data_size = get_data_size(&sensor_settings);
    //printf("%d\n", data_size);
    uint16_t offset = 0;
    for (slamdeck_sensor_id_e sensor_id = 0; sensor_id < SLAMDECK_NBR_OF_SENSORS; sensor_id++) {
        status[sensor_id] = get_single_sensor_data(&sensors[sensor_id], &buf[offset], data_size);
        offset += data_size;
    }

    return data_size * SLAMDECK_NBR_OF_SENSORS;
}

/* --- FreeRTOS Threads --- */
static void slamdeck_task_check_data_ready()
{
    xEventGroupSetBits(startUpEventGroup, START_UP_CHECK_DATA_READY);

    while (1) {

        for (uint8_t i = 0; i < SLAMDECK_NBR_OF_SENSORS; i++) {
            VL53L5CX_t* sensor = &sensors[i];
            if ((sensor->status == VL53L5CX_STATUS_OK) && VL53L5CX_data_ready(sensor))
                xQueueSend(queueDataReady, &sensor, portMAX_DELAY);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void slamdeck_task()
{
    // Set default sensor settings
    memcpy(&sensor_settings, &sensor_settings_default, sizeof(VL53L5CX_settings_t));

    // Ensure i2c is initialized before starting
    while (!i2c_is_initialized()) {
        vTaskDelay(50/portTICK_PERIOD_MS);
    }


    ESP_LOGI(TAG, "Initializing %d sensors.", SLAMDECK_NBR_OF_SENSORS);
    VL53L5CX_t* sensor;

    vTaskDelay(10/portTICK_PERIOD_MS);

    // Initialize gpio and disable all sensors
    for (uint8_t i = 0; i < SLAMDECK_NBR_OF_SENSORS; i++) {
        sensor = &sensors[i];
        VL53L5CX_init_gpio(sensor);
        VL53L5CX_disable(sensor);
    }

    // Initialize all (enabled) sensors
    int initialized_sensors = 0;
    for (uint8_t i = 0; i < SLAMDECK_NBR_OF_SENSORS; i++) {
        sensor = &sensors[i];
        if (sensor->status == VL53L5CX_STATUS_NOT_ENABLED)
            continue;

        // Initialize sensor. This include writing ~80kB firmware to it.
        ESP_LOGI(TAG, "Initializing sensor %d", i);
        sensor->status = VL53L5CX_init(sensor);

        if (sensor->status == VL53L5CX_STATUS_OK) {
            initialized_sensors++;
            ESP_LOGI(TAG, "Sensor %d initialized", i);
        } else {
            ESP_LOGI(TAG, "Sensor %d fail to initialize, error: %d", i, sensor->status);
        }
    }
    ESP_LOGD(TAG, "%d Sensors initialized", initialized_sensors);

    // Set settings
    VL53L5CX_status_e status[SLAMDECK_NBR_OF_SENSORS];
    slamdeck_set_sensor_settings(&sensor_settings_default, status);

    // Start all initialized sensors
    int started_sensors = 0;
    for (uint8_t i = 0; i < SLAMDECK_NBR_OF_SENSORS; i++) {
        sensor = &sensors[i];
        if (sensor->status != VL53L5CX_STATUS_OK)
            continue;

        ESP_LOGI(TAG, "Starting sensor %d", i);

        sensor->status = VL53L5CX_start(sensor);
        if (sensor->status == VL53L5CX_STATUS_OK) {
            //status = set_sensor_default_values(sensor);
            started_sensors++;
            if (sensor->status != VL53L5CX_STATUS_OK) {
                ESP_LOGW(TAG, "Failed to set default values to sensor %d", sensor->id);
            }
        }
    }
    ESP_LOGD(TAG, "%d Sensors started", started_sensors);


    queueDataReady = xQueueCreate(SLAMDECK_NBR_OF_SENSORS, sizeof(VL53L5CX_t*));
    xEventGroupSetBits(startUpEventGroup, START_UP_SLAMDECK);

    VL53L5CX_t* sensor_ready;

    uint64_t t0 = esp_timer_get_time();
    uint32_t samples[SLAMDECK_NBR_OF_SENSORS];

    while (1) {
        xQueueReceive(queueDataReady, &sensor_ready, portMAX_DELAY);
        VL53L5CX_collect_data(sensor_ready);
        samples[sensor_ready->id]++;

        uint64_t now = esp_timer_get_time();
        uint64_t dt = (now - t0) / 1000;
        uint8_t seconds = 3;

        if (dt > 1000*seconds) {
            t0 = now;
            for (int i = 0; i < SLAMDECK_NBR_OF_SENSORS; i++) {
                sensor = &sensors[i];

                if (sensor->status != VL53L5CX_STATUS_OK)
                    continue;

                printf("%lld: %dC | ", sensor->samples / seconds, sensor->result.silicon_temp_degc);
                //print_sensor(sensor);
                //print_sensor_data(sensor);
                sensor->samples = 0;
            }
            printf("\n");
            //print_sensor_data(&sensors[SLAMDECK_SENSOR_ID_BACK]);
        }



    }
}


void slamdeck_init()
{
    startUpEventGroup = xEventGroupCreate();

    xEventGroupClearBits(startUpEventGroup, START_UP_SLAMDECK | START_UP_CHECK_DATA_READY);
    xTaskCreate(slamdeck_task, "Slamdeck", 8000, NULL, 5, NULL);

    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_SLAMDECK,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    xTaskCreate(slamdeck_task_check_data_ready, "Slamdeck DR", 5000, NULL, 4, NULL);

    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_CHECK_DATA_READY,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "initialized");
}


