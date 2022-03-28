#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "slamdeck.h"
#include "vl53l5cx.h"

// CPX
#include "com.h"
#include "slamdeck_api.h"


#include "esp_log.h"
#include "esp_err.h"


typedef struct {
    VL53L5CX_id_e id;
    gpio_num_t    enable_pin;
} vl53l5cx_sensor_t;

static const vl53l5cx_sensor_t enabled_sensors[] = {
    //{.id=ID_MAIN, .enable_pin=SLAMDECK_GPIO_SENSOR_MAIN}
    {.id=ID_BACK, .enable_pin=SLAMDECK_GPIO_SENSOR_BACK}
};

static const char* TAG = "SLAMDECK";

static EventGroupHandle_t startUpEventGroup;
static const int START_UP_SLAMDECK = BIT0;
static const int START_UP_SLAMDECK_API = BIT1;

static esp_routable_packet_t rxp;

static void slamdeck_task()
{
    xEventGroupClearBits(startUpEventGroup, START_UP_SLAMDECK);
    uint8_t sensors = sizeof(enabled_sensors) / sizeof(vl53l5cx_sensor_t);

    if (sensors == 0) {
        ESP_LOGE(TAG, "No enabled sensors found!");
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    ESP_LOGD(TAG, "Initializing sensors. Total of %d enabled sensor(s) found.", sensors);
    for (uint8_t i = 0; i < sensors; i++) {
        vl53l5cx_sensor_t sensor = enabled_sensors[i];
        VL53L5CX_init(sensor.id, sensor.enable_pin);
    }

    ESP_LOGD(TAG, "Starting sensors");
    for (uint8_t i = 0; i < sensors; i++) {
        vl53l5cx_sensor_t sensor = enabled_sensors[i];
        VL53L5CX_start(sensor.id);
    }

    ESP_LOGD(TAG, "Sensors started");

    while (1) {
        for (uint8_t i = 0; i < sensors; i++) {
            vl53l5cx_sensor_t sensor = enabled_sensors[i];
            if (VL53L5CX_data_ready(sensor.id)) {
                VL53L5CX_collect_data(sensor.id);
                /*
                VL53L5CX_ResultsData* p_results = VL53L5CX_get_data(sensor.id);
                printf("-------------------------------------\n");
                for(int k = 0; k < 4; k++) {
                    printf(" |");
                    for(int j = 0; j < 4; j++) {
                        printf(" %3d ", p_results->distance_mm[k+j]);
                    }
                    printf(" |\n");
                }
                printf("-------------------------------------\n");
                fflush(stdout);
                */
            }
        }
        vTaskDelay(500/portTICK_RATE_MS);
    }

}


static slamdeck_packet_rx_t slamdeck_api_rx;
static slamdeck_packet_tx_t slamdeck_api_tx;


static void cmd_handler_get_data(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    ESP_LOGD(TAG, "Reading sensor %d", rx->sensor);
    VL53L5CX_ResultsData* data = VL53L5CX_get_data(rx->sensor);
    tx->size = 32;
    memcpy(tx->data, data->distance_mm, tx->size);

    printf("\n");
    for (int i = 0; i < tx->size; i+=2) {
        printf("%3d ", (uint16_t) tx->data[i]);
    }
    printf("\n");

    ESP_LOGD(TAG, "Reading data!");
}

#define SLAMDECK_API_COMMANDS_TOTAL 18

// Array of function pointers to command handlers.
void (*command_handler[SLAMDECK_API_COMMANDS_TOTAL])(const slamdeck_packet_rx_t*, slamdeck_packet_tx_t*);

static inline void printBuff(uint8_t* buff, uint16_t len)
{
    printf("[%d] ", len);
    for (int i = 0; i < len; i++) {
        printf("%02x ", buff[i]);
    }
    printf("\n");
}

static slamdeck_packet_packed_t slamdeck_api_rx_packed;

static void slamdeck_api()
{
    xEventGroupClearBits(startUpEventGroup, START_UP_SLAMDECK_API);
    command_handler[0] = cmd_handler_get_data;

    while (1) {
        // Receive packet & decode.
        com_receive_app_blocking(&rxp);
        ESP_LOGD(TAG, "----- SLAMDECK API --------");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, &rxp, 2, ESP_LOG_DEBUG);

        memcpy(&slamdeck_api_rx_packed, rxp.data, rxp.dataLength);
        slamdeck_api_rx.command = slamdeck_api_rx_packed.command;
        slamdeck_api_rx.sensor = slamdeck_api_rx_packed.sensor;
        slamdeck_api_rx.data = slamdeck_api_rx_packed.data;
        ESP_LOGD(TAG, "cmd: %02x, sensor: %02x, data: %02x", slamdeck_api_rx.command, slamdeck_api_rx.sensor, slamdeck_api_rx.data);

        // Send packet to correct API command handler
        command_handler[0](&slamdeck_api_rx, &slamdeck_api_tx);

        esp_routable_packet_t packet = {
            .route={
                .destination=CPX_T_HOST,
                .source=CPX_T_ESP32,
                .function=CPX_F_APP
            }
        };
        memcpy(packet.data, (const void*) slamdeck_api_tx.data, slamdeck_api_tx.size);
        packet.dataLength = slamdeck_api_tx.size;
        espAppSendToRouterBlocking(&packet);
    }

}


void slamdeck_init()
{
    startUpEventGroup = xEventGroupCreate();
    xEventGroupClearBits(startUpEventGroup, START_UP_SLAMDECK | START_UP_SLAMDECK_API);
    xTaskCreate(slamdeck_task, "Slamdeck", 4096, NULL, 3, NULL);
    xTaskCreate(slamdeck_api, "Slamdeck API", 4096, NULL, 3, NULL);

    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_SLAMDECK | START_UP_SLAMDECK_API,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Initialized");
}


