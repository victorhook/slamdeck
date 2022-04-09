#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

#include "com.h"
#include "slamdeck.h"
#include "slamdeck_api.h"
#include "vl53l5cx.h"
#include "led.h"

EventGroupHandle_t startUpEventGroup;
static const int START_UP_API_RX = BIT0;
static const int START_UP_API_TX = BIT1;

VL53L5CX_settings_t settings;

xQueueHandle queueTx;

static slamdeck_state_e slamdeck_state;

static const char* TAG = "SLAMDECK API";


static slamdeck_result_e parse_slamdeck_packet(const esp_routable_packet_t* rx_esp, slamdeck_packet_rx_t* rx)
{
    static slamdeck_packet_packed_t rx_packed;
    slamdeck_result_e result = SLAMDECK_RESULT_OK;
    memcpy(&rx_packed, rx_esp->data, rx_esp->dataLength);
    rx->command = rx_packed.command;
    rx->data = rx_packed.data;
    ESP_LOGI(TAG, "Cmd: %d, sensor: %d, data: %d", rx->command, rx->sensor, rx->data);

    if ((rx->command < 0) || (rx->command >= SLAMDECK_API_TOTAL_COMMANDS)) {
        ESP_LOGW(TAG, "Invalid command!");
        result = SLAMDECK_RESULT_COMMAND_INVALD;
    }

    return result;
}

static void fill_tx_error(slamdeck_packet_tx_t* tx, uint8_t error)
{
    ESP_LOGD(TAG, "Sending error tx: %d", error);
    tx->size = 1;
    tx->data[0] = error;
}

static uint8_t api_handler(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    static VL53L5CX_status_e status[SLAMDECK_NBR_OF_SENSORS];
    uitn8_t do_send_response = 1;

    switch (rx->command) {
        case SLAMDECK_COMMAND_GET_DATA: {
            tx->size = slamdeck_get_sensor_data(&tx->data[sizeof(status)], status);
            break;
        }
        case SLAMDECK_COMMAND_GET_SETTINGS: {
            slamdeck_get_sensor_settings(&settings, status);
            memcpy(tx->data, status, sizeof(status));
            memcpy(&tx->data[sizeof(status)], &settings, sizeof(VL53L5CX_settings_t));
            tx->size = sizeof(status) + sizeof(VL53L5CX_settings_t);
            ESP_LOGD(TAG, "GET SETTINGS SIZE: %d", tx->size);
            ESP_LOGD(TAG, "Size: %d", tx->size);
            break;
        }
        case SLAMDECK_COMMAND_SET_SETTINGS: {
            memcpy(&settings, rx->data, sizeof(VL53L5CX_settings_t));
            slamdeck_set_sensor_settings(&settings, status);
            memcpy(tx->data, status, sizeof(status));
            tx->size = sizeof(status);
            break;
        }
        case SLAMDECK_COMMAND_START_STREAMING: {
            slamdeck_state = SLAMDECK_STATE_STREAMING;
            do_send_response = 0;
            break;
        }
        case SLAMDECK_COMMAND_STOP_STREAMING: {
            slamdeck_state = SLAMDECK_STATE_IDLE;
            do_send_response = 0;
            break;
        }
        default:
            break;
    }

    return do_send_response;
}

static void send_to_cpx(const slamdeck_packet_tx_t* tx)
{
    static esp_routable_packet_t tx_esp = {
        .route={
            .destination=CPX_T_HOST,
            .source=CPX_T_ESP32,
            .function=CPX_F_APP
        }
    };
    ESP_LOGD(TAG, "Sending to cpx: %d bytes", tx->size);
    memcpy(tx_esp.data, (const void*) tx->data, tx->size);
    tx_esp.dataLength = tx->size;
    espAppSendToRouterBlocking(&tx_esp);
}

static void slamdeck_api_task_tx()
{
    xEventGroupSetBits(startUpEventGroup, START_UP_API_TX);
    static slamdeck_packet_rx_t rx_get_data = {.command = SLAMDECK_COMMAND_GET_DATA};
    static slamdeck_packet_tx_t tx;

    while (1) {
        if (xQueueReceive(queueTx, &tx, 0) == pdTRUE) {
            send_to_cpx(&tx);
        }

        if (slamdeck_state == SLAMDECK_STATE_STREAMING) {
            api_handler(&rx_get_data, &tx);
            send_to_cpx(&tx);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


static void slamdeck_api_task_rx()
{
    xEventGroupSetBits(startUpEventGroup, START_UP_API_RX);
    static esp_routable_packet_t rx_esp;
    static slamdeck_packet_rx_t rx;
    static slamdeck_packet_tx_t tx;
    static uint8_t send_response = 1;

    while (1) {
        com_receive_app_blocking(&rx_esp);

        slamdeck_result_e result = parse_slamdeck_packet(&rx_esp, &rx);

        if (result != SLAMDECK_RESULT_OK) {
            fill_tx_error(&tx, result);
        } else {
            send_response = api_handler(&rx, &tx);
        }

        if (send_response)
            xQueueSend(queueTx, &tx, portMAX_DELAY);
    }
}

void slamdeck_api_stop_streaming()
{
    slamdeck_state = SLAMDECK_STATE_IDLE;
}

void slamdeck_api_init()
{
    startUpEventGroup = xEventGroupCreate();
    queueTx = xQueueCreate(1, sizeof(slamdeck_packet_tx_t));
    xEventGroupClearBits(startUpEventGroup, START_UP_API_RX | START_UP_API_TX);

    xTaskCreate(slamdeck_api_task_tx, "API TX", 5000, NULL, 3, NULL);
    xTaskCreate(slamdeck_api_task_rx, "API RX", 5000, NULL, 3, NULL);

    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_API_RX | START_UP_API_TX,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Initialized OK");
}
