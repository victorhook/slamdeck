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

xQueueHandle queueTx;

static slamdeck_state_e slamdeck_state = SLAMDECK_STATE_IDLE;

static const char* TAG = "SLAMDECK API";


static slamdeck_result_e parse_slamdeck_packet(const esp_routable_packet_t* rx_esp, slamdeck_packet_rx_t* rx)
{
    rx->command = rx_esp->data[0];
    uint16_t data_size = rx_esp->dataLength - 1;
    if (data_size > 0) {
        memcpy(rx->data, &rx_esp->data[1], data_size);
    }
    ESP_LOGD(TAG, "Cmd: %d, data length: %d", rx->command, data_size);

    if ((rx->command < 0) || (rx->command >= SLAMDECK_API_TOTAL_COMMANDS)) {
        ESP_LOGW(TAG, "Invalid command: %d", rx->command);
        return SLAMDECK_RESULT_COMMAND_INVALD;
    }

    return SLAMDECK_RESULT_OK;
}

static void fill_tx_error(slamdeck_packet_tx_t* tx, uint8_t error)
{
    ESP_LOGD(TAG, "Sending error tx: %d", error);
    tx->size = 1;
    tx->data[0] = error;
}

static uint8_t api_handler(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    static VL53L5CX_settings_t settings;
    static VL53L5CX_status_e status[SLAMDECK_NBR_OF_SENSORS];
    uint8_t send_response = 1;

    switch (rx->command) {
        case SLAMDECK_COMMAND_GET_DATA: {
            tx->size = slamdeck_get_sensor_data(tx->data, status);
            //tx->size = 640;
            //for (int i = 0; i < tx->size; i++) {
            //    tx->data[i] = i;
            //}
            //ESP_LOGI(TAG, "Size: %d", tx->size);
            //ESP_LOG_BUFFER_HEX(TAG, tx->data, tx->size);
            break;
        }
        case SLAMDECK_COMMAND_GET_SETTINGS: {
            ESP_LOGD(TAG, "Getting settings");
            slamdeck_get_sensor_settings(&settings, status);
            memcpy(tx->data, status, sizeof(status));
            memcpy(&tx->data[sizeof(status)], &settings, sizeof(VL53L5CX_settings_t));
            tx->size = sizeof(status) + sizeof(VL53L5CX_settings_t);
            break;
        }
        case SLAMDECK_COMMAND_SET_SETTINGS: {
            ESP_LOGD(TAG, "Setting settings");
            memcpy(&settings, rx->data, sizeof(VL53L5CX_settings_t));
            slamdeck_set_sensor_settings(&settings, status);
            memcpy(tx->data, status, sizeof(status));
            tx->size = sizeof(status);
            break;
        }
        case SLAMDECK_COMMAND_START_STREAMING: {
            ESP_LOGD(TAG, "Start streaming");
            slamdeck_state = SLAMDECK_STATE_STREAMING;
            send_response = 0;
            break;
        }
        case SLAMDECK_COMMAND_STOP_STREAMING: {
            ESP_LOGD(TAG, "Stop streaming");
            slamdeck_state = SLAMDECK_STATE_IDLE;
            send_response = 0;
            break;
        }
        default:
            ESP_LOGW(TAG, "Unknown command: %d", rx->command);
            break;
    }

    return send_response;
}

static void send_to_cpx(const slamdeck_packet_tx_t* tx)
{
    static esp_routable_packet_t tx_esp;
    //cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_APP, &tx_esp.route);
    cpxInitRoute(CPX_T_ESP32, CPX_T_HOST_NRF, CPX_F_APP, &tx_esp.route);
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

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

static void slamdeck_api_task_rx()
{
    xEventGroupSetBits(startUpEventGroup, START_UP_API_RX);
    static esp_routable_packet_t rx_esp;
    static slamdeck_packet_rx_t rx;
    static slamdeck_packet_tx_t tx;

    while (1) {
        com_receive_app_blocking(&rx_esp);
        uint8_t send_response = 1;

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

    xTaskCreate(slamdeck_api_task_tx, "API TX", 5000, NULL, 2, NULL);
    xTaskCreate(slamdeck_api_task_rx, "API RX", 5000, NULL, 2, NULL);

    slamdeck_state = SLAMDECK_STATE_STREAMING;

    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_API_RX | START_UP_API_TX,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Initialized OK");
}
