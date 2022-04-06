#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

// CPX
#include "com.h"
#include "slamdeck_api.h"
#include "vl53l5cx.h"
#include "led.h"


static esp_routable_packet_t rx_esp;
static esp_routable_packet_t tx_esp = {
        .route={
            .destination=CPX_T_HOST,
            .source=CPX_T_ESP32,
            .function=CPX_F_APP
        }
    };
static slamdeck_packet_rx_t rx;
static slamdeck_packet_tx_t tx;

static xQueueHandle api_request_queue_rx;
static xQueueHandle api_request_queue_tx;
#define MAX_API_REQUESTS_QUEUE_SIZE 1

static const char* TAG = "SLAMDECK Api";


static inline void printBuff(uint8_t* buff, uint16_t len)
{
    printf("[%d] ", len);
    for (int i = 0; i < len; i++) {
        printf("%02x ", buff[i]);
    }
    printf("\n");
}

/* --- Slamdeck API --- */

static void cmd_handler_get_data(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    VL53L5CX_t* sensor = slamdeck_get_sensor(rx->sensor);
    tx->size = VL53L5CX_get_data_size(sensor);
    ESP_LOGD(TAG, "Get data sensor %d, %d bytes", rx->sensor, tx->size);
    memcpy(tx->data, (uint8_t*) sensor->result.distance_mm, tx->size);
}

int available_api_request()
{
    return xQueueReceive(api_request_queue_rx, &rx, 0) == pdTRUE;
}

static uint16_t handle_sensor_request(const slamdeck_sensor_id_e sensor_id, const slamdeck_command_e command, const uint8_t* rx_data, uint8_t* tx_buf)
{
    if (!slamdeck_sensor_enabled(sensor_id)) {
        tx_buf[0] = SLAMDECK_RESULT_SENSOR_NOT_ENABLED;
        return;
    }
    VL53L5CX_t* sensor = slamdeck_get_sensor(sensor_id);

    // Send packet to correct API command handler
    uint16_t tx_length = 1;

    switch (command) {
        case SLAMDECK_GET_SENSOR_STATUS:
            ESP_LOGD(TAG, "Test!");
            tx_buf[0] = SLAMDECK_RESULT_OK;
            break;
        case SLAMDECK_GET_DATA:
            tx_length = VL53L5CX_get_data_size(sensor);
            ESP_LOGD(TAG, "Get data sensor %d, %d bytes", sensor->id, tx_length);
            memcpy(tx_buf, (uint8_t*) sensor->data_distance_mm, tx_length);
            break;
        case SLAMDECK_GET_I2C_ADDRESS:
            tx_buf[0] = VL53L5CX_get_i2c_address(sensor);
            break;
        case SLAMDECK_SET_I2C_ADDRESS:
            tx_buf[0] = VL53L5CX_set_i2c_address(sensor, rx_data[0]);
            break;
        case SLAMDECK_GET_POWER_MODE:
            tx_buf[0] = VL53L5CX_get_power_mode(sensor);
            break;
        case SLAMDECK_SET_POWER_MODE:
            tx_buf[0] = VL53L5CX_set_power_mode(sensor, rx_data[0]);
            break;
        case SLAMDECK_GET_RESOLUTION:
            tx_buf[0] = VL53L5CX_get_resolution(sensor);
            break;
        case SLAMDECK_SET_RESOLUTION:
            tx_buf[0] = VL53L5CX_set_resolution(sensor, rx_data[0]);
            break;
        case SLAMDECK_GET_RANGING_FREQUENCY_HZ:
            tx_buf[0] = VL53L5CX_get_ranging_frequency_hz(sensor);
            break;
        case SLAMDECK_SET_RANGING_FREQUENCY_HZ:
            tx_buf[0] = VL53L5CX_set_ranging_frequency_hz(sensor, rx_data[0]);
            break;
        case SLAMDECK_GET_INTEGRATION_TIME_MS:
            ((uint32_t*) tx_buf)[0] = VL53L5CX_get_integration_time_ms(sensor);
            tx_length = 4;
            break;
        case SLAMDECK_SET_INTEGRATION_TIME_MS:
            tx_buf[0] = VL53L5CX_set_integration_time_ms(sensor, (uint32_t) rx_data[0]);
            break;
        case SLAMDECK_GET_SHARPENER_PERCENT:
            tx_buf[0] = VL53L5CX_get_sharpener_percent(sensor);
            break;
        case SLAMDECK_SET_SHARPENER_PERCENT:
            tx_buf[0] = vl53l5cx_set_sharpener_percent(sensor, rx_data[0]);
            break;
        case SLAMDECK_GET_TARGET_ORDER:
            tx_buf[0] = VL53L5CX_get_target_order(sensor);
            break;
        case SLAMDECK_SET_TARGET_ORDER:
            tx_buf[0] = vl53l5cx_set_target_order(sensor, rx_data[0]);
            break;
        case SLAMDECK_GET_RANGING_MODE:
            tx_buf[0] = VL53L5CX_get_ranging_mode(sensor);
            break;
        case SLAMDECK_SET_RANGING_MODE:
            tx_buf[0] = vl53l5cx_set_ranging_mode(sensor, rx_data[0]);
            break;
    }

    return tx_length;
}

void execute_api_request()
{
    uint16_t tx_size = 0;

    // Ensure we recognize the command
    if ((rx.command < 0) || (rx.command >= SLAMDECK_API_TOTAL_COMMANDS)) {
        // We don't know this command, let the receiver know.
        ESP_LOGW(TAG, "Failed to recognize command %d", rx.command);
        tx_size = 1;
        tx.data[0] = SLAMDECK_RESULT_COMMAND_INVALD;
    } else {

        // Check if request is for all sensors, or just one.
        if (rx.sensor == SLAMDECK_SENSOR_ID_ALL) {
            for (int sensor_id = 0; sensor_id <= 4; sensor_id++) {
                tx_size += handle_sensor_request(sensor_id, rx.command, &rx.data, &tx.data[tx_size]);
            }
        } else {
            tx_size += handle_sensor_request(rx.sensor, rx.command, &rx.data, tx.data);

        }

    }

    tx.size = tx_size;
    xQueueSend(api_request_queue_tx, &tx, portMAX_DELAY);
}

void slamdeck_api_task()
{
    api_request_queue_rx = xQueueCreate(MAX_API_REQUESTS_QUEUE_SIZE, sizeof(slamdeck_packet_rx_t));
    api_request_queue_tx = xQueueCreate(MAX_API_REQUESTS_QUEUE_SIZE, sizeof(slamdeck_packet_tx_t));

    static slamdeck_packet_packed_t rx_packed;

    while (1) {
        // Receive packet & decode.
        ESP_LOGD(TAG, "Waiting for slamdeck packet");
        com_receive_app_blocking(&rx_esp);
        led_set_state(LED_RED, LED_STATE_ON);
        //ESP_LOG_BUFFER_HEX_LEVEL(TAG, &rxp, 2, ESP_LOG_DEBUG);

        memcpy(&rx_packed, rx_esp.data, rx_esp.dataLength);
        rx.command = rx_packed.command;
        rx.sensor = rx_packed.sensor;
        rx.data = rx_packed.data;
        ESP_LOGD(TAG, "cmd: %02x, sensor: %02x, data: %02x", rx.command, rx.sensor, rx.data);

        // Send packet to slamdeck receive queue
        xQueueSend(api_request_queue_rx, &rx, portMAX_DELAY);

        // Wait for Slamdeck api request to finish
        xQueueReceive(api_request_queue_tx, &tx, portMAX_DELAY);

        // Copy data into esp_transport data packet.
        ESP_LOGI(TAG, "Sending with size: %d", tx.size);
        memcpy(tx_esp.data, (const void*) tx.data, tx.size);
        tx_esp.dataLength = tx.size;
        espAppSendToRouterBlocking(&tx_esp);
        led_set_state(LED_RED, LED_STATE_OFF);
    }

}
