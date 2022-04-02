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

typedef uint8_t (*vl53l5cx_setter)(VL53L5CX_t* sensor, const uint8_t arg);
static inline void cmd_handler_set_generic(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx, vl53l5cx_setter setter)
{
    tx->size = 1;
    if (slamdeck_sensor_enabled(rx->sensor)) {
        tx->data[0] = setter(rx->sensor, rx->data);
    } else {
        tx->data[0] = SLAMDECK_RESULT_SENSOR_NOT_ENABLED;
    }
}


#define cmd_handler_get_generic(sensor, tx, setting)  \
tx->size = 1;                                         \
if (slamdeck_sensor_enabled(sensor)) {                \
    tx->data[0] = setting;                            \
} else {                                              \
    tx->data[0] = SLAMDECK_RESULT_SENSOR_NOT_ENABLED; \
}


static void cmd_handler_set_i2c_address(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_set_generic(rx, tx, VL53L5CX_set_i2c_address);
}
static void cmd_handler_get_i2c_address(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_get_generic(rx->sensor, tx, slamdeck_get_sensor(rx->sensor)->i2c_address);
}
static void cmd_handler_get_power_mode(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_get_generic(rx->sensor, tx, slamdeck_get_sensor(rx->sensor)->power_mode);
}
static void cmd_handler_set_power_mode(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_set_generic(rx, tx, (vl53l5cx_setter) VL53L5CX_set_power_mode);
}
static void cmd_handler_get_resolution(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_get_generic(rx->sensor, tx, slamdeck_get_sensor(rx->sensor)->resolution);
}
static void cmd_handler_set_resolution(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_set_generic(rx, tx, (vl53l5cx_setter) VL53L5CX_set_resolution);
}
static void cmd_handler_get_ranging_frequency_hz(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_get_generic(rx->sensor, tx, slamdeck_get_sensor(rx->sensor)->ranging_frequency_hz);
}
static void cmd_handler_set_ranging_frequency_hz(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_set_generic(rx, tx, (vl53l5cx_setter) vl53l5cx_set_ranging_frequency_hz);
}
static void cmd_handler_get_integration_time_ms(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_get_generic(rx->sensor, tx, slamdeck_get_sensor(rx->sensor)->integration_time_ms);
}
static void cmd_handler_set_integration_time_ms(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_set_generic(rx, tx, vl53l5cx_set_integration_time_ms); // TODO: This expects uint32..
}
static void cmd_handler_get_sharpener_percent(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_get_generic(rx->sensor, tx, slamdeck_get_sensor(rx->sensor)->sharpener_percent);
}
static void cmd_handler_set_sharpener_percent(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_set_generic(rx, tx, VL53L5CX_set_sharpener_percent);
}
static void cmd_handler_get_target_order(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_get_generic(rx->sensor, tx, slamdeck_get_sensor(rx->sensor)->target_order);
}
static void cmd_handler_set_target_order(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_set_generic(rx, tx, (vl53l5cx_setter) VL53L5CX_set_target_order);
}
static void cmd_handler_get_ranging_mode(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_get_generic(rx->sensor, tx, slamdeck_get_sensor(rx->sensor)->ranging_mode);
}
static void cmd_handler_set_ranging_mode(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    cmd_handler_set_generic(rx, tx, (vl53l5cx_setter) VL53L5CX_set_ranging_mode);
}

static void cmd_handler_test(const slamdeck_packet_rx_t* rx, slamdeck_packet_tx_t* tx)
{
    ESP_LOGD(TAG, "Test!");
    tx->size = 1;
    tx->data[0] = SLAMDECK_RESULT_OK;
}

// Array of function pointers to command handlers.
void (*command_handler[])(const slamdeck_packet_rx_t*, slamdeck_packet_tx_t*) = {
    cmd_handler_test,
    cmd_handler_get_data,
    cmd_handler_get_i2c_address,
    cmd_handler_set_i2c_address,
    cmd_handler_get_power_mode,
    cmd_handler_set_power_mode,
    cmd_handler_get_resolution,
    cmd_handler_set_resolution,
    cmd_handler_get_ranging_frequency_hz,
    cmd_handler_set_ranging_frequency_hz,
    cmd_handler_get_integration_time_ms,
    cmd_handler_set_integration_time_ms,
    cmd_handler_get_sharpener_percent,
    cmd_handler_set_sharpener_percent,
    cmd_handler_get_target_order,
    cmd_handler_set_target_order,
    cmd_handler_get_ranging_mode,
    cmd_handler_set_ranging_mode
};

typedef void (*cmd_handler)(const slamdeck_packet_rx_t*, slamdeck_packet_tx_t*);
typedef void (*cmd_handler_array[])(const slamdeck_packet_rx_t*, slamdeck_packet_tx_t*);

static const int TOTAL_COMMANDS = sizeof(command_handler) / sizeof(cmd_handler);

int available_api_request()
{
    return xQueueReceive(api_request_queue_rx, &rx, 0) == pdTRUE;
}

void execute_api_request()
{
    // Ensure we recognize the command
    if ((rx.command < 0) || (rx.command >= TOTAL_COMMANDS)) {
        // We don't know this command, let the receiver know.
        ESP_LOGW(TAG, "Failed to recognize command %d", rx.command);
        tx.size = 1;
        tx.data[0] = SLAMDECK_RESULT_COMMAND_INVALD;
    } else {
        // Send packet to correct API command handler
        command_handler[rx.command](&rx, &tx);
    }
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
