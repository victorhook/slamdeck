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
#include "slamdeck_api.h"

#include "com.h"
#include "WIFI_SSID.h"
#include "wifi.h"
#include "router.h"
#include "uart_transport.h"

#include "nvs_flash.h"
#include "esp_event.h"


static const char* TAG = "MAIN";

static void wifi_ctrl()
{
    static esp_routable_packet_t packet = {
        .route={
            .destination=CPX_T_ESP32,
            .source=CPX_T_ESP32,
            .function=CPX_F_WIFI_CTRL
        }
    };
    static const char* SSID = WIFI_SSID;
    static const char* PASS = WIFI_PASS;

    packet.data[0] = WIFI_CTRL_SET_SSID;
    memcpy(&packet.data[1], SSID, strlen(SSID));
    packet.dataLength = 1 + strlen(SSID);
    espAppSendToRouterBlocking(&packet);

    packet.data[0] = WIFI_CTRL_SET_KEY;
    memcpy(&packet.data[1], PASS, strlen(PASS));
    packet.dataLength = 1 + strlen(PASS);
    espAppSendToRouterBlocking(&packet);

    packet.data[0] = WIFI_CTRL_WIFI_CONNECT;
    packet.data[1] = 0; // Indicate STA mode
    packet.dataLength = 1 + 1;
    espAppSendToRouterBlocking(&packet);
}


void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("VL53L5CX", ESP_LOG_DEBUG);
    esp_log_level_set("I2C", ESP_LOG_DEBUG);
    //esp_log_level_set("VL53L5CX Api", ESP_LOG_DEBUG);
    esp_log_level_set("SLAMDECK", ESP_LOG_DEBUG);
    esp_log_level_set("SLAMDECK API", ESP_LOG_DEBUG);
    esp_log_level_set("WIFI", ESP_LOG_DEBUG);
    esp_log_level_set("COM", ESP_LOG_DEBUG);
    esp_log_level_set("UART", ESP_LOG_DEBUG);
    esp_log_level_set("ROUTER", ESP_LOG_DEBUG);
    esp_log_level_set("Platform", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "Available esp_get_free_heap_size: %d", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Available esp_get_free_internal_heap_size: %d", esp_get_free_internal_heap_size());


    led_init();
    led_set_state(LED_BLUE, LED_STATE_BLINK_0_25_HZ);
    i2c_init();

    // Initialize esp APP transport before "com"
    espTransportInit();
    #ifndef DISABLED_WIFI_API
        uart_transport_init();
        com_init();
        wifi_init();
        router_init();
        wifi_ctrl();
    #endif

    slamdeck_init();
    slamdeck_api_init();

    //test_vl53l5cx();

    //led_set_state(LED_RED, LED_STATE_ON);
    led_set_state(LED_BLUE, LED_STATE_BLINK_1_HZ);
}
