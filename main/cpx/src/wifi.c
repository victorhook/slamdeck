/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP deck firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "wifi.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_task_wdt.h"

#include "slamdeck.h"
#include "slamdeck_api.h"
#include "led.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "esp_netif.h"

#include "com.h"

static esp_routable_packet_t rxp;
static esp_routable_packet_t txp;

#define MAX_SSID_SIZE (50)
#define MAX_PASSWD_SIZE (50)

static char ssid[MAX_SSID_SIZE];
static char key[MAX_SSID_SIZE];

static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_SOCKET_DISCONNECTED = BIT1;
static EventGroupHandle_t s_wifi_event_group;

static const int START_UP_MAIN_TASK = BIT0;
static const int START_UP_RX_TASK = BIT1;
static const int START_UP_TX_TASK = BIT2;
static const int START_UP_CTRL_TASK = BIT3;
static EventGroupHandle_t startUpEventGroup;

#define WIFI_SOCKET_PORT 5000
#define WIFI_HOST_QUEUE_LENGTH (2)

static xQueueHandle wifiRxQueue;
static xQueueHandle wifiTxQueue;

/* Log printout tag */
static const char *TAG = "WIFI";

/* Socket for receiving WiFi connections */
static int sock = -1;
/* Accepted WiFi connection */
static int conn = -1;

/* WiFi event handler */
static void event_handler(void* handlerArg, esp_event_base_t eventBase, int32_t eventId, void* eventData)
{
  if (eventBase == WIFI_EVENT) {
    switch(eventId) {
      case WIFI_EVENT_STA_START:
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
      case WIFI_EVENT_STA_CONNECTED:
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Connected to access point");
        break;
      case WIFI_EVENT_STA_DISCONNECTED:
        ESP_ERROR_CHECK(esp_wifi_connect());
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG,"Disconnected from access point");
        break;
      case WIFI_EVENT_AP_STACONNECTED:
        {
          wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)eventData;
          ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                    MAC2STR(event->mac),
                    event->aid);
        }
        break;
      case WIFI_EVENT_AP_STADISCONNECTED:
        {
          wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)eventData;
          ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                    MAC2STR(event->mac),
                    event->aid);
        }
        break;
      default:
        ESP_LOGI(TAG, "Unknown Wifi event: %d", eventId);
        // Fall through
        break;
    }
  }

  if (eventBase == IP_EVENT) {
    switch (eventId) {
      case IP_EVENT_STA_GOT_IP:
        {
          ip_event_got_ip_t* event = (ip_event_got_ip_t*)eventData;
          ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));

          wifi_ap_record_t ap_info;
          ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
          ESP_LOGD(TAG, "BSAP MAC is %x:%x:%x:%x:%x:%x",
              ap_info.bssid[0], ap_info.bssid[1], ap_info.bssid[2],
              ap_info.bssid[3], ap_info.bssid[4], ap_info.bssid[5]);
          ESP_LOGI(TAG, "country: %s", ap_info.country.cc);
          ESP_LOGI(TAG, "rssi: %d", ap_info.rssi);
          ESP_LOGI(TAG, "11b: %d, 11g: %d, 11n: %d, lr: %d",
          ap_info.phy_11b, ap_info.phy_11g, ap_info.phy_11n, ap_info.phy_lr);

          xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        break;
      default:
        ESP_LOGI(TAG, "Unknown IP event: %d", eventId);
        // Fall through
        break;
    }
  }
}

/* Initialize WiFi as AP */
static void wifi_init_softap(const char *ssid, const char* key)
{
  esp_netif_t* ap_netif = esp_netif_create_default_wifi_ap();
  assert(ap_netif);

  wifi_config_t wifi_config = {
      .ap = {
          .ssid_len = strlen(ssid),
          .max_connection = 1,
          .authmode = WIFI_AUTH_OPEN},
  };
  strncpy((char *)wifi_config.ap.ssid, ssid, strlen(ssid));
  if (strlen(key) > 0) {
    strncpy((char *)wifi_config.ap.password, key, strlen(key));
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_softap finished");
}

static void wifi_init_sta(const char * ssid, const char * key)
{
  esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_config_t wifi_config;
  memset((void *)&wifi_config, 0, sizeof(wifi_config_t));
  strncpy((char *)wifi_config.sta.ssid, ssid, strlen(ssid));
  ESP_LOGD(TAG, "SSID is %u chars", strlen(ssid));
  strncpy((char *)wifi_config.sta.password, key, strlen(key));
  ESP_LOGD(TAG, "KEY is %u chars", strlen(key));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");

}

static void wifi_ctrl(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_CTRL_TASK);
  while (1) {
    com_receive_wifi_ctrl_blocking(&rxp);
    ESP_LOGD(TAG, "Received ctrl packet from com: %02x", rxp.data[0]);
    switch (rxp.data[0]) {
      case WIFI_CTRL_SET_SSID:
        ESP_LOGD("WIFI", "Should set SSID");
        memcpy(ssid, &rxp.data[1], rxp.dataLength - 1);
        ssid[rxp.dataLength - 1 + 1] = 0;
        ESP_LOGD(TAG, "SSID: %s", ssid);
        // Save to NVS?
        break;
      case WIFI_CTRL_SET_KEY:
        ESP_LOGD("WIFI", "Should set password");
        memcpy(key, &rxp.data[1], rxp.dataLength - 1);
        key[rxp.dataLength - 1 + 1] = 0;
        ESP_LOGD(TAG, "KEY: %s", key);
        // Save to NVS?
        break;
      case WIFI_CTRL_WIFI_CONNECT:
        ESP_LOGD("WIFI", "Should connect");

        if (strlen(ssid) > 0) {
          if (rxp.data[1] == 0) {
            wifi_init_sta(ssid, key);
          } else {
            wifi_init_softap(ssid, key);
          }
        } else {
          ESP_LOGW(TAG, "No SSID set, cannot start wifi");
        }

        break;
      default:
        ESP_LOGD(TAG, "Failed to recognize wifi ctrl packet %02x", rxp.data[0]);
    }
  }
}

void wifi_bind_socket() {
  char addr_str[128];
  int addr_family;
  int ip_protocol;
  struct sockaddr_in destAddr;
  destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(WIFI_SOCKET_PORT);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;
  inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
    sock = socket(addr_family, SOCK_STREAM, ip_protocol);
  if (sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket created");

  int err = bind(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket binded");

  err = listen(sock, 1);
  if (err != 0) {
    ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket listening to port %d", WIFI_SOCKET_PORT);
}

typedef enum {
    SOCKET_CONNECTION_RESULT_OK     = 0,
    SOCKET_CONNECTION_RESULT_NOT_OK = 1
} socket_connection_result_e;

uint8_t wifi_wait_for_socket_connected() {
  ESP_LOGI(TAG, "Waiting for connection");
  struct sockaddr sourceAddr;
  uint addrLen = sizeof(sourceAddr);
  conn = accept(sock, (struct sockaddr *)&sourceAddr, &addrLen);
  socket_connection_result_e result = SOCKET_CONNECTION_RESULT_OK;

  if (conn < 0) {
    ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
    result = SOCKET_CONNECTION_RESULT_NOT_OK;
  }
  ESP_LOGI(TAG, "Connection accepted");
  return result;
}

void wifi_wait_for_disconnect() {
	xEventGroupWaitBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED, pdTRUE, pdFALSE, portMAX_DELAY);

  // Reset rx and tx queues.
  xQueueReset(wifiRxQueue);
  xQueueReset(wifiTxQueue);

  // Must close sockets properly.
	shutdown(conn, 0);
	close(conn);

	// Notify app that we're disconnected
	slamdeck_api_stop_streaming();
}

static void wifi_task(void *pvParameters) {

  s_wifi_event_group = xEventGroupCreate();

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL, NULL);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  uint8_t mac[6];
  ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
  ESP_LOGD(TAG, "AP MAC is %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_STA, mac));
  ESP_LOGD(TAG, "STA MAC is %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  wifi_bind_socket();

  xEventGroupSetBits(startUpEventGroup, START_UP_MAIN_TASK);
  while (1) {
    wifi_wait_for_socket_connected();
    ESP_LOGI(TAG, "Client connected");
    led_set_state(LED_RED, LED_STATE_ON);

    // Notify that we're connected
    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_WIFI_CTRL, &txp.route);
    txp.data[0] = WIFI_CTRL_STATUS_CLIENT_CONNECTED;
    txp.data[1] = 1;    // connected
    txp.dataLength = 2;
    espAppSendToRouterBlocking(&txp);

    wifi_wait_for_disconnect();
    ESP_LOGI(TAG, "Client disconnected");
    led_set_state(LED_RED, LED_STATE_OFF);

    // Notify that we're disconnected
    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_WIFI_CTRL, &txp.route);
    txp.data[0] = WIFI_CTRL_STATUS_CLIENT_CONNECTED;
    txp.data[1] = 0;    // disconnected
    txp.dataLength = 2;
    espAppSendToRouterBlocking(&txp);
  }
}

void wifi_send_packet(const char * buffer, size_t size) {
  if (conn != -1) {
    ESP_LOGD(TAG, "Sending WiFi packet of size %u", size);
    int err = send(conn, buffer, size, 0);
    if (err < 0) {
      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
      conn = -1;
      xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
    }
  } else {
    ESP_LOGD(TAG, "Connection is -1");
  }
}

static void wifi_sending_task(void *pvParameters) {
  static WifiTransportPacket_t txp_wifi;
  static CPXRoutablePacket_t qPacket;

  xEventGroupSetBits(startUpEventGroup, START_UP_TX_TASK);
  while (1) {
    xQueueReceive(wifiTxQueue, &qPacket, portMAX_DELAY);
    txp_wifi.payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;

    cpxRouteToPacked(&qPacket.route, &txp_wifi.routablePayload.route);

    memcpy(txp_wifi.routablePayload.data, qPacket.data, qPacket.dataLength);

    wifi_send_packet((const char *)&txp_wifi, txp_wifi.payloadLength + 2);
  }
}

static void wifi_receiving_task(void *pvParameters) {
	static WifiTransportPacket_t rxp_wifi;
	int len;

	char buf[400];
	uint64_t t0 = esp_timer_get_time();

	xEventGroupSetBits(startUpEventGroup, START_UP_RX_TASK);

	while (1) {
        uint64_t now = esp_timer_get_time();
        uint64_t dt = (now - t0) / 1000;
        uint8_t seconds = 3;

        if (dt > 1000*seconds) {
            t0 = now;
            //vTaskGetRunTimeStats(buf);
            //printf(buf);
            //printf("\n");
		  }

		len = recv(conn, &rxp_wifi, 2, 0);
		if (len > 0) {
			ESP_LOGD(TAG, "Wire data length %i", rxp_wifi.payloadLength);

			int totalRxLen = 0;
			do {
				len = recv(conn, &rxp_wifi.payload[totalRxLen], rxp_wifi.payloadLength - totalRxLen, 0);
				ESP_LOGD(TAG, "Read %i bytes", len);
				totalRxLen += len;
			} while ((len > 0) && (totalRxLen < rxp_wifi.payloadLength));
			//ESP_LOG_BUFFER_HEX_LEVEL(TAG, &rxp_wifi, 10, ESP_LOG_DEBUG);
			xQueueSend(wifiRxQueue, &rxp_wifi, portMAX_DELAY);
		} else if (len == 0) {
			xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
		} else {
			vTaskDelay(50/portTICK_PERIOD_MS);
		}
	}
}

void wifi_transport_send(const CPXRoutablePacket_t* packet) {
  assert(packet->dataLength <= WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
  xQueueSend(wifiTxQueue, packet, portMAX_DELAY);
}

void wifi_transport_receive(CPXRoutablePacket_t* packet) {
  // Not reentrant safe. Assuming only one task is popping the queue
  static WifiTransportPacket_t qPacket;
  xQueueReceive(wifiRxQueue, &qPacket, portMAX_DELAY);

  packet->dataLength = qPacket.payloadLength - CPX_ROUTING_PACKED_SIZE;
  CPXRoutingPacked_t *p = &qPacket.routablePayload.route;
  ESP_LOGD(TAG, "Source: %02x, dest: %02x, function: %02x, last: %02x", p->source, p->destination, p->function, p->lastPacket);

  cpxPackedToRoute(&qPacket.routablePayload.route, &packet->route);

  memcpy(packet->data, qPacket.routablePayload.data, packet->dataLength);
}

void wifi_init() {
  esp_netif_init();

  led_set_state(LED_RED, LED_STATE_BLINK_0_25_HZ);

  s_wifi_event_group = xEventGroupCreate();

  wifiRxQueue = xQueueCreate(WIFI_HOST_QUEUE_LENGTH, sizeof(WifiTransportPacket_t));
  wifiTxQueue = xQueueCreate(WIFI_HOST_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

  startUpEventGroup = xEventGroupCreate();
  xEventGroupClearBits(startUpEventGroup, START_UP_MAIN_TASK | START_UP_RX_TASK | START_UP_TX_TASK | START_UP_CTRL_TASK);
  xTaskCreate(wifi_task, "WiFi TASK", 5000, NULL, 2, NULL);
  xTaskCreate(wifi_sending_task, "WiFi TX", 5000, NULL, 2, NULL);
  xTaskCreate(wifi_receiving_task, "WiFi RX", 5000, NULL, 2, NULL);
  ESP_LOGI(TAG, "Waiting for main, RX and TX tasks to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_MAIN_TASK | START_UP_RX_TASK | START_UP_TX_TASK,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  xTaskCreate(wifi_ctrl, "WiFi CTRL", 5000, NULL, 2, NULL);
  ESP_LOGI(TAG, "Waiting for CTRL task to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_CTRL_TASK,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  //led_set_state(LED_RED, LED_STATE_ON);
  ESP_LOGI("WIFI", "Wifi initialized");
}
