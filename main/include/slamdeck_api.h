#pragma once

#include "vl53l5cx.h"

typedef struct {
    uint8_t command;
    uint8_t data[sizeof(VL53L5CX_settings_t)];
} slamdeck_packet_rx_t;

typedef struct {
    uint16_t size;
    uint8_t data[VL53L5CX_RESULT_MAX_BUF_SIZE * 5 + 5]; // (Data + status) * 5
} slamdeck_packet_tx_t;

typedef enum {
    SLAMDECK_API_STATE_IDLE,
    SLAMDECK_API_STATE_STREAMING
} slamdeck_api_state_e;

typedef struct {
    slamdeck_api_state_e api_state;
    slamdeck_sensor_id_e sensor;
} slamdeck_api_t;


#define START_UP_SLAMDECK_API_BIT BIT1
#define SLAMDECK_API_TOTAL_COMMANDS 5


void slamdeck_api_stop_streaming();

void slamdeck_api_init();

VL53L5CX_t* slamdeck_get_sensor(const slamdeck_sensor_id_e sensor);

