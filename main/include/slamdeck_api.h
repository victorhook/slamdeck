#pragma once

#include "vl53l5cx.h"

typedef struct {
    slamdeck_command_e command;
    uint8_t sensor;
    uint8_t data;
} slamdeck_packet_rx_t;

typedef struct {
    uint8_t data;     // *5 is for all sensors.
    uint16_t size;
    slamdeck_result_e result;
} slamdeck_packet_tx_t;

typedef struct {
    uint8_t  command;
    uint8_t* [VL53L5CX_RESULT_MAX_BUF_SIZE * 5];
} __attribute__((packed)) slamdeck_packet_packed_t;

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

