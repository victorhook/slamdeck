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

#define SLAMDECK_API_TOTAL_COMMANDS 5

void slamdeck_api_stop_streaming();

void slamdeck_api_init();
