#pragma once

#include "vl53l5cx.h"

typedef enum {
    SLAMDECK_GET_SENSOR_STATUS = 0,
    SLAMDECK_GET_DATA = 1,
    SLAMDECK_GET_I2C_ADDRESS = 2,
    SLAMDECK_SET_I2C_ADDRESS = 3,
    SLAMDECK_GET_POWER_MODE = 4,
    SLAMDECK_SET_POWER_MODE = 5,
    SLAMDECK_GET_RESOLUTION = 6,
    SLAMDECK_SET_RESOLUTION = 7,
    SLAMDECK_GET_RANGING_FREQUENCY_HZ = 8,
    SLAMDECK_SET_RANGING_FREQUENCY_HZ = 9,
    SLAMDECK_GET_INTEGRATION_TIME_MS = 10,
    SLAMDECK_SET_INTEGRATION_TIME_MS = 11,
    SLAMDECK_GET_SHARPENER_PERCENT = 12,
    SLAMDECK_SET_SHARPENER_PERCENT = 13,
    SLAMDECK_GET_TARGET_ORDER = 14,
    SLAMDECK_SET_TARGET_ORDER = 15,
    SLAMDECK_GET_RANGING_MODE = 16,
    SLAMDECK_SET_RANGING_MODE = 17
} slamdeck_command_e;

typedef struct {
    slamdeck_command_e command;
    VL53L5CX_id_e sensor;
    uint8_t data;
} slamdeck_packet_rx_t;

typedef struct {
    uint8_t data[VL53L5CX_RESULT_MAX_BUF_SIZE * 5];     // *5 is for all sensors.
    uint16_t size;
} slamdeck_packet_tx_t;

typedef struct {
    slamdeck_command_e command : 5;
    VL53L5CX_id_e sensor       : 3;
    uint8_t data               : 8;
} __attribute__((packed)) slamdeck_packet_packed_t;



