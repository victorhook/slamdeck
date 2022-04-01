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

typedef enum {
    SLAMDECK_RESULT_OK                   = VL53L5CX_STATUS_OK,
    SLAMDECK_RESULT_TIMEOUT              = VL53L5CX_STATUS_TIMEOUT_ERROR,
    SLAMDECK_RESULT_COMMAND_INVALD       = 2,
    SLAMDECK_RESULT_SENSOR_NOT_ENABLED   = 3,
    SLAMDECK_RESULT_MCU_ERROR            = VL53L5CX_MCU_ERROR,
    SLAMDECK_RESULT_INVALID_PARAM        = VL53L5CX_STATUS_INVALID_PARAM,
    SLAMDECK_RESULT_ERROR                = VL53L5CX_STATUS_ERROR
} slamdeck_result_e;

typedef struct {
    slamdeck_command_e command;
    uint8_t sensor;
    uint8_t data;
} slamdeck_packet_rx_t;

typedef struct {
    uint8_t data[VL53L5CX_RESULT_MAX_BUF_SIZE * 5];     // *5 is for all sensors.
    uint16_t size;
    slamdeck_result_e result;
} slamdeck_packet_tx_t;

typedef struct {
    slamdeck_command_e command : 5;
    uint8_t sensor       : 3;
    uint8_t data               : 8;
} __attribute__((packed)) slamdeck_packet_packed_t;


typedef enum {
    SLAMDECK_SENSOR_ID_MAIN    = 0,
    SLAMDECK_SENSOR_ID_FRONT   = 1,
    SLAMDECK_SENSOR_ID_RIGHT   = 2,
    SLAMDECK_SENSOR_ID_BACK    = 3,
    SLAMDECK_SENSOR_ID_LEFT    = 4,
    SLAMDECK_SENSOR_ID_NOT_SET = 0xff,
} slamdeck_sensor_id_e;


#define START_UP_SLAMDECK_API_BIT BIT1


void slamdeck_api_task();

int available_api_request();

void execute_api_request();

uint8_t slamdeck_sensor_enabled(const slamdeck_sensor_id_e sensor);

VL53L5CX_t* slamdeck_get_sensor(const slamdeck_sensor_id_e sensor);

