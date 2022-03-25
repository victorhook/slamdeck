#pragma once

#include "driver/gpio.h"


typedef enum {
    RESULT_OK,
    RESULT_FAIL
} VL53L5CX_result_e;

typedef enum {
    SENSOR_STATUS_FAILED,
    SENSOR_STATUS_INITIALIZING,
    SENSOR_STATUS_INITIALIZED,
    SENSOR_INITIALIZATION_FAILED,
    SENSOR_STATUS_ENABLED,
    SENSOR_STATUS_OK
} VL53L5CX_status_e;

typedef enum {
    POWER_MODE_SLEEP  = 0,
    POWER_MODE_WAKEUP = 1
} VL53L5CX_power_mode_e;

typedef enum {
    RESOLUTION_4X4 = 16,
    RESOLUTION_8X8 = 64
} VL53L5CX_resolution_e;

typedef enum {
    TARGET_ORDER_CLOSEST   = 1,
    TARGET_ORDER_STRONGEST = 2
} VL53L5CX_target_order_e;

typedef enum {
    RANGING_MODE_CONTINUOUS = 1,
    RANGING_MODE_AUTONOMOUS = 3
} VL53L5CX_ranging_mode_e;

typedef enum {
    ID_MAIN  = 1,
    ID_FRONT = 2,
    ID_RIGHT = 3,
    ID_BACK  = 4,
    ID_LEFT  = 5
} VL53L5CX_id_e;

uint8_t VL53L5CX_init(VL53L5CX_id_e sensor, gpio_num_t enable_pin);
uint8_t VL53L5CX_set_enable(VL53L5CX_id_e sensor, const uint8_t is_enabled);
uint8_t VL53L5CX_set_i2c_address(VL53L5CX_id_e sensor, const uint8_t address);
uint8_t VL53L5CX_set_power_mode(VL53L5CX_id_e sensor, const VL53L5CX_power_mode_e power_mode);
uint8_t VL53L5CX_set_resolution(VL53L5CX_id_e sensor, const VL53L5CX_resolution_e resolution);
uint8_t VL53L5CX_set_ranging_frequency_hz(VL53L5CX_id_e sensor, const uint8_t frequency_hz);
uint8_t VL53L5CX_set_integration_time_ms(VL53L5CX_id_e sensor, const uint32_t integration_time_ms);
uint8_t VL53L5CX_set_sharpener_percent(VL53L5CX_id_e sensor, const uint8_t sharpener);
uint8_t VL53L5CX_set_target_order(VL53L5CX_id_e sensor, const VL53L5CX_target_order_e target_order);
uint8_t VL53L5CX_set_ranging_mode(VL53L5CX_id_e sensor, const VL53L5CX_ranging_mode_e ranging_mode);
