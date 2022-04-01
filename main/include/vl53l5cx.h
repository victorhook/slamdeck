#pragma once

#include "driver/gpio.h"
#include "vl53l5cx_api.h"


#define VL53L5CX_RESULT_MAX_BUF_SIZE 128

typedef enum {
    RESULT_OK,
    RESULT_FAIL
} VL53L5CX_result_e;

typedef enum {
    SENSOR_STATUS_OK           = 0,
    SENSOR_STATUS_INITIALIZING = 1,
    SENSOR_STATUS_FAILED       = 2
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

typedef struct {
    gpio_num_t              enable_pin;
    uint8_t                 enable_pin_initialized;
    uint8_t                 data_ready;
    uint8_t                 id;
    uint8_t                 i2c_address;
    uint8_t                 disabled;
    uint32_t                last_sample;
    uint64_t                samples;
    uint8_t                 is_ranging;
    uint32_t                integration_time_ms;
    uint8_t                 sharpener_percent;
    uint8_t                 ranging_frequency_hz;
    VL53L5CX_status_e       status;
    VL53L5CX_resolution_e   resolution;
    VL53L5CX_power_mode_e   power_mode;
    VL53L5CX_target_order_e target_order;
    VL53L5CX_ranging_mode_e ranging_mode;

    // ST's driver types
    VL53L5CX_Configuration  config;
    VL53L5CX_ResultsData    result;
} VL53L5CX_t;


/* --- Control --- */
VL53L5CX_status_e VL53L5CX_init(VL53L5CX_t* sensor);
void VL53L5CX_enable(VL53L5CX_t* sensor);
void VL53L5CX_disable(VL53L5CX_t* sensor);
void VL53L5CX_init_gpio(VL53L5CX_t* sensor);
/* Returns 0 is sensor is alive, otherwise an error code */
uint8_t VL53L5CX_is_alive(VL53L5CX_t* sensor);
uint8_t VL53L5CX_start(VL53L5CX_t* sensor);
uint8_t VL53L5CX_stop(VL53L5CX_t* sensor);
uint8_t VL53L5CX_collect_data(VL53L5CX_t* sensor);
uint8_t VL53L5CX_data_ready(VL53L5CX_t* sensor);
const VL53L5CX_ResultsData* VL53L5CX_get_data(VL53L5CX_t* sensor);
/* --- Setters --- */
uint8_t VL53L5CX_set_i2c_address(VL53L5CX_t* sensor, const uint8_t address);
uint8_t VL53L5CX_set_power_mode(VL53L5CX_t* sensor, const VL53L5CX_power_mode_e power_mode);
uint8_t VL53L5CX_set_resolution(VL53L5CX_t* sensor, const VL53L5CX_resolution_e resolution);
uint8_t VL53L5CX_set_ranging_frequency_hz(VL53L5CX_t* sensor, const uint8_t frequency_hz);
uint8_t VL53L5CX_set_integration_time_ms(VL53L5CX_t* sensor, const uint32_t integration_time_ms);
uint8_t VL53L5CX_set_sharpener_percent(VL53L5CX_t* sensor, const uint8_t sharpener);
uint8_t VL53L5CX_set_target_order(VL53L5CX_t* sensor, const VL53L5CX_target_order_e target_order);
uint8_t VL53L5CX_set_ranging_mode(VL53L5CX_t* sensor, const VL53L5CX_ranging_mode_e ranging_mode);
