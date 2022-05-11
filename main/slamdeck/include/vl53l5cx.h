#pragma once

#include "driver/gpio.h"
#include "vl53l5cx_api.h"


#define VL53L5CX_RESULT_MAX_BUF_SIZE 128
#define DATA_DISTANCE_MM_MAX_BUFFER_SIZE_U16 64

#define VL53L5CX_STATUS_NOT_ENABLED 2
#define VL53L5CX_STATUS_INIT_FAIL   3

typedef uint8_t  VL53L5CX_status_e;
typedef uint32_t VL53L5CX_integration_time_ms_e;
typedef uint8_t  VL53L5CX_sharpener_percent_e;
typedef uint8_t  VL53L5CX_ranging_frequency_hz_e;
typedef uint8_t  VL53L5CX_power_mode_e;
typedef uint8_t  VL53L5CX_resolution_e;
typedef uint8_t  VL53L5CX_target_order_e;
typedef uint8_t  VL53L5CX_ranging_mode_e;

typedef struct {
    VL53L5CX_integration_time_ms_e   integration_time_ms;
    VL53L5CX_sharpener_percent_e     sharpener_percent;
    VL53L5CX_ranging_frequency_hz_e  ranging_frequency_hz;
    VL53L5CX_resolution_e            resolution;
    VL53L5CX_power_mode_e            power_mode;
    VL53L5CX_target_order_e          target_order;
    VL53L5CX_ranging_mode_e          ranging_mode;
} __attribute__((packed)) VL53L5CX_settings_t;

/*
To have consistent data, the user needs to filter invalid target status. To give a confidence rating, a target with
status 5 is considered as 100 % valid. A status of 6 or 9 can be considered with a confidence value of 50 %. All
other statuses are below 50 % confidence level.
*/
typedef enum {
    TARGET_STATUS_NOT_UPDATED                     = 0,  // Ranging data are not updated
    TARGET_STATUS_SIGNAL_RATE_TOO_LOW_ON_SPAD     = 1,  // Signal rate too low on SPAD array
    TARGET_STATUS_PHASE                           = 2,  // Target phase
    TARGET_STATUS_SIGMA_TOO_HIGH                  = 3,  // Sigma estimator too high
    TARGET_STATUS_TARGET_CONSISTENCY_FAILED       = 4,  // Target consistency failed
    TARGET_STATUS_RANGE_VALID                     = 5,  // Range valid
    TARGET_STATUS_WRAP_AROUND                     = 6,  // Wrap around not performed (Typically the first range)
    TARGET_STATUS_RATE_CONSISTENCY_FAILED         = 7,  // Rate consistency failed
    TARGET_STATUS_SIGNAL_RATE_TOO_LOW_FOR_TARGET  = 8,  // Signal rate too low for the current target
    TARGET_STATUS_RANGE_VALID_BUT_LARGE_PULSE     = 9,  // Range valid with large pulse (may be due to a merged target)
    TARGET_STATUS_RANGE_VALID_BUT_NO_TARGET       = 10, // Range valid, but no target detected at previous range
    TARGET_STATUS_MEASUREMENT_CONSISTENCY_FAILED  = 11, // Measurement consistency failed
    TARGET_STATUS_TARGET_BLURRED                  = 12, // Target blurred by another one, due to sharpener
    TARGET_STATUS_TARGET_OK_BUT_INCONSISTENT_DATA = 13, // Target detected but inconsistent data. Frequently happens for secondary targets.
    TARGET_STATUS_TARGET_NOT_DETECTED             = 255 // No target detected (only if number of target detected is enabled)
} VL53L5CX_target_status_codes_e;


typedef struct {
    gpio_num_t              enable_pin;
    uint8_t                 enable_pin_initialized;
    uint8_t                 data_ready;
    uint8_t                 id;
    uint8_t                 i2c_address;
    uint32_t                last_sample;
    uint64_t                samples;
    uint8_t                 is_ranging;
    VL53L5CX_status_e       status;
    VL53L5CX_settings_t     settings;
    uint16_t                data_distance_mm[DATA_DISTANCE_MM_MAX_BUFFER_SIZE_U16];

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

/* --- Getters --- */
uint16_t VL53L5CX_get_data_size(const VL53L5CX_t* sensor);
uint8_t VL53L5CX_get_i2c_address(VL53L5CX_t* sensor);
uint8_t VL53L5CX_get_power_mode(VL53L5CX_t* sensor);
uint8_t VL53L5CX_get_resolution(VL53L5CX_t* sensor);
uint8_t VL53L5CX_get_ranging_frequency_hz(VL53L5CX_t* sensor);
uint32_t VL53L5CX_get_integration_time_ms(VL53L5CX_t* sensor);
uint8_t VL53L5CX_get_sharpener_percent(VL53L5CX_t* sensor);
uint8_t VL53L5CX_get_target_order(VL53L5CX_t* sensor);
uint8_t VL53L5CX_get_ranging_mode(VL53L5CX_t* sensor);

/* --- Setters --- */
uint8_t VL53L5CX_set_i2c_address(VL53L5CX_t* sensor, const uint8_t address);
uint8_t VL53L5CX_set_power_mode(VL53L5CX_t* sensor, const VL53L5CX_power_mode_e power_mode);
uint8_t VL53L5CX_set_resolution(VL53L5CX_t* sensor, const VL53L5CX_resolution_e resolution);
uint8_t VL53L5CX_set_ranging_frequency_hz(VL53L5CX_t* sensor, const uint8_t frequency_hz);
uint8_t VL53L5CX_set_integration_time_ms(VL53L5CX_t* sensor, const uint32_t integration_time_ms);
uint8_t VL53L5CX_set_sharpener_percent(VL53L5CX_t* sensor, const uint8_t sharpener);
uint8_t VL53L5CX_set_target_order(VL53L5CX_t* sensor, const VL53L5CX_target_order_e target_order);
uint8_t VL53L5CX_set_ranging_mode(VL53L5CX_t* sensor, const VL53L5CX_ranging_mode_e ranging_mode);
