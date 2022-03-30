#pragma once


typedef enum {
    SLAMDECK_LED_BLUE,
    SLAMDECK_LED_RED,
    SLAMDECK_LED_GREEN
} slamdeck_led_e;

typedef enum {
    LED_STATE_OFF           = 0,
    LED_STATE_ON            = 1,
    LED_STATE_BLINK_0_25_HZ = 250,
    LED_STATE_BLINK_0_5_HZ  = 500,
    LED_STATE_BLINK_1_HZ    = 1000,
    LED_STATE_BLINK_2_HZ    = 2000
} slamdeck_led_state_e;

typedef struct {
    slamdeck_led_e       led;
    slamdeck_led_state_e state;
} led_state_change_t;


void led_set_state(const led_state_change_t* state_change);

void led_init();

void led_task();
