#pragma once


typedef enum {
    LED_BLUE  = 0,
    LED_RED   = 1,
    LED_GREEN = 2
} slamdeck_led_e;

typedef enum {
    LED_STATE_OFF           = 0,
    LED_STATE_ON            = 1,
    LED_STATE_BLINK_0_25_HZ = 250,
    LED_STATE_BLINK_0_5_HZ  = 500,
    LED_STATE_BLINK_1_HZ    = 1000,
    LED_STATE_BLINK_2_HZ    = 2000
} slamdeck_led_state_e;


void led_set_state(slamdeck_led_e led, const slamdeck_led_state_e state);

void led_init();

void led_task();
