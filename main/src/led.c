#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "led.h"
#include "driver/gpio.h"
#include "slamdeck.h"


typedef struct {
    gpio_num_t           pin;
    slamdeck_led_state_e state;
    uint8_t              value;
    uint32_t             last_toggle;
} led_t;

#define OFF 0
#define ON  1
#define MIN_DELAY LED_STATE_BLINK_0_25_HZ

static led_t leds[] = {
    {SLAMDECK_GPIO_LED_BLUE, .state=LED_STATE_OFF, .value=OFF, .last_toggle=0},
    {SLAMDECK_GPIO_LED_RED, .state=LED_STATE_OFF, .value=OFF, .last_toggle=0},
    {SLAMDECK_GPIO_LED_GREEN, .state=LED_STATE_OFF, .value=OFF, .last_toggle=0},
};

#define NBR_OF_LEDS sizeof(leds) / sizeof(led_t)

EventGroupHandle_t startUpEventGroup;
static const int START_UP_LEDS = BIT0;


static const char* TAG = "Leds";


static inline esp_err_t led_set(led_t* led, const uint8_t value)
{
    led->value = value;
    return gpio_set_level(led->pin, led->value);
}

static inline esp_err_t led_toggle(led_t* led, const uint32_t time)
{
    led->last_toggle = time;
    return led_set(led, led->value == ON ? OFF : ON);
}

static inline uint8_t time_to_toggle(const led_t* led, uint32_t time)
{
    if (led->state == LED_STATE_ON || led->state == LED_STATE_OFF)
        return 0;
    else
        return (time - led->last_toggle) > led->state;
}

static void led_change_state(const slamdeck_led_e led, const slamdeck_led_state_e state, const uint32_t time)
{
    led_t* led_to_change = &leds[led];
    led_to_change->state = state;
    led_to_change->last_toggle = time;
    if (led_to_change->state == LED_STATE_ON)
        led_to_change->value = ON;
    else
        led_to_change->value = OFF;
}

void led_set_state(const slamdeck_led_e led, const slamdeck_led_state_e state)
{
    led_change_state(led, state, get_current_time());
}

void led_task()
{
    xEventGroupSetBits(startUpEventGroup, START_UP_LEDS);
    ESP_LOGI(TAG, "Starting led");

    for (int i = 0; i < NBR_OF_LEDS; i++) {
        led_t led = leds[i];
        ESP_ERROR_CHECK(gpio_reset_pin(led.pin));
        ESP_ERROR_CHECK(gpio_set_direction(led.pin, GPIO_MODE_OUTPUT));
        led_set_state(i, LED_STATE_OFF);
    }


    while (1) {
        uint32_t time = get_current_time();
        for (int i = 0; i < NBR_OF_LEDS; i++) {
            led_t* led = &leds[i];
            //ESP_LOGI(TAG, "Led: %d, last: %d, diff: %d, state: %d time to toggle: %d", led.pin, led.last_toggle, (time - led.last_toggle), led.state, time_to_toggle(led, time));
            if (time_to_toggle(led, time)) {
                led_toggle(led, time);
                //ESP_LOGI(TAG, "Time: %d, lasttoggle: %d, Time tot oggle pin %d", time, led->last_toggle, led->pin);
            }
        }

        vTaskDelay(MIN_DELAY / portTICK_PERIOD_MS);
    }
}

void led_init()
{
    startUpEventGroup = xEventGroupCreate();
    xEventGroupClearBits(startUpEventGroup, START_UP_LEDS);
    xTaskCreate(led_task, "Led Task", 5000, NULL, 10, NULL);

    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_LEDS,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Initialized OK");
}
