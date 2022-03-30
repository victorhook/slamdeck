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

static led_t leds[] = {
    {SLAMDECK_GPIO_LED_BLUE, .state=LED_STATE_OFF, .value=OFF, .last_toggle=0},
    {SLAMDECK_GPIO_LED_RED, .state=LED_STATE_OFF, .value=OFF, .last_toggle=0},
    {SLAMDECK_GPIO_LED_GREEN, .state=LED_STATE_OFF, .value=OFF, .last_toggle=0},
};

#define NBR_OF_LEDS sizeof(leds) / sizeof(led_t)

EventGroupHandle_t startUpEventGroup;
static const int START_UP_LEDS = BIT0;

static xQueueHandle stateChangeQueue;

static const char* TAG = "Leds";


static inline esp_err_t led_set(led_t led, const uint8_t value)
{
    led.value = value;
    return gpio_set_level(led.pin, led.value);
}

static inline esp_err_t led_toggle(led_t led)
{
    return led_set(led, led.value == ON ? OFF : ON);
}

static inline uint8_t time_to_toggle(led_t led, uint32_t time)
{
    if (led.state == LED_STATE_ON || led.state == LED_STATE_OFF)
        return 0;
    else
        return (time - led.last_toggle) > led.state;
}

static void led_change_state(const led_state_change_t* state_change, const uint32_t time)
{
    led_t led = leds[state_change->led];
    led.state = state_change->state;
    led.last_toggle = time;
    if (led.state == LED_STATE_ON)
        led.value = ON;
    else
        led.value = OFF;
}

void led_set_state(const led_state_change_t* state_change)
{
    xQueueSend(stateChangeQueue, state_change, 50/portTICK_PERIOD_MS);
}

void led_task()
{
    xEventGroupSetBits(startUpEventGroup, START_UP_LEDS);
    led_t led;

    for (int i = 0; i < NBR_OF_LEDS; i++) {
        led = leds[i];
        ESP_ERROR_CHECK(gpio_reset_pin(led.pin));
        ESP_ERROR_CHECK(gpio_set_direction(led.pin, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(led_set(led, ON));
    }

    uint32_t time;
    led_state_change_t state_change;

    while (1) {
        time = xTaskGetTickCount() / portTICK_PERIOD_MS;
        for (int i = 0; i < NBR_OF_LEDS; i++) {
            led = leds[i];
            if (time_to_toggle(led, time))
                led_toggle(led);
        }

        if (xQueueReceive(stateChangeQueue, &state_change, 50/portTICK_PERIOD_MS) == pdTRUE)
            led_change_state(&state_change, time);

    }
}

void led_init()
{
    stateChangeQueue = xQueueCreate(3, sizeof(led_state_change_t));

    xEventGroupClearBits(startUpEventGroup, START_UP_LEDS);
    xTaskCreate(led_task, "Led Task", 1024, NULL, 10, NULL);

    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_LEDS,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Initialized OK");
}
