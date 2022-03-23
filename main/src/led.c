#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"
#include "driver/gpio.h"
#include "slamdeck.h"


#define BLINK_DELAY_MS 500

static const char* TAG = "Leds";


void led_set(gpio_num_t led, uint32_t value)
{
    gpio_set_level(led, value);
}

void led_task()
{
    while (1) {
        led_set(SLAMDECK_GPIO_LED_BLUE, 0);
        led_set(SLAMDECK_GPIO_LED_RED, 0);
        led_set(SLAMDECK_GPIO_LED_GREEN, 0);
        vTaskDelay(BLINK_DELAY_MS/portTICK_PERIOD_MS);
        led_set(SLAMDECK_GPIO_LED_BLUE, 1);
        led_set(SLAMDECK_GPIO_LED_RED, 1);
        led_set(SLAMDECK_GPIO_LED_GREEN, 1);
        vTaskDelay(BLINK_DELAY_MS/portTICK_PERIOD_MS);
    }
}


void led_init()
{
    ESP_ERROR_CHECK(gpio_reset_pin(SLAMDECK_GPIO_LED_BLUE));
    ESP_ERROR_CHECK(gpio_reset_pin(SLAMDECK_GPIO_LED_RED));
    ESP_ERROR_CHECK(gpio_reset_pin(SLAMDECK_GPIO_LED_GREEN));
    ESP_ERROR_CHECK(gpio_set_direction(SLAMDECK_GPIO_LED_BLUE, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(SLAMDECK_GPIO_LED_RED, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(SLAMDECK_GPIO_LED_GREEN, GPIO_MODE_OUTPUT));
    led_set(SLAMDECK_GPIO_LED_BLUE, 0);
    led_set(SLAMDECK_GPIO_LED_RED, 0);
    led_set(SLAMDECK_GPIO_LED_GREEN, 0);
    xTaskCreate(led_task, "Led Task", 1024, NULL, 10, NULL);
    ESP_LOGI(TAG, "Initialized OK");
}
