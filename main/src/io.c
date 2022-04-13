#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "io.h"
#include "slamdeck.h"
#include "driver/gpio.h"


static EventGroupHandle_t startUpEventGroup;
static const int START_UP_IO = BIT0;

static const char* TAG = "IO";

static void init_pins()
{
    gpio_reset_pin(SLAMDECK_GPIO_CF_IO_1);
    gpio_set_direction(SLAMDECK_GPIO_CF_IO_1, GPIO_MODE_INPUT);

    gpio_reset_pin(SLAMDECK_GPIO_CF_IO_2);
    gpio_set_direction(SLAMDECK_GPIO_CF_IO_2, GPIO_MODE_INPUT);

    gpio_reset_pin(SLAMDECK_GPIO_CF_IO_3);
    gpio_set_direction(SLAMDECK_GPIO_CF_IO_3, GPIO_MODE_INPUT);

    gpio_reset_pin(SLAMDECK_GPIO_CF_IO_3);
    gpio_set_direction(SLAMDECK_GPIO_CF_IO_3, GPIO_MODE_INPUT);

    // We need to pull these high, otherwise the Crazyflie seems to have issues with
    // the i2c bus during startup sometimes.
    gpio_reset_pin(SLAMDECK_I2C_SCL_CF);
    gpio_set_pull_mode(SLAMDECK_I2C_SCL_CF, GPIO_PULLUP_ENABLE);
    gpio_reset_pin(SLAMDECK_I2C_SDA_CF);
    gpio_set_pull_mode(SLAMDECK_I2C_SDA_CF, GPIO_PULLUP_ENABLE);
}


void io_task()
{
    xEventGroupSetBits(startUpEventGroup, START_UP_IO);
    init_pins();

    while (1) {
        slamdeck.gpio_cf_io_1 = gpio_get_level(SLAMDECK_GPIO_CF_IO_1);
        slamdeck.gpio_cf_io_2 = gpio_get_level(SLAMDECK_GPIO_CF_IO_2);
        slamdeck.gpio_cf_io_3 = gpio_get_level(SLAMDECK_GPIO_CF_IO_3);
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
}

void io_init()
{
    startUpEventGroup = xEventGroupCreate();
    xEventGroupClearBits(startUpEventGroup, START_UP_IO);
    xTaskCreate(io_task, "IO", 5000, NULL, 1, NULL);

    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_IO,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Initialized");
}
