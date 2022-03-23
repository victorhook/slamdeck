#include "tests_slamdeck.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "hal/uart_types.h"
#include "driver/uart.h"

#define UART_BUF_SIZE 1024


static void uart_echo_task(const uint32_t baud, const uart_port_t uart_port,
                           const int tx_pin, const int rx_pin)
{
    uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_port, UART_BUF_SIZE*2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_port, tx_pin, rx_pin, 0, 0));

    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    //uint8_t data[UART_BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(uart_port, data, UART_BUF_SIZE, 20 / portTICK_RATE_MS);
        uart_write_bytes(uart_port, (const char *) data, len);
    }
}

static void t0()
{
    uart_echo_task(115200, 0, 43, 44);
}

void test_uart0()
{
    xTaskCreate(t0, "uart_echo_task", 1024, NULL, 10, NULL);
}

void test_uart1()
{

}

void test_button()
{

}

void test_leds()
{

}
