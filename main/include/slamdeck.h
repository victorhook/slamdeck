#pragma once

#include "hal/gpio_types.h"


/* --- GPIO definitions --- */
#define SLAMDECK_GPIO_LED_BLUE      GPIO_NUM_1
#define SLAMDECK_GPIO_LED_RED       GPIO_NUM_2
#define SLAMDECK_GPIO_LED_GREEN     GPIO_NUM_4
#define SLAMDECK_GPIO_ROM_BOOT_SRC  GPIO_NUM_46
#define SLAMDECK_GPIO_BOOT_BUTTON   GPIO_NUM_0
#define SLAMDECK_GPIO_SENSOR_1      GPIO_NUM_15   // Onboard main PCB
#define SLAMDECK_GPIO_SENSOR_2      GPIO_NUM_33   // Front
#define SLAMDECK_GPIO_SENSOR_3      GPIO_NUM_11   // Right
#define SLAMDECK_GPIO_SENSOR_4      GPIO_NUM_5    // Back
#define SLAMDECK_GPIO_SENSOR_5      GPIO_NUM_40   // Left

#define SLAMDECK_I2C_SCL_CF         GPIO_NUM_7
#define SLAMDECK_I2C_SDA_CF         GPIO_NUM_8
#define SLAMDECK_I2C_SCL            GPIO_NUM_12
#define SLAMDECK_I2C_SDA            GPIO_NUM_13

#define SLAMDECK_UART_TX1           GPIO_NUM_17
#define SLAMDECK_UART_RX1           GPIO_NUM_18
#define SLAMDECK_UART_TX0           GPIO_NUM_43
#define SLAMDECK_UART_RX0           GPIO_NUM_44

#define SLAMDECK_SPI_MOSI           GPIO_NUM_35
#define SLAMDECK_SPI_SCK            GPIO_NUM_36
#define SLAMDECK_SPI_MISO           GPIO_NUM_37
#define SLAMDECK_SPI_CS             GPIO_NUM_41

#define SLAMDECK_I2C_BUS_MASTER     I2C_NUM_0
#define SLAMDECK_I2C_BUS_CF         I2C_NUM_1
#define SLAMDECK_I2C_MASTER_FREQ_HZ 400000
#define SLAMDECK_I2C_ADDRESS_CF      0x99


/* --- Functions --- */
uint8_t slamdeck_init();
