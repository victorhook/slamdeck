#pragma once

#include "hal/gpio_types.h"
#include "vl53l5cx.h"


/* --- GPIO definitions --- */
#define SLAMDECK_GPIO_LED_BLUE      GPIO_NUM_1
#define SLAMDECK_GPIO_LED_RED       GPIO_NUM_2
#define SLAMDECK_GPIO_LED_GREEN     GPIO_NUM_4
#define SLAMDECK_GPIO_ROM_BOOT_SRC  GPIO_NUM_46
#define SLAMDECK_GPIO_BOOT_BUTTON   GPIO_NUM_0
#define SLAMDECK_GPIO_SENSOR_MAIN   GPIO_NUM_15   // Onboard main PCB
#define SLAMDECK_GPIO_SENSOR_FRONT  GPIO_NUM_33   // Front
#define SLAMDECK_GPIO_SENSOR_RIGHT  GPIO_NUM_11   // Right
#define SLAMDECK_GPIO_SENSOR_BACK   GPIO_NUM_5    // Back
#define SLAMDECK_GPIO_SENSOR_LEFT   GPIO_NUM_40   // Left

#define SLAMDECK_I2C_SCL_CF         GPIO_NUM_7
#define SLAMDECK_I2C_SDA_CF         GPIO_NUM_8
#define SLAMDECK_I2C_SCL            GPIO_NUM_12
#define SLAMDECK_I2C_SDA            GPIO_NUM_13

#define SLAMDECK_UART_CF            UART_NUM_1
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
#define SLAMDECK_I2C_MASTER_FREQ_HZ 1000000
#define SLAMDECK_I2C_ADDRESS_CF      0x99

#define get_current_time() ((xTaskGetTickCount() / portTICK_PERIOD_MS) * 100)

#define SLAMDECK_SENSOR_HANDLING_CORE 1
#define SLAMDECK_NOT_SENSOR_HANDLING_CORE 0

#define SLAMDECK_NBR_OF_SENSORS 5

typedef enum {
    SLAMDECK_SENSOR_ID_MAIN    = 0,
    SLAMDECK_SENSOR_ID_FRONT   = 1,
    SLAMDECK_SENSOR_ID_RIGHT   = 2,
    SLAMDECK_SENSOR_ID_BACK    = 3,
    SLAMDECK_SENSOR_ID_LEFT    = 4,
    SLAMDECK_SENSOR_ID_ALL     = 5,
    SLAMDECK_SENSOR_ID_NOT_SET = 255,
} slamdeck_sensor_id_e;

typedef enum {
    SLAMDECK_COMMAND_GET_DATA            = 0,
    SLAMDECK_COMMAND_GET_SETTINGS        = 1,
    SLAMDECK_COMMAND_SET_SETTINGS        = 2,
    SLAMDECK_COMMAND_START_STREAMING     = 3,
    SLAMDECK_COMMAND_STOP_STREAMING      = 4,
} slamdeck_command_e;

typedef enum {
    SLAMDECK_RESULT_OK             = 0,
    SLAMDECK_RESULT_ERROR          = 1,
    SLAMDECK_RESULT_COMMAND_INVALD = 2
} slamdeck_result_e;

typedef enum {
    SLAMDECK_STATE_IDLE      = 0,
    SLAMDECK_STATE_STREAMING = 1,
    SLAMDECK_STATE_SLEEP     = 2
} slamdeck_state_e;

typedef struct {
    slamdeck_sensor_id_e sensor;
    slamdeck_command_e   command;
    uint8_t              data;
} set_request_t;

//#define DISABLED_WIFI_API

/* --- Functions --- */
void slamdeck_init();
void slamdeck_set_sensor_settings(const VL53L5CX_settings_t* settings, VL53L5CX_status_e status[SLAMDECK_NBR_OF_SENSORS]);
void slamdeck_get_sensor_settings(VL53L5CX_settings_t* settings, VL53L5CX_status_e status[SLAMDECK_NBR_OF_SENSORS]);

/* Gets the data (in mm) from the sensors and puts them into buf.
   The status of each sensor is stored in the status array.
   Returns the size of all the data.
*/
uint16_t slamdeck_get_sensor_data(uint8_t* buf, VL53L5CX_status_e status[SLAMDECK_NBR_OF_SENSORS]);

