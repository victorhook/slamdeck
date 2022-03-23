#include "platform.h"
#include "driver/i2c.h"
#include "slamdeck.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#define MASTER_TIMEOUT 100
#define I2C_BUS        SLAMDECK_I2C_BUS_MASTER
#define ACK_EN         true
#define MASTER_WRITE   0
#define MASTER_READ    1

static const char* TAG = "Platform";

#define DEBUG 1
#ifdef DEBUG
#define check_err(res) ESP_LOGI(TAG, "Result: %s", esp_err_to_name(res))
#else
#define check_err(res)
#endif

static inline esp_err_t i2c_write_reg16_to_device(i2c_cmd_handle_t cmd, const uint8_t address, const uint16_t reg)
{
	i2c_master_write_byte(cmd, (address << 1) | MASTER_WRITE, ACK_EN);
    i2c_master_write_byte(cmd, reg >> 8, ACK_EN);
    return i2c_master_write_byte(cmd, reg & 0xff, ACK_EN);
}

static esp_err_t i2c_master_read_from_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	check_err(i2c_write_reg16_to_device(cmd, address, reg));
	i2c_master_stop(cmd);
	esp_err_t result = i2c_master_cmd_begin(I2C_BUS, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (result != ESP_OK) {
		check_err(result);
		return result;
	}

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    check_err(i2c_master_write_byte(cmd, (address << 1) | MASTER_READ, ACK_EN));
    check_err(i2c_master_read(cmd, buf, size, ACK_EN));
	i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_BUS, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
	check_err(result);

	#ifdef DEBUG
		fflush(stdout);
		printf("Read %d bytes at address %04x: ", size, address);
		for (uint32_t i = 0; i < size; i++)
			printf("%02x ", buf[i]);;
		printf("\n");
	#endif

	return result;
}

static esp_err_t i2c_master_write_to_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_write_reg16_to_device(cmd, address, reg);
	i2c_master_write(cmd, buf, size, ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_BUS, cmd, MASTER_TIMEOUT/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	check_err(result);

	#ifdef DEBUG
		fflush(stdout);
		printf("Write %d bytes to address %04x: ", size, address);
		for (uint32_t i = 0; i < size; i++)
			printf("%02x ", buf[i]);;
		printf("\n");
	#endif

    return result;
}


uint8_t RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
    return i2c_master_read_from_reg16(p_platform->address, RegisterAdress, p_value, 1);
}

uint8_t WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
    return i2c_master_write_to_reg16(p_platform->address, RegisterAdress, &value, 1);
}

uint8_t WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
    return i2c_master_write_to_reg16(p_platform->address, RegisterAdress, p_values, size);
}

uint8_t RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	return i2c_master_read_from_reg16(p_platform->address, RegisterAdress, p_values, size);
}

uint8_t Reset_Sensor(
		VL53L5CX_Platform *p_platform)
{
	return 0;
}

void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4)
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs)
{
    vTaskDelay(TimeMs / portTICK_RATE_MS);
	return 0;
}
