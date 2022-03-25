#include "platform.h"
#include "driver/i2c.h"
#include "slamdeck.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#define MASTER_TIMEOUT 500
#define I2C_BUS        SLAMDECK_I2C_BUS_MASTER
#define ACK_CHECK_EN         0x01
#define MASTER_WRITE   0
#define MASTER_READ    1

static const char* TAG = "Platform";

//#define DO_DEBUG
#ifdef DO_DEBUG
#define check_err(res) printf("Result: %s\n", esp_err_to_name(res))
#else
#define check_err(res)
#endif

#define I2C_CMD_LINK_BUFF_SIZE 1024
static uint8_t i2c_cmd_link_buf[I2C_CMD_LINK_BUFF_SIZE];


static inline esp_err_t i2c_write_reg16_to_device(i2c_cmd_handle_t cmd, const uint8_t address, const uint16_t reg)
{
	uint8_t data[] = {
		(address << 1) | MASTER_WRITE,
		(uint8_t) (reg >> 8),
		reg & 0xff};
	return i2c_master_write(cmd, data, 3, ACK_CHECK_EN);
}

static esp_err_t i2c_master_read_from_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
	#ifdef DO_DEBUG
		printf("-- [%02x] Read %d bytes from register %04x --\n", address, size, reg);
	#endif

	i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(i2c_cmd_link_buf, I2C_CMD_LINK_BUFF_SIZE);
	uint8_t header[] = {
		(address << 1) | I2C_MASTER_WRITE,
		(uint8_t) (reg >> 8),
		reg & 0xff
    };

    i2c_master_start(cmd);
	i2c_master_write(cmd, header, 3, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t result = i2c_master_cmd_begin(I2C_BUS, cmd, MASTER_TIMEOUT/portTICK_PERIOD_MS);
	i2c_cmd_link_delete_static(cmd);
	check_err(result);
	if (result != ESP_OK) {
		check_err(result);
		return result;
	}

	#ifdef DO_DEBUG
		fflush(stdout);
		for (uint32_t i = 0; i < size; i++)
			printf("%02x ", buf[i]);;
		printf("\n");
		printf("----------------------------------\n");
	#endif

	result = i2c_master_read_from_device(I2C_BUS, address, buf, size, MASTER_TIMEOUT/portTICK_PERIOD_MS);
	check_err(result);
	return result;
}

static esp_err_t i2c_master_write_to_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
	#ifdef DO_DEBUG
		printf("-- [%02x] Write %d bytes to register %04x --\n", address, size, reg);
    #endif
	i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(i2c_cmd_link_buf, I2C_CMD_LINK_BUFF_SIZE);
	uint8_t header[] = {
		(address << 1) | I2C_MASTER_WRITE,
		(uint8_t) (reg >> 8),
		reg & 0xff
    };

    i2c_master_start(cmd);
	i2c_master_write(cmd, header, 3, ACK_CHECK_EN);
	i2c_master_write(cmd, buf, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_BUS, cmd, MASTER_TIMEOUT/portTICK_RATE_MS);
    i2c_cmd_link_delete_static(cmd);
	check_err(result);

	#ifdef DO_DEBUG
		fflush(stdout);
		for (uint32_t i = 0; i < size; i++)
			printf("%02x \n", buf[i]);;
		printf("\n");
		printf("----------------------------------\n");
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
	for (i = 0; i < size; i = i + 4)
	{
		tmp = (buffer[i] << 24) | (buffer[i + 1] << 16) | (buffer[i + 2] << 8) | (buffer[i + 3]);
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
