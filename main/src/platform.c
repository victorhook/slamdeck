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

//#define DO_DEBUG
#ifdef DEBUG
#define check_err(res) ESP_LOGI(TAG, "Result: %s", esp_err_to_name(res))
#define check_err(res)
#else
#define check_err(res)
#endif

static inline esp_err_t i2c_write_reg16_to_device(i2c_cmd_handle_t cmd, const uint8_t address, const uint16_t reg)
{
	uint8_t data[] = {
		(address << 1) | MASTER_WRITE,
		(uint8_t) (reg >> 8),
		reg & 0xff};
	esp_err_t res = i2c_master_write(cmd, data, 3, ACK_EN);
	ESP_ERROR_CHECK(res);
    return res;
}

static esp_err_t i2c_master_read_from_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
	i2c_write_reg16_to_device(cmd, address, reg);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	esp_err_t result = i2c_master_cmd_begin(I2C_BUS, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	ESP_ERROR_CHECK(result);
	if (result != ESP_OK) {
		check_err(result);
		return result;
	}

	result = i2c_master_read_from_device(I2C_BUS, address, buf, size, 50/portTICK_PERIOD_MS);
	ESP_ERROR_CHECK(result);
	return result;

	/*
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (address << 1) | MASTER_READ, ACK_EN));


	uint16_t bytes_read = 0;
	uint16_t pos = size - 1;

	while (bytes_read < size) {
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &buf[bytes_read], ACK_EN));
		bytes_read++;
		pos--;
	}

    //ESP_ERROR_CHECK(i2c_master_read(cmd, buf, size, ACK_EN));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
    result = i2c_master_cmd_begin(I2C_BUS, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
	ESP_ERROR_CHECK(result);

	#ifdef DO_DEBUG
		fflush(stdout);
		printf("Read %d bytes at address %04x: ", size, reg);
		for (uint32_t i = 0; i < size; i++)
			printf("%02x ", buf[i]);;
		printf("\n");
	#endif
	*/

	return result;
}

static esp_err_t i2c_master_read_single_byte(const uint8_t address, const uint16_t reg, uint8_t* data, bool ack)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
	i2c_write_reg16_to_device(cmd, address, reg);
	i2c_master_read_byte(cmd, data, ack);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	esp_err_t result = i2c_master_cmd_begin(I2C_BUS, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	ESP_ERROR_CHECK(result);

	return result;
}

static esp_err_t i2c_master_write_single_byte(const uint8_t address, const uint16_t reg, uint8_t value)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
	i2c_write_reg16_to_device(cmd, address, reg);
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value, ACK_EN));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	esp_err_t result = i2c_master_cmd_begin(I2C_BUS, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	ESP_ERROR_CHECK(result);

	return result;
}

static esp_err_t i2c_master_read_from_reg16_2(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
	esp_err_t res = ESP_OK;
	uint8_t ack;

	for (uint32_t i = 0; i < size; i++) {
		if (i == (size-1))
			ack = I2C_MASTER_LAST_NACK;
		else
			ack = I2C_MASTER_ACK;
		res = i2c_master_read_single_byte(address, reg+i, &buf[i], ack);
	}
	return res;
}

static esp_err_t i2c_master_write_to_reg16_2(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
	esp_err_t res = ESP_OK;
	for (uint32_t i = 0; i < size; i++) {
		res = i2c_master_write_single_byte(address, reg+i, buf[i]);
	}
	return res;
}

static esp_err_t i2c_master_write_to_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
	i2c_write_reg16_to_device(cmd, address, reg);
	ESP_ERROR_CHECK(i2c_master_write(cmd, buf, size, ACK_EN));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t result = i2c_master_cmd_begin(I2C_BUS, cmd, MASTER_TIMEOUT/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	ESP_ERROR_CHECK(result);

	#ifdef DO_DEBUG
		fflush(stdout);
		printf("Write %d bytes to address %04x: ", size, reg);
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
	//printf("Read Address: %04x: %02x\n", RegisterAdress, *p_value);
	esp_err_t res = i2c_master_read_from_reg16(p_platform->address, RegisterAdress, p_value, 1);
	ESP_ERROR_CHECK(res);
    return res;
}

uint8_t WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	uint8_t buf[1] = {value};
    //return i2c_master_write_to_reg16(p_platform->address, RegisterAdress, &value, 1);
	//printf("Write Address: %04x: %02x\n", RegisterAdress, value);
	esp_err_t res = i2c_master_write_to_reg16(p_platform->address, RegisterAdress, buf, 1);
	ESP_ERROR_CHECK(res);
	return res;
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
