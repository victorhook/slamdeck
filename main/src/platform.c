#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "platform.h"
#include "i2c.h"
#include "slamdeck.h"

static const char* TAG = "Platform";

//#define DO_DEBUG

uint8_t RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	return RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	return WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	#ifdef DO_DEBUG
		ESP_LOGD(TAG, "[%02x] Writing %d bytes to %04x", p_platform->address, size, RegisterAdress);
	#endif
	uint8_t res = i2c_master_write_to_reg16(p_platform->address, RegisterAdress, p_values, size);
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "I2C ERROR: %s", esp_err_to_name(res));
	}
	return res;
}

uint8_t RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t res = i2c_master_read_from_reg16(p_platform->address, RegisterAdress, p_values, size);
	#ifdef DO_DEBUG
		ESP_LOGD(TAG, "[%02x] Read %02x to %04x", p_platform->address, *p_value, RegisterAdress);
	#endif
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "I2C ERROR: %s", esp_err_to_name(res));
	}
	return res;
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
	uint32_t tmp;

	for (uint32_t i = 0; i < size; i = i + 4)
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
