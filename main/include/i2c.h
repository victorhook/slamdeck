#pragma once

#include "esp_err.h"


void i2c_init();

void i2c_wait_until_initialized();

esp_err_t i2c_master_read_byte_from_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf);

esp_err_t i2c_master_read_bytes_from_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size);

esp_err_t i2c_master_write_byte_to_reg16(const uint8_t address, const uint16_t reg, const uint8_t value);

esp_err_t i2c_master_write_bytes_to_reg16(const uint8_t address, const uint16_t reg, const uint8_t* buf, const uint32_t size);
