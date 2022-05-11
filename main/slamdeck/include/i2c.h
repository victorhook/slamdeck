#pragma once

#define I2C_ACK_CHECK_DIS 0
#define I2C_ACK_CHECK_EN  1
#define I2C_ERROR        -1
#define I2C_MASTER_WRITE  0
#define I2C_MASTER_READ   1
#define i2c_addr_write(address) ((address << 1) | I2C_MASTER_WRITE)
#define i2c_addr_read(address)  ((address << 1) | I2C_MASTER_READ)


void i2c_init();

void i2c_wait_until_initialized();

int i2c_master_write_to_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size);

int i2c_master_read_from_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size);

int i2c_is_initialized();

int i2c_master_ping_address(const uint8_t address);