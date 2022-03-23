#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "stdio.h"

#include "i2c.h"
#include "slamdeck.h"


#define SLAVE_RX_BUFFER 128
#define SLAVE_TX_BUFFER 128
#define I2C_MASTER_TIMEOUT_MS 200
#define I2C_MASTER_WRITE 0
#define I2C_MASTER_READ  1

#define ACK_CHECK_DIS 0
#define ACK_CHECK_EN  1

#define I2C_ERROR -1

#define addr_write(address) ((address << 1) | I2C_MASTER_WRITE)
#define addr_read(address)  ((address << 1) | I2C_MASTER_READ)

static const char* TAG = "I2C";


static esp_err_t i2c_init_master(void)
{
    int i2c_master_port = SLAMDECK_I2C_BUS_MASTER;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SLAMDECK_I2C_SDA,
        .scl_io_num = SLAMDECK_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = SLAMDECK_I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_init_slave(void)
{
    int i2c_slave_port = SLAMDECK_I2C_BUS_CF;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SLAMDECK_I2C_SCL_CF,
        .scl_io_num = SLAMDECK_I2C_SDA_CF,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = SLAMDECK_I2C_ADDRESS_CF
    };

    i2c_param_config(i2c_slave_port, &conf);

    return i2c_driver_install(i2c_slave_port, conf.mode, SLAVE_RX_BUFFER, SLAVE_TX_BUFFER, 0);
}


static inline esp_err_t i2c_write_reg16_to_device(const uint8_t address, const uint16_t reg)
{
    uint8_t reg_bytes[] = {reg << 8, reg & 0xff};
    return i2c_master_write_to_device(SLAMDECK_I2C_BUS_MASTER, address, reg_bytes, 2, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);
}


esp_err_t i2c_master_write_byte_to_reg16(const uint8_t address, const uint16_t reg, const uint8_t value)
{
    esp_err_t res = i2c_write_reg16_to_device(address, reg);
    uint8_t buf[] = {value};
    if (res != ESP_OK)
        return res;
    return i2c_master_write_to_device(SLAMDECK_I2C_BUS_MASTER, address, buf, 1, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);
}

esp_err_t i2c_master_read_byte_from_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf)
{
    esp_err_t res = i2c_write_reg16_to_device(address, reg);
    if (res != ESP_OK)
        return res;
    return i2c_master_read_from_device(SLAMDECK_I2C_BUS_MASTER, address, buf, 1, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);
}

#define I2C_MULTI_CHUNK_SIZE 32

esp_err_t i2c_master_read_bytes_from_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
    esp_err_t res = i2c_write_reg16_to_device(address, reg);
    if (res != ESP_OK)
        ESP_LOGI(TAG, "Error reading: %s", esp_err_to_name(res));
        return res;

    return i2c_master_read_from_device(SLAMDECK_I2C_BUS_MASTER, address, buf, size, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr_read(address), ACK_CHECK_EN);
    i2c_master_read(cmd, buf, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    res = i2c_master_cmd_begin(SLAMDECK_I2C_BUS_MASTER, cmd, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);


    return res;




    res = i2c_master_read_from_device(SLAMDECK_I2C_BUS_MASTER, address, buf, size, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);
    return res;

    uint32_t bytes_read = 0;
    uint32_t bytes_to_read;
    uint32_t bytes_left;

    while (bytes_read < size) {
        bytes_left = (size - bytes_read);
        if (bytes_left >= I2C_MULTI_CHUNK_SIZE)
            bytes_to_read = I2C_MULTI_CHUNK_SIZE;
        else
            bytes_to_read = bytes_left;

        //ESP_LOGI(TAG, "Size %d, BR: %d, BtR: %d, BL: %d, Chunk: %d", size, bytes_read, bytes_to_read, bytes_left, I2C_MULTI_CHUNK_SIZE);

        res = i2c_master_read_from_device(SLAMDECK_I2C_BUS_MASTER, address, &buf[size-bytes_read], bytes_to_read, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);
        //ESP_LOGI(TAG, "Reading %d bytes", bytes_to_read);

        bytes_read += bytes_to_read;

        if (res != ESP_OK)
            ESP_LOGI(TAG, "Error reading: %s", esp_err_to_name(res));
            return res;
    }

    return res;
}

esp_err_t i2c_master_write_bytes_to_reg16(const uint8_t address, const uint16_t reg, const uint8_t* buf, const uint32_t size)
{
    esp_err_t res = ESP_OK;
    uint32_t bytes_sent = 0;
    uint32_t bytes_to_send;
    uint32_t bytes_left;

    while (bytes_sent < size) {
        bytes_left = (size - bytes_sent);
        if (bytes_left > I2C_MULTI_CHUNK_SIZE)
            bytes_to_send = I2C_MULTI_CHUNK_SIZE;
        else
            bytes_to_send = bytes_left;

        res = i2c_write_reg16_to_device(address, reg);
        if (res != ESP_OK)
            return res;

        res = i2c_master_write_to_device(SLAMDECK_I2C_BUS_MASTER, address, &buf[bytes_sent], bytes_sent, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);
        ESP_LOGI(TAG, "Sending %d bytes", bytes_to_send);

        bytes_sent += bytes_to_send;

        if (res != ESP_OK)
            ESP_LOGI(TAG, "Error reading: %s", esp_err_to_name(res));
            return res;
    }

    printf("Writing following %d bytes: ", size);
    for (uint32_t i = 0; i < size; i++) {
        printf("%02x ", buf[i]);
    }
    printf(" ");

    return res;
}


void i2c_init()
{
    ESP_ERROR_CHECK(i2c_init_master());
    ESP_LOGI(TAG, "initialized OK");
}


/*
    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(i2c_cmd_link_buf, I2C_CMD_LINK_BUF_SIZE);
    i2c_master_start(cmd);
    //i2c_write_reg16_to_device(cmd, address, reg);
    i2c_master_write_byte(cmd, addr_write(address), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg << 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg & 0xff, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    // For some reason we must send this part of the commands first, and then completely restart. Skipping this crashses esp...
    esp_err_t result = i2c_master_cmd_begin(SLAMDECK_I2C_BUS_MASTER, cmd, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);
    i2c_cmd_link_delete_static(cmd);
    if (result != ESP_OK)
        return result;

    cmd = i2c_cmd_link_create_static(i2c_cmd_link_buf, I2C_CMD_LINK_BUF_SIZE);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr_read(address), ACK_CHECK_EN);
    i2c_master_read_byte(cmd, buf, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    result = i2c_master_cmd_begin(SLAMDECK_I2C_BUS_MASTER, cmd, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);
    i2c_cmd_link_delete_static(cmd);

    //return i2c_master_read_from_device(SLAMDECK_I2C_BUS_MASTER, address, buf, 1, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);

    return result;
*/

/*
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //i2c_write_reg16_to_device(cmd, address, reg);
    i2c_master_write_byte(cmd, addr_write(address), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg << 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg & 0xff, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_DIS);
    i2c_master_stop(cmd);

    esp_err_t result = i2c_master_cmd_begin(SLAMDECK_I2C_BUS_MASTER, cmd, I2C_MASTER_TIMEOUT_MS/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return result;
    */