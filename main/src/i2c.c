#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "stdio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "i2c.h"
#include "slamdeck.h"


SemaphoreHandle_t semaphore_i2c_master;


#define SLAVE_RX_BUFFER    128
#define SLAVE_TX_BUFFER    128
#define I2C_MASTER_TIMEOUT 50
#define I2C_BUS            SLAMDECK_I2C_BUS_MASTER

#define I2C_TRANSFER_CHUNK_SIZE 1024
// Biggest transaction: Write I2C_TRANSFER_CHUNK_SIZE bytes:
// Write addr
// Write high(reg)
// Write low(reg)
// Write I2C_TRANSFER_CHUNK_SIZE bytes
#define I2C_CMD_LINK_BUFF_SIZE I2C_LINK_RECOMMENDED_SIZE(I2C_TRANSFER_CHUNK_SIZE + 3)
static uint8_t i2c_cmd_link_buf[I2C_CMD_LINK_BUFF_SIZE];


#define DO_DEBUG
#ifdef DO_DEBUG
#define check_err(res) printf("Result: %s\n", esp_err_to_name(res))
#else
#define check_err(res)
#endif


/*
Reset rx and tx buffers?
*/

static const char* TAG = "I2C";
static uint8_t i2c_initialized = 0;


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

static inline esp_err_t i2c_write_reg16_to_device(i2c_cmd_handle_t cmd, const uint8_t address, const uint16_t reg)
{
	uint8_t data[] = {
		(address << 1) | I2C_MASTER_WRITE,
		(uint8_t) (reg >> 8),
		reg & 0xff};
	//#ifdef DO_DEBUG
	//	ESP_LOGD(TAG, "i2c addr: %02x", (address << 1) | I2C_MASTER_WRITE);
	//#endif
	return i2c_master_write(cmd, data, 3, I2C_ACK_CHECK_EN);
}

static esp_err_t i2c_master_read_chunk_from_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
	xSemaphoreTake(semaphore_i2c_master, portMAX_DELAY);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(i2c_cmd_link_buf, I2C_CMD_LINK_BUFF_SIZE);
    i2c_master_start(cmd);
    i2c_write_reg16_to_device(cmd, address, reg);

	// Repeated start in order to read
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
	uint32_t last = size - 1;
	for (uint32_t i = 0; i < size; i++) {
		i2c_ack_type_t ack = i == last ? I2C_MASTER_LAST_NACK : I2C_MASTER_ACK;
		i2c_master_read_byte(cmd, &buf[i], ack);
	}
	i2c_master_stop(cmd);
	esp_err_t result = i2c_master_cmd_begin(I2C_BUS, cmd, I2C_MASTER_TIMEOUT/portTICK_PERIOD_MS);
	i2c_cmd_link_delete_static(cmd);

	xSemaphoreGive(semaphore_i2c_master);
	return result;
}

static esp_err_t i2c_master_write_chunk_to_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
	xSemaphoreTake(semaphore_i2c_master, portMAX_DELAY);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(i2c_cmd_link_buf, I2C_CMD_LINK_BUFF_SIZE);
    i2c_master_start(cmd);
    i2c_write_reg16_to_device(cmd, address, reg);
	i2c_master_write(cmd, buf, size, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_BUS, cmd, I2C_MASTER_TIMEOUT/portTICK_RATE_MS);
    i2c_cmd_link_delete_static(cmd);

	xSemaphoreGive(semaphore_i2c_master);
	return result;
}

int i2c_master_ping_address(const uint8_t address)
{
	xSemaphoreTake(semaphore_i2c_master, portMAX_DELAY);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_BUS, cmd, 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
	xSemaphoreGive(semaphore_i2c_master);
    return ret;
}

int i2c_is_initialized()
{
    return i2c_initialized;
}

int i2c_master_read_from_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
	//#ifdef DO_DEBUG
	//	printf("-- [%02x] Read %d bytes from register %04x --\n", address, size, reg);
	//#endif
	uint16_t bytes_left = size;
	uint16_t index = 0;
	uint16_t chunk_size = 0;
	esp_err_t err = ESP_OK;

	while (bytes_left > 0) {
		if (bytes_left > I2C_TRANSFER_CHUNK_SIZE)
			chunk_size = I2C_TRANSFER_CHUNK_SIZE;
		else
			chunk_size = bytes_left;

		err = i2c_master_read_chunk_from_reg16(address, reg+index, &buf[index], chunk_size);

		if (err) {
			#ifdef DO_DEBUG
				check_err(err);
			#endif
			return err;
		}

		bytes_left -= chunk_size;
		index += chunk_size;
	}

    return err;
}

int i2c_master_write_to_reg16(const uint8_t address, const uint16_t reg, uint8_t* buf, const uint32_t size)
{
	//#ifdef DO_DEBUG
	//	printf("-- [%02x] Write %d bytes to register %04x --\n", address, size, reg);
    //#endif

	uint16_t bytes_left = size;
	uint16_t index = 0;
	uint16_t chunk_size = 0;
	esp_err_t err = ESP_OK;

	while (bytes_left > 0) {
		if (bytes_left > I2C_TRANSFER_CHUNK_SIZE)
			chunk_size = I2C_TRANSFER_CHUNK_SIZE;
		else
			chunk_size = bytes_left;

		err = i2c_master_write_chunk_to_reg16(address, reg+index, &buf[index], chunk_size);

		if (err) {
			#ifdef DO_DEBUG
				check_err(err);
			#endif
			return err;
		}

		bytes_left -= chunk_size;
		index += chunk_size;
	}

    return 0;
}

void i2c_init()
{
	semaphore_i2c_master = xSemaphoreCreateBinary();
	xSemaphoreGive(semaphore_i2c_master);
    ESP_ERROR_CHECK(i2c_init_master());
    ESP_LOGI(TAG, "initialized OK");
    i2c_initialized = 1;
}
