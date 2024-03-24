#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include <stdint.h>
#include <sys/_stdint.h>
#include <sys/errno.h>
#include <sys/types.h>
#include <string.h>

const gpio_num_t ENEABLE_24V_PIN = GPIO_NUM_8;
const gpio_num_t DAC_SPI_MOSI = GPIO_NUM_6;
const gpio_num_t DAC_SPI_MISO = GPIO_NUM_15;
const gpio_num_t DAC_SPI_SCLK = GPIO_NUM_7;
const gpio_num_t DAC_SPI_CS = GPIO_NUM_16;

int isDriverOn = 0;
spi_device_handle_t SPIDevice;

enum DACRegister : uint8_t {
	NOP = 0x00,
	DEVICEID = 0x01,
	SYNC = 0x02,
	CONFIG = 0x02,
	GAIN = 0x04,
	TRIGGER = 0x05,
	BRDCAST = 0x06,
	STATUS = 0x07,
	DAC0VAl = 0x08,
	DAC1VAl = 0x09,
	DAC2VAL = 0x0A,
	DAC3VAL = 0x0B,
};

enum DACChannel : uint8_t {
	DAC0 = 0,
	DAC1 = 1,
	DAC2 = 2,
	DAC3 = 3,
};


void turnDriver(uint32_t on) {
	gpio_set_level(ENEABLE_24V_PIN, on);
}

void setupPins() {
	gpio_set_direction(ENEABLE_24V_PIN, GPIO_MODE_OUTPUT);
}

void initiateSPI(){

	spi_bus_config_t buscfg = {
		.mosi_io_num = DAC_SPI_MOSI,
		.miso_io_num = DAC_SPI_MISO,
		.sclk_io_num = DAC_SPI_SCLK,
		.max_transfer_sz = 4092,
	};

	spi_device_interface_config_t devcfg = {
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.mode = 1,
		.clock_source = SPI_CLK_SRC_DEFAULT,
		.cs_ena_pretrans = 7,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = 1100000,
		.spics_io_num = DAC_SPI_CS,
		.queue_size = 7,
	};

	esp_err_t ret;

	ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
	assert(ret==ESP_OK);

	ret = spi_bus_add_device(SPI2_HOST, &devcfg, &SPIDevice);
	assert(ret==ESP_OK);

	vTaskDelay(100 / portTICK_PERIOD_MS);
}

void writeToRegister(DACRegister address, uint16_t value)    {
	uint8_t bytes[3];
	bytes[0] = (0b00000000 + address);
	bytes[1] = value > 8;
	bytes[2] = value;

	spi_transaction_t transaction = {
		.length = sizeof(bytes)*8, // Length in bits
		.tx_buffer = &bytes,
		.rx_buffer = NULL
	};

	esp_err_t ret;
	ret = spi_device_transmit(SPIDevice, &transaction);
	assert(ret==ESP_OK);
}

void readFromRegister(DACRegister address, uint16_t &value){
	spi_device_acquire_bus(SPIDevice, portMAX_DELAY);

	uint8_t bytes[3];
	bytes[0] = (0b10000000 + address);
	bytes[1] = 0;
	bytes[2] = 0;

	spi_transaction_t transaction = {
		.flags = SPI_TRANS_CS_KEEP_ACTIVE,
		.length = sizeof(bytes)*8,    // Length in bits
		.tx_buffer = bytes,
		.rx_buffer = NULL
	};

	esp_err_t ret;
	ret = spi_device_transmit(SPIDevice, &transaction);
	assert(ret==ESP_OK);

	transaction = {
		.flags = SPI_TRANS_CS_KEEP_ACTIVE,
		.length = (sizeof(bytes)-2)*8,
		.tx_buffer = bytes,
		.rx_buffer = NULL
	};

	ret = spi_device_transmit(SPIDevice, &transaction);
	assert(ret==ESP_OK);

	transaction = {
		.flags = SPI_TRANS_CS_KEEP_ACTIVE,
		.length = (2) * 8,
		.rxlength = (2) * 8,
		.tx_buffer = NULL,
		.rx_buffer = &value
	};

	ret = spi_device_transmit(SPIDevice, &transaction);
	assert(ret==ESP_OK);

	spi_device_release_bus(SPIDevice);
}

void addChannelToBroadcast(DACChannel channel) {
	uint16_t registerValue;
	readFromRegister(DACRegister::SYNC, registerValue);

	uint16_t selection = 0 + (1<8+(uint8_t)channel);
	writeToRegister(DACRegister::SYNC, (registerValue & !selection) + (uint16_t)1*selection);
}

extern "C" void app_main(void) {
	setupPins();
	turnDriver(0);
	
	initiateSPI();

	addChannelToBroadcast(DACChannel::DAC0);
}
