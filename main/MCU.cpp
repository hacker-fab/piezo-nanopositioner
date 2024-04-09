#include "driver/spi_common.h"
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
#include <math.h>
#include "esp_timer.h"

const gpio_num_t ENEABLE_24V_PIN = GPIO_NUM_8;
const gpio_num_t DAC_SPI_MOSI = GPIO_NUM_6;
const gpio_num_t DAC_SPI_MISO = GPIO_NUM_15;
const gpio_num_t DAC_SPI_SCLK = GPIO_NUM_7;
const gpio_num_t DAC_SPI_CS = GPIO_NUM_16;

const int railCorrection = 12000;
const int maxADCValue = pow(2, 16)-1-railCorrection;

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
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = SPI_MASTER_FREQ_40M,
		.spics_io_num = DAC_SPI_CS,
		.queue_size = 7,
	};

	esp_err_t ret;

	ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
	assert(ret==ESP_OK);

	ret = spi_bus_add_device(SPI2_HOST, &devcfg, &SPIDevice);
	assert(ret==ESP_OK);

	vTaskDelay(100 / portTICK_PERIOD_MS);
}

void writeToRegister(DACRegister address, uint16_t value)    {
	spi_device_acquire_bus(SPIDevice, portMAX_DELAY);
	spi_transaction_t transaction = {
		.flags = SPI_TRANS_USE_TXDATA,
		.length = 3*8,
	};

	transaction.tx_data[0] = (0b00000000 + (uint8_t)address);
	transaction.tx_data[1] = value >> 8;
	transaction.tx_data[2] = value;

	esp_err_t ret;
	ret = spi_device_transmit(SPIDevice, &transaction);
	assert(ret==ESP_OK);
	spi_device_release_bus(SPIDevice);
}

void fast_writeToRegister(DACRegister address, uint16_t value)    {
	spi_transaction_t transaction = {
		.flags = SPI_TRANS_USE_TXDATA,
		.length = 3*8,
	};

	transaction.tx_data[0] = (0b00000000 + (uint8_t)address);
	transaction.tx_data[1] = value >> 8;
	transaction.tx_data[2] = value;

	spi_device_polling_transmit(SPIDevice, &transaction);
}

void readFromRegister(DACRegister address, uint16_t &value){
	spi_device_acquire_bus(SPIDevice, portMAX_DELAY);
	uint8_t t_bytes[3];
	t_bytes[0] = (0b10000000 + (uint8_t)address);
	t_bytes[1] = 0;
	t_bytes[2] = 0;

	uint8_t r_bytes[3];
	memset(r_bytes, 0, sizeof(r_bytes));

	spi_transaction_t transaction = {
		.length = sizeof(t_bytes)*8,    // length in bits
		.tx_buffer = t_bytes,
		.rx_buffer = r_bytes
	};

	esp_err_t ret;
	ret = spi_device_transmit(SPIDevice, &transaction);
	assert(ret==ESP_OK);

	memset(r_bytes, 0, sizeof(r_bytes));

	transaction = {
		.length = sizeof(r_bytes)*8,    // length in bits
		.tx_buffer = r_bytes,
		.rx_buffer = r_bytes
	};

	ret = spi_device_transmit(SPIDevice, &transaction);
	assert(ret==ESP_OK);

	value = (uint16_t)(r_bytes[1] << 8 | r_bytes[2]);
	spi_device_release_bus(SPIDevice);
}

void addChannelToBroadcast(DACChannel channel) {
	uint16_t registerValue = 0;
	readFromRegister(DACRegister::SYNC, registerValue);

	writeToRegister(DACRegister::SYNC, registerValue | (1 << (8+channel)));
}

void removeChannelFromBroadcast(DACChannel channel) {
	uint16_t registerValue = 0;
	readFromRegister(DACRegister::SYNC, registerValue);

	writeToRegister(DACRegister::SYNC, registerValue & (1 << (8+channel)));
}

void broadcast(uint16_t value) {
	writeToRegister(DACRegister::BRDCAST, value);
}

void fast_broadcast(uint16_t value) {
	fast_writeToRegister(DACRegister::BRDCAST, value+railCorrection);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void stickSlip(int frequency, int direction) {
  long usPeriod = 1000000 / frequency;
  long usStart = esp_timer_get_time();
	uint64_t now = esp_timer_get_time();

	fast_broadcast(direction ? 0 : maxADCValue);

  while (1) {
		now = esp_timer_get_time();
		if (now - usStart > usPeriod) break;

		if (direction == 1) {
			fast_broadcast((double)maxADCValue * pow((double)(now - usStart) / (double)usPeriod, 2));
		} else {
			fast_broadcast(maxADCValue - (double)maxADCValue * pow((double)(now - usStart) / (double)usPeriod, 2));
		}
	}

	fast_broadcast(0);
}

void initiateDAC() {
	removeChannelFromBroadcast(DACChannel::DAC0);
	removeChannelFromBroadcast(DACChannel::DAC1);
	removeChannelFromBroadcast(DACChannel::DAC2);
	removeChannelFromBroadcast(DACChannel::DAC3);

	addChannelToBroadcast(DACChannel::DAC0);
	addChannelToBroadcast(DACChannel::DAC1);
	addChannelToBroadcast(DACChannel::DAC2);
	addChannelToBroadcast(DACChannel::DAC3);

	writeToRegister(DACRegister::GAIN, 0b0000000100001111);
}

extern "C" void app_main(void) {
	setupPins();
	turnDriver(1);
	
	initiateSPI();

	initiateDAC();

  long yieldTimer = esp_timer_get_time();
	spi_device_acquire_bus(SPIDevice, portMAX_DELAY);
	while (1) {
		if (esp_timer_get_time() - yieldTimer > 10000000) {
			vTaskDelay(1); // yield
			yieldTimer = esp_timer_get_time();
		}

		stickSlip(2000, 0);
	}
	spi_device_release_bus(SPIDevice);
}
