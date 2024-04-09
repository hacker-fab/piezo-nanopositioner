#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include <stdint.h>
#include <sys/_stdint.h>
#include <sys/errno.h>
#include <sys/types.h>
#include <string.h>
#include <math.h>
#include <string>
#include "esp_timer.h"
#include "hal/usb_phy_types.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"

const gpio_num_t ENEABLE_24V_PIN = GPIO_NUM_8;
const gpio_num_t DAC_SPI_MOSI = GPIO_NUM_6;
const gpio_num_t DAC_SPI_MISO = GPIO_NUM_15;
const gpio_num_t DAC_SPI_SCLK = GPIO_NUM_7;
const gpio_num_t DAC_SPI_CS = GPIO_NUM_16;

const int railCorrection = 12000;
const int maxADCValue = pow(2, 16)-1-railCorrection;

int g_itf;
int driveFrequency = 1000;
uint8_t transactionBuffer[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1] = {0};

int isDriverOn = 0;
int safeToRun = 1;
int running = 0;
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

enum USBOpCode : uint32_t {
	ABORT = 'abrt',
	RUN = 'runp',
	SETCH = 'setc',
	SETFREQ = 'setf',
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

void setChannelBroadcast(DACChannel channel, int broadcast) {
	uint16_t registerValue = 0;
	readFromRegister(DACRegister::SYNC, registerValue);

	if(broadcast)
		writeToRegister(DACRegister::SYNC, registerValue | (1 << (8+channel)));
	else
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
  uint64_t usStart = esp_timer_get_time();
	uint64_t now = esp_timer_get_time();

	fast_broadcast(direction ? 0 : maxADCValue);

  while (1) {
		now = esp_timer_get_time();
		if (now - usStart > usPeriod) break;
		if (!safeToRun) return;

		if (direction == 1) {
			fast_broadcast((double)maxADCValue * pow((double)(now - usStart) / (double)usPeriod, 2));
		} else {
			fast_broadcast(maxADCValue - (double)maxADCValue * pow((double)(now - usStart) / (double)usPeriod, 2));
		}
	}

	fast_broadcast(0);
}

void initiateDAC() {
	setChannelBroadcast(DACChannel::DAC0, 0);
	setChannelBroadcast(DACChannel::DAC1, 0);
	setChannelBroadcast(DACChannel::DAC2, 0);
	setChannelBroadcast(DACChannel::DAC3, 0);

	writeToRegister(DACRegister::GAIN, 0b0000000100001111);
}

bool startsWithOpcode(int msgSize, USBOpCode opCode) {
	if(msgSize<sizeof(opCode))
		return 0;
	
	return memcmp(transactionBuffer, &opCode, sizeof(opCode)) == 0;
}

bool bufferContentIsBool(int i) {
	if(transactionBuffer[i]<48 || transactionBuffer[i]>48+1) return 0;
	
	return 1;
}

bool bufferContentIsNumber(int i) {
	if(transactionBuffer[i]<48 || transactionBuffer[i]>57) return 0;
	
	return 1;
}

void USBHandler(int msgSize) {
	if(startsWithOpcode(msgSize, USBOpCode::ABORT)) {
		safeToRun=0;
		running=0;

		tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)g_itf, transactionBuffer, 4);
		tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)g_itf, 0);

		return;
	}

	if(running) return;
	safeToRun=1;

	if(startsWithOpcode(msgSize, USBOpCode::SETCH)) {
		if(msgSize<7) return;

		for(int i=0; i<3; i++) {
			if(!bufferContentIsBool(4+i)) return;
		}

		setChannelBroadcast(DACChannel::DAC0, transactionBuffer[4]-48);
		setChannelBroadcast(DACChannel::DAC1, transactionBuffer[5]-48);
		setChannelBroadcast(DACChannel::DAC2, transactionBuffer[6]-48);

		tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)g_itf, transactionBuffer, 4);
		tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)g_itf, 0);

		return;
	}

	if(startsWithOpcode(msgSize, USBOpCode::SETFREQ)) {
		if(msgSize<9) return;

		for(int i=0; i<5; i++) {
			if(!bufferContentIsNumber(4+i)) return;
		}

		std::string n(transactionBuffer+4, transactionBuffer+4+5);
		driveFrequency = std::stoi(n);

		tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)g_itf, transactionBuffer, 4);
		tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)g_itf, 0);

		return;
	}

	if(startsWithOpcode(msgSize, USBOpCode::RUN)) {
		if(msgSize<6) return;

		if(!bufferContentIsBool(4)) return;

		for(int i=0; i<2; i++) {
			if(!bufferContentIsNumber(4+i)) return;
		}

		running=1;

		std::string n(transactionBuffer+4, transactionBuffer+4+2);
		int usDuration = std::stoi(n) * 1000000;

		spi_device_acquire_bus(SPIDevice, portMAX_DELAY);

		uint64_t yieldTimer = esp_timer_get_time();
		uint64_t usStart = esp_timer_get_time();
		while(usStart - esp_timer_get_time() < usDuration) {
			if (esp_timer_get_time() - yieldTimer > 10000000) {
				vPortYield(); // yield
				yieldTimer = esp_timer_get_time();
			}

			if(!safeToRun) {
				safeToRun=1;
				return;
			}

			stickSlip(driveFrequency, transactionBuffer[4]);
		}
		spi_device_release_bus(SPIDevice);

		running=0;

		tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)g_itf, transactionBuffer, 4);
		tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)g_itf, 0);

		return;
	}

}

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event) {
	size_t rx_size = 0;

	tinyusb_cdcacm_read((tinyusb_cdcacm_itf_t)itf, transactionBuffer, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
	if(rx_size > 0){
		g_itf = itf;
		USBHandler(rx_size);
	}
}

void initiateUSB() {
		const tinyusb_config_t tusb_cfg = {
		.device_descriptor = NULL,
		.string_descriptor = NULL,
		.external_phy = false,
		.configuration_descriptor = NULL,
	};

	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	tinyusb_config_cdcacm_t acm_cfg;
	memset(&acm_cfg, 0, sizeof(acm_cfg));
	acm_cfg.callback_rx = &tinyusb_cdc_rx_callback;

	ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

	esp_tusb_init_console(TINYUSB_CDC_ACM_0);
}

extern "C" void app_main(void) {
	setupPins();
	turnDriver(1);
	
	initiateSPI();
	initiateDAC();
	initiateUSB();
}
