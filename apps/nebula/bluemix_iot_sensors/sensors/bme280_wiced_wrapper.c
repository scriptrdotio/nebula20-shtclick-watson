/*
 * Copyright 2017, Future Electronics Inc. or a subsidiary of
 * Future Electronics Inc. All Rights Reserved.
 *
 * This software, associated documentation and materials ("Software"),
 * is owned by Future Electronics Inc. or one of its
 * subsidiaries ("Future Electronics") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Future Electronics hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Future Electronics's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Future Electronics.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Future Electronics
 * reserves the right to make changes to the Software without notice. Future Electronics
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Future Electronics does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Future Electronics product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Future Electronics's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Future Electronics against all liability.
 */

/** @file
 *  Implements functions to get the BME280 driver from Bosch to work with WICED.
 */

#include "bme280_wiced_wrapper.h"

/******************************************************
 *               Variable Definitions
 ******************************************************/
/**
 * The I2C device that is used to communicate with the BME280.
 */
static wiced_i2c_device_t bme280_i2c_dev;

/**
 * The SPI device that is used to communicate with the BME280.
 */
static wiced_spi_device_t bme280_spi_dev;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/**
 * Read the BME280 device register with I2C communications. This is used by the bme280_dev
 * as a function pointer for read.
 *
 * @param[in]  dev_id   : The device ID (I2C address)
 * @param[in]  reg_addr : The register address to read
 * @param[out] data     : The buffer to put the read data into
 * @param[in]  len      : The length of data to read
 *
 * @return 0 for success, otherwise error
 */
static int8_t bme280_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

/**
 * Write the BME280 device register with I2C communications. This is used by the bme280_dev
 * as a function pointer for write.
 *
 * @param[in] dev_id   : The device ID (I2C address)
 * @param[in] reg_addr : The register address to write to
 * @param[in] data     : The buffer data to write
 * @param[in] len      : The length of data to write
 *
 * @return 0 for success, otherwise error
 */
static int8_t bme280_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

/**
 * Read the BME280 device register with SPI communications. This is used by the bme280_dev
 * as a function pointer for read.
 *
 * @param[in]  dev_id   : The device ID (I2C address)
 * @param[in]  reg_addr : The register address to read
 * @param[out] data     : The buffer to put the read data into
 * @param[in]  len      : The length of data to read
 *
 * @return 0 for success, otherwise error
 */
static int8_t bme280_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

/**
 * Write the BME280 device register with SPI communications. This is used by the bme280_dev
 * as a function pointer for write.
 *
 * @param[in] dev_id   : The device ID (I2C address)
 * @param[in] reg_addr : The register address to write to
 * @param[in] data     : The buffer data to write
 * @param[in] len      : The length of data to write
 *
 * @return 0 for success, otherwise error
 */
static int8_t bme280_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

/**
 * Delay for period milliseconds. This is used by the bme200_dev as a function pointer
 * for delay_ms.
 *
 * @param[in]  period : The number of milliseconds to delay
 *
 * @return void
 */
static void bme280_delay_ms(uint32_t period);

/******************************************************
 *               Function Definitions
 ******************************************************/

static int8_t bme280_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	wiced_i2c_message_t msg;

	if(data == NULL){
		return BME280_E_NULL_PTR;
	}

	bme280_i2c_dev.address = (uint16_t)dev_id;

	if(wiced_i2c_init_tx_message(&msg, &reg_addr, 1, 1, BME280_I2C_DISABLE_DMA) != WICED_SUCCESS){
		return BME280_E_COMM_FAIL;
	}

	if(wiced_i2c_transfer(&bme280_i2c_dev, &msg, 1) != WICED_SUCCESS){
		return BME280_E_COMM_FAIL;
	}

	if(wiced_i2c_init_rx_message(&msg, data, len, 1, BME280_I2C_DISABLE_DMA) != WICED_SUCCESS){
		return BME280_E_COMM_FAIL;
	}

	if(wiced_i2c_transfer(&bme280_i2c_dev, &msg, 1) != WICED_SUCCESS){
		return BME280_E_COMM_FAIL;
	}

	return BME280_OK;
}

static int8_t bme280_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	wiced_i2c_message_t msg;
	uint8_t* tx_data;

	if(data == NULL){
		return BME280_E_NULL_PTR;
	}

	tx_data = (uint8_t*)malloc(len+1);
	if(tx_data == NULL){
		return BME280_E_INVALID_LEN;
	}

	bme280_i2c_dev.address = (uint16_t)dev_id;

	tx_data[0] = reg_addr;
	memcpy(&tx_data[1], data, len);

	if(wiced_i2c_init_tx_message(&msg, tx_data, len+1, 1, BME280_I2C_DISABLE_DMA) != WICED_SUCCESS){
		free(tx_data);
		return BME280_E_COMM_FAIL;
	}

	if(wiced_i2c_transfer(&bme280_i2c_dev, &msg, 1) != WICED_SUCCESS){
		free(tx_data);
		return BME280_E_COMM_FAIL;
	}

	free(tx_data);
	return BME280_OK;
}


static void bme280_delay_ms(uint32_t period)
{
	wiced_rtos_delay_milliseconds(period);
}


static int8_t bme280_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	wiced_spi_message_segment_t msg;
	uint8_t *xfer_buffer;
	int8_t ret = BME280_OK;

	if(data == NULL){
		return BME280_E_NULL_PTR;
	}

	xfer_buffer = (uint8_t*)malloc(len+1);
	if(xfer_buffer == NULL){
		return BME280_E_INVALID_LEN;
	}

	xfer_buffer[0] = reg_addr;
	memset(&xfer_buffer[1], 0x00, len);

	msg.length    = len+1;
	msg.tx_buffer = xfer_buffer;
	msg.rx_buffer = xfer_buffer;

	if(wiced_spi_transfer(&bme280_spi_dev, &msg, 1 ) == WICED_SUCCESS){
		memcpy(data, &xfer_buffer[1], len);
	}
	else{
		ret = BME280_E_COMM_FAIL;
	}

	free(xfer_buffer);
	return ret;
}


static int8_t bme280_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	wiced_spi_message_segment_t msg;
	uint8_t *xfer_buffer;
	int8_t ret = BME280_OK;

	if(data == NULL){
		return BME280_E_NULL_PTR;
	}

	xfer_buffer = (uint8_t*)malloc(len+1);
	if(xfer_buffer == NULL){
		return BME280_E_INVALID_LEN;
	}

	xfer_buffer[0] = reg_addr;
	memcpy(&xfer_buffer[1], data, len);

	msg.length    = len+1;
	msg.tx_buffer = xfer_buffer;
	msg.rx_buffer = NULL;

	if(wiced_spi_transfer(&bme280_spi_dev, &msg, 1 ) != WICED_SUCCESS){
		ret = BME280_E_COMM_FAIL;
	}

	free(xfer_buffer);
	return ret;
}

wiced_result_t bme280_wiced_init_i2c(struct bme280_dev *dev, wiced_i2c_t i2c_port, uint8_t i2c_addr)
{
	wiced_result_t wres;

	bme280_i2c_dev.port = i2c_port;
	bme280_i2c_dev.address = (uint16_t)i2c_addr;
	bme280_i2c_dev.address_width = I2C_ADDRESS_WIDTH_7BIT;
	bme280_i2c_dev.flags = 0x00;
	bme280_i2c_dev.speed_mode = I2C_HIGH_SPEED_MODE;

	dev->id = i2c_addr;
	dev->interface = BME280_I2C_INTF;
	dev->read = bme280_i2c_read;
	dev->write = bme280_i2c_write;
	dev->delay_ms = bme280_delay_ms;

	if((wres = wiced_i2c_init(&bme280_i2c_dev)) != WICED_SUCCESS){
		return wres;
	}

	if(bme280_init(dev) != BME280_OK){
		wres = WICED_ERROR;
	}

	return wres;
}


wiced_result_t bme280_wiced_init_spi(struct bme280_dev *dev, wiced_spi_t spi_port, wiced_gpio_t chip_select)
{
	wiced_result_t wres;

	bme280_spi_dev.port = spi_port;
	bme280_spi_dev.chip_select = chip_select;
	bme280_spi_dev.mode = (SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_HIGH | SPI_NO_DMA | SPI_MSB_FIRST);
	bme280_spi_dev.speed = 10000000; /* 10MHz */
	bme280_spi_dev.bits = 8;

	dev->interface = BME280_SPI_INTF;
	dev->read = bme280_spi_read;
	dev->write = bme280_spi_write;
	dev->delay_ms = bme280_delay_ms;

	if((wres = wiced_spi_init(&bme280_spi_dev)) != WICED_SUCCESS){
		return wres;
	}

	if(bme280_init(dev) != BME280_OK){
		wres = WICED_ERROR;
	}

	return wres;
}


uint16_t bme280_wiced_get_meas_time(const struct bme280_dev *dev)
{
	uint16_t meas_time = 0;

	if(dev == NULL){
		return 0;
	}

	if(dev->settings.osr_h != BME280_NO_OVERSAMPLING || dev->settings.osr_p != BME280_NO_OVERSAMPLING
			|| dev->settings.osr_t != BME280_NO_OVERSAMPLING){
		meas_time = 1;

		if(dev->settings.osr_h != BME280_NO_OVERSAMPLING || dev->settings.osr_p != BME280_NO_OVERSAMPLING){
			meas_time += 1;
		}

		if(dev->settings.osr_t != BME280_NO_OVERSAMPLING){
			meas_time += (0x01 << (dev->settings.osr_t));
		}

		if(dev->settings.osr_p != BME280_NO_OVERSAMPLING){
			meas_time += (0x01 << (dev->settings.osr_p));
		}

		if(dev->settings.osr_h != BME280_NO_OVERSAMPLING){
			meas_time += (0x01 << (dev->settings.osr_h));
		}
	}

	return meas_time;
}
