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
 *  SHT3x driver implementation.
 */

#include "sht3x.h"
#include <stdlib.h>

/******************************************************
 *                    Constants
 ******************************************************/

/**
 *  CRC8 Calculation Table for the SHT3x
 *  Polynomial: 0x31
 *  Initialization: 0xFF
 *  Reflect input: False
 *  Reflect output: False
 *  Final XOR: 0x00
 */
static const uint8_t sht3x_crc8_table[256] = {
	0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
	0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
	0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
	0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
	0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
	0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
	0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
	0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
	0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
	0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
	0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
	0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
	0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
	0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
	0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
	0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
	0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
	0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
	0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
	0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
	0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
	0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
	0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
	0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
	0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
	0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
	0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
	0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
	0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
	0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
	0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
	0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/**
 * Calculate the 8-bit CRC for the desired data.
 *
 * @param[in] data   : Pointer to the data bytes to use for the calculation
 * @param[in] nbytes : The number of data bytes to use
 *
 * @return uint8_t the 8-bit CRC value
 */
static uint8_t sht3x_calcCrc(uint8_t* data, unsigned nbytes);

/**
 * Write a command to the SHT3x sensor
 *
 * @param[in] device : SHT3x device
 * @param[in] cmd    : The command to write
 *
 * @return @ref wiced_result_t
 */
static wiced_result_t sht3x_write_cmd(sht3x_device_t* device, sht3x_command_t cmd);

/**
 * Write an alert-limit command to the SHT3x sensor
 *
 * @param[in] device      : SHT3x device
 * @param[in] cmd         : The command to use
 * @param[in] temperature : The temperature in Celcius to use for the alert limit
 * @param[in] humidity    : The humidity to use for the alert limit
 *
 * @return @ref wiced_result_t
 */
static wiced_result_t sht3x_write_alert_limit(sht3x_device_t* device, sht3x_command_t cmd, float temperature, float humidity);

/**
 * Perform a read command to the SHT3x sensor
 *
 * @param[in]  device : SHT3x device
 * @param[in]  cmd    : The command to use
 * @param[out] data   : The array to put the read data
 * @param[in]  len    : The length of bytes to read
 *
 * @return @ref wiced_result_t
 */
static wiced_result_t sht3x_do_read_cmd(sht3x_device_t* device, sht3x_command_t cmd, uint8_t* data, uint16_t len);

/**
 * Read an alert-limit setting from the SHT3x
 *
 * @param[in]  device      : SHT3x device
 * @param[in]  cmd         : The command to use
 * @param[out] temperature : Pointer to the temperature value to put the read data in
 * @param[out] humidity    : Pointer to the humidity value to put the read data in
 *
 * @return @ref wiced_result_t
 */
static wiced_result_t sht3x_read_alert_limit(sht3x_device_t* device, sht3x_command_t cmd, float* temperature, float* humidity);

/**
 * Calculate a temperature value from a raw 16-bit value
 *
 * @param[in] rawValue : The raw temperature value
 *
 * @return float the calculated temperature value
 */
static float sht3x_calcTemperature(uint16_t rawValue);

/**
 * Calculate a humidity value from a raw 16-bit value
 *
 * @param[in] rawValue : The raw humidity value
 *
 * @return float the calculated humidity value
 */
static float sht3x_calcHumidity(uint16_t rawValue);

/**
 * Calculate a raw temperature value from a float value
 *
 * @param[in] temperature : The temperature value
 *
 * @return uint16_t the calculated raw temperature value
 */
static uint16_t sht3x_calcRawTemperature(float temperature);

/**
 * Calculate a raw humidity value from a float value
 *
 * @param[in] humidity : The humidity value
 *
 * @return uint16_t the calculated raw humidity value
 */
static uint16_t sht3x_calcRawHumidity(float humidity);

/**
 * Get a 16-bit alert-limit format value from temperature and humidity values
 *
 * @param[in] temperature : The temperature value
 * @param[in] humidity    : The humidity value
 *
 * @return uint16_t the alert-limit format value
 */
static uint16_t sht3x_get_limit_format(float temperature, float humidity);

/**
 * Get temperature and humidity limit values from a alert-limit formatted value
 *
 * @param[in]  limit_format : The alert-limit formatted value
 * @param[out] temperature  : Pointer to the temperature value to set
 * @param[out] humidity     : Pointer to the humidity value to set
 *
 * @return void
 */
static void sht3x_get_limits_from_format(uint16_t limit_format, float* temperature, float* humidity);

/******************************************************
 *               Function Definitions
 ******************************************************/

static uint8_t sht3x_calcCrc(uint8_t* data, unsigned nbytes)
{
	uint8_t crc = 0xFF; // calculated checksum

	while (nbytes-- > 0){
	  crc = sht3x_crc8_table[(crc ^ *data++) & 0xff];
	}

	return crc;
}

static wiced_result_t sht3x_write_cmd(sht3x_device_t* device, sht3x_command_t cmd)
{
	wiced_result_t wres;
	wiced_i2c_message_t msg;
	uint8_t wrBuf[2];

	if(device == NULL){
		return WICED_BADARG;
	}

	wrBuf[0] = ((uint16_t)cmd)>>8;
	wrBuf[1] = ((uint16_t)cmd)&0xFF;

	if((wres = wiced_i2c_init_tx_message(&msg, wrBuf, sizeof(wrBuf), 1, device->disable_dma)) != WICED_SUCCESS){
		return wres;
	}

	if((wres = wiced_i2c_transfer(&device->i2c_dev, &msg, 1)) == WICED_SUCCESS){
		return wres;
	}

	/* We may need to just delay a bit to do the I2C transfer */
	wiced_rtos_delay_milliseconds(SHT3X_RETRY_DELAY_MS);

	return wiced_i2c_transfer(&device->i2c_dev, &msg, 1);
}

static wiced_result_t sht3x_write_alert_limit(sht3x_device_t* device, sht3x_command_t cmd, float temperature, float humidity)
{
	wiced_result_t wres;
	wiced_i2c_message_t msg;
	uint8_t wrBuf[5];
	uint16_t alert_format;

	if(device == NULL){
		return WICED_BADARG;
	}

	if((humidity < 0.0f) || (humidity > 100.0f) || (temperature < -45.0f) || (temperature > 130.0f)){
		return WICED_BADARG;
	}

	alert_format = sht3x_get_limit_format(temperature, humidity);

	wrBuf[0] = ((uint16_t)cmd)>>8;
	wrBuf[1] = ((uint16_t)cmd)&0xFF;
	wrBuf[2] = alert_format>>8;
	wrBuf[3] = alert_format&0xFF;
	wrBuf[4] = sht3x_calcCrc(&wrBuf[2], 2);

	if((wres = wiced_i2c_init_tx_message(&msg, wrBuf, sizeof(wrBuf), 1, device->disable_dma)) != WICED_SUCCESS){
		return wres;
	}

	if((wres = wiced_i2c_transfer(&device->i2c_dev, &msg, 1)) == WICED_SUCCESS){
		return wres;
	}

	/* We may need to just delay a bit to do the I2C transfer */
	wiced_rtos_delay_milliseconds(SHT3X_RETRY_DELAY_MS);

	return wiced_i2c_transfer(&device->i2c_dev, &msg, 1);
}

static wiced_result_t sht3x_do_read_cmd(sht3x_device_t* device, sht3x_command_t cmd, uint8_t* data, uint16_t len)
{
	wiced_result_t wres;
	wiced_i2c_message_t msg;
	uint8_t wrBuf[2];

	if(device == NULL || data == NULL){
		return WICED_BADARG;
	}

	wrBuf[0] = ((uint16_t)cmd)>>8;
	wrBuf[1] = ((uint16_t)cmd)&0xFF;

	if((wres = wiced_i2c_init_combined_message(&msg, wrBuf, data, 2, len, 1, device->disable_dma)) != WICED_SUCCESS){
		return wres;
	}

	if((wres = wiced_i2c_transfer(&device->i2c_dev, &msg, 1)) == WICED_SUCCESS){
		return wres;
	}

	/* We may need to just delay a bit to read the sensor */
	wiced_rtos_delay_milliseconds(SHT3X_RETRY_DELAY_MS);

	return wiced_i2c_transfer(&device->i2c_dev, &msg, 1);
}

static wiced_result_t sht3x_read_alert_limit(sht3x_device_t* device, sht3x_command_t cmd, float* temperature, float* humidity)
{
	wiced_result_t wres;
	uint8_t rdBuf[3];
	uint16_t alert_format;

	if(device == NULL){
		return WICED_BADARG;
	}

	if(temperature == NULL && humidity == NULL){
		return WICED_SUCCESS;
	}

	if((wres = sht3x_do_read_cmd(device, cmd, rdBuf, sizeof(rdBuf))) != WICED_SUCCESS){
		return wres;
	}

	if(sht3x_calcCrc(&rdBuf[0], 2) != rdBuf[2]){
		return WICED_PACKET_BUFFER_CORRUPT;
	}

	alert_format = ((uint16_t)rdBuf[0])<<8;
	alert_format |= ((uint16_t)rdBuf[1]);

	sht3x_get_limits_from_format(alert_format, temperature, humidity);

	return WICED_SUCCESS;
}

wiced_result_t sht3x_hard_reset(wiced_gpio_t gpio, wiced_bool_t invert)
{
	if(!invert){
		wiced_gpio_output_low(gpio);
	}
	else{
		wiced_gpio_output_high(gpio);
	}

	wiced_rtos_delay_microseconds(100);

	if(!invert){
		wiced_gpio_output_high(gpio);
	}
	else{
		wiced_gpio_output_low(gpio);
	}

	wiced_rtos_delay_milliseconds(SHT3X_HARD_RESET_DELAY_MS);

	return WICED_SUCCESS;
}

wiced_result_t sht3x_soft_reset(sht3x_device_t* device)
{
	wiced_result_t wres;

	if((wres = sht3x_write_cmd(device, SHT3X_CMD_SOFT_RESET)) != WICED_SUCCESS){
		return wres;
	}

	wiced_rtos_delay_milliseconds(SHT3X_SOFT_RESET_DELAY_MS);

	return WICED_SUCCESS;
}

wiced_result_t sht3x_read_serial_number(sht3x_device_t* device, uint32_t* serial_number)
{
	wiced_result_t wres;
	uint8_t data[6];

	if((wres = sht3x_do_read_cmd(device, SHT3X_CMD_READ_SERIALNBR, data, sizeof(data))) != WICED_SUCCESS){
		return wres;
	}

	if(sht3x_calcCrc(&data[0], 2) != data[2] || sht3x_calcCrc(&data[3], 2) != data[5]){
		return WICED_PACKET_BUFFER_CORRUPT;
	}

	*serial_number = ((uint32_t)data[0])<<24;
	*serial_number |= ((uint32_t)data[1])<<16;
	*serial_number |= ((uint32_t)data[3])<<8;
	*serial_number |= ((uint32_t)data[4]);

	return WICED_SUCCESS;
}

wiced_result_t sht3x_read_status(sht3x_device_t* device, uint16_t* status)
{
	wiced_result_t wres;
	uint8_t data[3];

	if((wres = sht3x_do_read_cmd(device, SHT3X_CMD_READ_STATUS, data, sizeof(data))) != WICED_SUCCESS){
		return wres;
	}

	if(sht3x_calcCrc(&data[0], 2) != data[2]){
		return WICED_PACKET_BUFFER_CORRUPT;
	}

	*status = ((uint16_t)data[0])<<8;
	*status |= ((uint16_t)data[1]);

	return WICED_SUCCESS;
}

wiced_result_t sht3x_clear_alert_flags(sht3x_device_t* device)
{
	return sht3x_write_cmd(device, SHT3X_CMD_CLEAR_STATUS);
}

wiced_result_t sht3x_get_temp_humdity(sht3x_device_t* device, float* temperature, float* humidity, sht3x_repeatability_t repeatability)
{
	wiced_result_t wres;
	uint8_t data[6];
	wiced_i2c_message_t msg;
	sht3x_command_t cmd;
	uint16_t timeout;
	uint16_t tmpval;

	switch(repeatability)
	{
		case SHT3X_REPEATAB_HIGH:
			if(device->read_mode == SHT3X_MODE_POLLING){
				cmd = SHT3X_CMD_MEAS_POLLING_H;
			}
			else{
				cmd = SHT3X_CMD_MEAS_CLOCKSTR_H;
			}
			break;

		case SHT3X_REPEATAB_MEDIUM:
			if(device->read_mode == SHT3X_MODE_POLLING){
				cmd = SHT3X_CMD_MEAS_POLLING_M;
			}
			else{
				cmd = SHT3X_CMD_MEAS_CLOCKSTR_M;
			}
			break;

		case SHT3X_REPEATAB_LOW:
			if(device->read_mode == SHT3X_MODE_POLLING){
				cmd = SHT3X_CMD_MEAS_POLLING_L;
			}
			else{
				cmd = SHT3X_CMD_MEAS_CLOCKSTR_L;
			}
			break;

		default:
			return WICED_BADARG;
	}

	if((wres = sht3x_write_cmd(device, cmd)) != WICED_SUCCESS){
		return wres;
	}

	if((wres = wiced_i2c_init_rx_message(&msg, data, sizeof(data), 1, device->disable_dma)) != WICED_SUCCESS){
		return wres;
	}

	if(device->read_mode == SHT3X_MODE_CLKSTRETCH)
	{
		if((wres = wiced_i2c_transfer(&device->i2c_dev, &msg, 1)) != WICED_SUCCESS){
			return wres;
		}
	}
	else
	{
		timeout = SHT3X_T_RH_MEASURE_TIMEOUT;
		while(timeout){
			if((wres = wiced_i2c_transfer(&device->i2c_dev, &msg, 1)) == WICED_SUCCESS){
				break;
			}
			wiced_rtos_delay_milliseconds(1);
			timeout--;
		}

		if(timeout == 0){
			return WICED_TIMEOUT;
		}
	}

	if(sht3x_calcCrc(&data[0], 2) != data[2] || sht3x_calcCrc(&data[3], 2) != data[5]){
		return WICED_PACKET_BUFFER_CORRUPT;
	}

	tmpval = ((uint16_t)data[0])<<8;
	tmpval |= ((uint16_t)data[1]);

	*temperature = sht3x_calcTemperature(tmpval);

	tmpval = ((uint16_t)data[3])<<8;
	tmpval |= ((uint16_t)data[4]);

	*humidity = sht3x_calcHumidity(tmpval);

	return WICED_SUCCESS;
}


wiced_result_t sht3x_start_periodic(sht3x_device_t* device, sht3x_repeatability_t repeatability, sht3x_frequency_t frequency)
{
	sht3x_command_t cmd;

	switch(repeatability)
	{
		case SHT3X_REPEATAB_HIGH:
			switch(frequency)
			{
				case SHT3X_FREQUENCY_HZ5:
					cmd = SHT3X_CMD_MEAS_PERI_05_H;
					break;
				case SHT3X_FREQUENCY_1HZ:
					cmd = SHT3X_CMD_MEAS_PERI_1_H;
					break;
				case SHT3X_FREQUENCY_2HZ:
					cmd = SHT3X_CMD_MEAS_PERI_2_H;
					break;
				case SHT3X_FREQUENCY_4HZ:
					cmd = SHT3X_CMD_MEAS_PERI_4_H;
					break;
				case SHT3X_FREQUENCY_10HZ:
					cmd = SHT3X_CMD_MEAS_PERI_10_H;
					break;
				default:
					return WICED_BADARG;
			}
			break;

		case SHT3X_REPEATAB_MEDIUM:
			switch(frequency)
			{
				case SHT3X_FREQUENCY_HZ5:
					cmd = SHT3X_CMD_MEAS_PERI_05_M;
					break;
				case SHT3X_FREQUENCY_1HZ:
					cmd = SHT3X_CMD_MEAS_PERI_1_M;
					break;
				case SHT3X_FREQUENCY_2HZ:
					cmd = SHT3X_CMD_MEAS_PERI_2_M;
					break;
				case SHT3X_FREQUENCY_4HZ:
					cmd = SHT3X_CMD_MEAS_PERI_4_M;
					break;
				case SHT3X_FREQUENCY_10HZ:
					cmd = SHT3X_CMD_MEAS_PERI_10_M;
					break;
				default:
					return WICED_BADARG;
			}
			break;

		case SHT3X_REPEATAB_LOW:
			switch(frequency)
			{
				case SHT3X_FREQUENCY_HZ5:
					cmd = SHT3X_CMD_MEAS_PERI_05_L;
					break;
				case SHT3X_FREQUENCY_1HZ:
					cmd = SHT3X_CMD_MEAS_PERI_1_L;
					break;
				case SHT3X_FREQUENCY_2HZ:
					cmd = SHT3X_CMD_MEAS_PERI_2_L;
					break;
				case SHT3X_FREQUENCY_4HZ:
					cmd = SHT3X_CMD_MEAS_PERI_4_L;
					break;
				case SHT3X_FREQUENCY_10HZ:
					cmd = SHT3X_CMD_MEAS_PERI_10_L;
					break;
				default:
					return WICED_BADARG;
			}
			break;

		default:
			return WICED_BADARG;
	}

	return sht3x_write_cmd(device, cmd);
}

wiced_result_t sht3x_stop_periodic(sht3x_device_t* device)
{
	return sht3x_write_cmd(device, SHT3X_CMD_BREAK);
}

wiced_result_t sht3x_read_measurement_buffer(sht3x_device_t* device, float* temperature, float* humidity)
{
	wiced_result_t wres;
	uint8_t data[6];
	uint16_t tmpval;

	if((wres = sht3x_do_read_cmd(device, SHT3X_CMD_FETCH_DATA, data, sizeof(data))) != WICED_SUCCESS){
		return wres;
	}

	if(sht3x_calcCrc(&data[0], 2) != data[2] || sht3x_calcCrc(&data[3], 2) != data[5]){
		return WICED_PACKET_BUFFER_CORRUPT;
	}

	tmpval = ((uint16_t)data[0])<<8;
	tmpval |= ((uint16_t)data[1]);

	*temperature = sht3x_calcTemperature(tmpval);

	tmpval = ((uint16_t)data[3])<<8;
	tmpval |= ((uint16_t)data[4]);

	*humidity = sht3x_calcHumidity(tmpval);

	return WICED_SUCCESS;
}

wiced_result_t sht3x_set_heater(sht3x_device_t* device, wiced_bool_t enable)
{
	return sht3x_write_cmd(device, enable ? SHT3X_CMD_HEATER_ENABLE : SHT3X_CMD_HEATER_DISABLE);
}

wiced_result_t sht3x_set_alert_limits(sht3x_device_t* device, float temp_high_set, float humidity_high_set, float temp_high_clear, float humidity_high_clear, float temp_low_clear, float humidity_low_clear, float temp_low_set, float humidity_low_set)
{
	wiced_result_t wres;

	if((wres = sht3x_write_alert_limit(device, SHT3X_CMD_W_AL_LIM_HS, temp_high_set, humidity_high_set)) != WICED_SUCCESS){
		return wres;
	}

	if((wres = sht3x_write_alert_limit(device, SHT3X_CMD_W_AL_LIM_HC, temp_high_clear, humidity_high_clear)) != WICED_SUCCESS){
		return wres;
	}

	if((wres = sht3x_write_alert_limit(device, SHT3X_CMD_W_AL_LIM_LC, temp_low_clear, humidity_low_clear)) != WICED_SUCCESS){
		return wres;
	}

	return sht3x_write_alert_limit(device, SHT3X_CMD_W_AL_LIM_LS, temp_low_set, humidity_low_set);
}

wiced_result_t sht3x_get_alert_limits(sht3x_device_t* device, float* temp_high_set, float* humidity_high_set, float* temp_high_clear, float* humidity_high_clear, float* temp_low_clear, float* humidity_low_clear, float* temp_low_set, float* humidity_low_set)
{
	wiced_result_t wres;

	if((wres = sht3x_read_alert_limit(device, SHT3X_CMD_W_AL_LIM_HS, temp_high_set, humidity_high_set)) != WICED_SUCCESS){
		return wres;
	}

	if((wres = sht3x_read_alert_limit(device, SHT3X_CMD_W_AL_LIM_HC, temp_high_clear, humidity_high_clear)) != WICED_SUCCESS){
		return wres;
	}

	if((wres = sht3x_read_alert_limit(device, SHT3X_CMD_W_AL_LIM_LC, temp_low_clear, humidity_low_clear)) != WICED_SUCCESS){
		return wres;
	}

	return sht3x_read_alert_limit(device, SHT3X_CMD_W_AL_LIM_LS, temp_low_set, humidity_low_set);
}

static float sht3x_calcTemperature(uint16_t rawValue)
{
	/*
	 * Calculate temperature [°C]
	 * T = -45 + 175 * rawValue / (2^16-1)
	 */
	return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}

static float sht3x_calcHumidity(uint16_t rawValue)
{
	/*
	 * Calculate relative humidity [%RH]
	 * RH = rawValue / (2^16-1) * 100
	 */
	return 100.0f * (float)rawValue / 65535.0f;
}

static uint16_t sht3x_calcRawTemperature(float temperature)
{
	/*
	 * Calculate raw temperature [ticks]
	 * rawT = (temperature + 45) / 175 * (2^16-1)
	 */
	return (temperature + 45.0f) / 175.0f * 65535.0f;
}

static uint16_t sht3x_calcRawHumidity(float humidity)
{
  /*
   * Calculate raw relative humidity [ticks]
   * rawRH = humidity / 100 * (2^16-1)
   */
  return humidity / 100.0f * 65535.0f;
}

static uint16_t sht3x_get_limit_format(float temperature, float humidity)
{
	uint16_t rawTemp;
	uint16_t rawHumidity;

	rawTemp = sht3x_calcRawTemperature(temperature);
	rawHumidity = sht3x_calcRawHumidity(humidity);

	return ((rawHumidity & 0xFE00) | ((rawTemp >> 7) & 0x001FF));
}

static void sht3x_get_limits_from_format(uint16_t limit_format, float* temperature, float* humidity)
{
	if(temperature != NULL){
		*temperature = sht3x_calcTemperature(limit_format << 7);
	}
	if(humidity != NULL){
		*humidity = sht3x_calcHumidity(limit_format & 0xFE00);
	}
}
