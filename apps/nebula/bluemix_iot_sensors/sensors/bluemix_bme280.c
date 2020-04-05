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
 *  Defines functions to use the BME280 with the Bluemix project.
 */

#include "bluemix_bme280.h"
#include "bme280_wiced_wrapper.h"

/******************************************************
 *               Variable Definitions
 ******************************************************/
/** BME280 device */
static struct bme280_dev dev_bme280;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t init_bme280(void)
{
	wiced_result_t wres;
	uint8_t settings_sel;
	int8_t bme_rslt;

	if((wres = bme280_wiced_init_i2c(&dev_bme280, BME280_I2C, BME280_I2C_ADDR_PRIM)) != WICED_SUCCESS){
		WPRINT_APP_INFO( ( "No BME280 sensor found\n") );
		return wres;
	}

	WPRINT_APP_INFO(("Found BME280 Humidity, Pressure, and Temperature Sensor. Initializing...\n" ));

	/* BME280 datasheet recommended mode of operation: Indoor navigation */
	dev_bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev_bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev_bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev_bme280.settings.filter = BME280_FILTER_COEFF_16;
	dev_bme280.settings.standby_time = BME280_STANDBY_TIME_125_MS;

	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL | BME280_STANDBY_SEL;

	if((bme_rslt = bme280_set_sensor_settings(settings_sel, &dev_bme280)) != BME280_OK){
		WPRINT_APP_INFO(("Error %d while configuring BME280!\n", bme_rslt));
	}

	/* Start periodic measurements */
	bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev_bme280);

	return WICED_SUCCESS;
}

wiced_result_t read_bme280(char *mqtt_msg, wiced_bool_t first_data)
{
	int8_t bme_rslt;
	struct bme280_data comp_data;
	uint16_t msg_len, start_len;
	int add_pos;

	start_len = msg_len = strlen(mqtt_msg);

	if((bme_rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev_bme280)) == BME280_OK){
		WPRINT_APP_INFO((" BME280: Temperature = %.2f\xf8""C, Humidity = %.2f%%, Pressure = %.2fPa\n", comp_data.temperature, comp_data.humidity, comp_data.pressure));
		if(!first_data){
			strcat(mqtt_msg, ",");
			msg_len++;
		}
		if((add_pos = sprintf(&mqtt_msg[msg_len], "\"bme280_temp\":%.2f,\"bme280_humidity\":%.2f,\"bme280_pressure\":%.2f",comp_data.temperature, comp_data.humidity, comp_data.pressure)) >= 0){
			msg_len += add_pos;
		}
	}
	else{
		WPRINT_APP_INFO((" Error %d reading BME280 sensor data!\n", bme_rslt));
	}

	if(start_len == msg_len){
		return WICED_ERROR;
	}

	return WICED_SUCCESS;
}
