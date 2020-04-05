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
 *  Implements functions to use the SHT3x with the Bluemix project.
 */

#include "bluemix_sht3x.h"
#include "sht3x.h"

/******************************************************
 *               Variable Definitions
 ******************************************************/

/** SHT3x device */
static sht3x_device_t sht3x_dev = {
	.i2c_dev = {
			.port = SHT3X_I2C,
			.address = SHT3X_DEFAULT_I2C_ADDR,
			.address_width = I2C_ADDRESS_WIDTH_7BIT,
			.flags         = 0x00,
			.speed_mode    = I2C_HIGH_SPEED_MODE,
	},
	.disable_dma = WICED_TRUE,
	.read_mode = SHT3X_MODE_POLLING
};

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t init_sht3x(void)
{
	wiced_result_t wres;
	uint16_t status = 0;

	sht3x_hard_reset(SHT3X_RESET, WICED_FALSE);

	wiced_i2c_init(&sht3x_dev.i2c_dev);

	if(wiced_i2c_probe_device(&sht3x_dev.i2c_dev, 4) != WICED_TRUE){
		WPRINT_APP_INFO( ("No SHT3x sensor found.\n" ));
		return WICED_ERROR;
	}

	WPRINT_APP_INFO(("Found SHT3x Temperature and Humidity Sensor, initializing...\n" ));

	if((wres = sht3x_clear_alert_flags(&sht3x_dev)) != WICED_SUCCESS){
		WPRINT_APP_INFO( ( "Error %u while clearing SHT3x alert flags!\n", (unsigned)wres ) );
		return wres;
	}

	if((wres = sht3x_set_heater(&sht3x_dev, WICED_FALSE)) != WICED_SUCCESS){
		WPRINT_APP_INFO( ( "Error %u while setting SHT3x heater off!\n", (unsigned)wres ) );
		return wres;
	}

	if((wres = sht3x_read_status(&sht3x_dev, &status)) == WICED_SUCCESS){
		WPRINT_APP_INFO( ( "SHT3x Status: 0x%04X\n", (unsigned)status ) );
	}
	else{
		WPRINT_APP_INFO( ( "Error %u while reading SHT3x status!\n", (unsigned)wres ) );
		return wres;
	}

	/* Start periodic measurements and return the result */
	return sht3x_start_periodic(&sht3x_dev, SHT3X_REPEATAB_HIGH, SHT3X_FREQUENCY_4HZ);
}

wiced_result_t read_sht3x(char *mqtt_msg, wiced_bool_t first_data)
{
	wiced_result_t wres;
	float temperature, humidity;
	uint16_t msg_len;

	msg_len = strlen(mqtt_msg);

	if((wres = sht3x_read_measurement_buffer(&sht3x_dev, &temperature, &humidity)) == WICED_SUCCESS){
		WPRINT_APP_INFO( ( "SHT3x Temperature = %.1f\xf8""C, Humidity = %.1f%%\n", temperature, humidity ) );

		if(!first_data){
			strcat(mqtt_msg, ",");
			msg_len++;
		}
		sprintf(&mqtt_msg[msg_len], "\"t\":%.1f,\"h\":%.1f", temperature, humidity);
	}
	else{
		WPRINT_APP_INFO( ( "Error %u while reading SHT3x temperature and humidity buffer!\n", (unsigned)wres ) );
	}

	return wres;
}
