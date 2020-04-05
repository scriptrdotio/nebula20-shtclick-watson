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
 *
 * BME280 API Application
 *
 * This application demonstrates how to use the BME280 library API
 * to read temperature, humidity, and pressure.
 *
 * Features demonstrated
 *  - BME280 API
 *
 */

#include "wiced.h"
#include "bme280_test.h"
#include "bme280_wiced_wrapper.h"

/******************************************************
 *                      Macros
 ******************************************************/
//#define BME280_USE_SPI

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/**
 * Print the sensor data read from the BME280.
 *
 * @param[in] comp_data : Pointer to the compensated BME280 data to print out
 *
 * @return void
 */
static void print_sensor_data(struct bme280_data *comp_data);

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
	wiced_result_t wres;
	int8_t bme_rslt;

	struct bme280_dev dev_bme280;
	struct bme280_data comp_data;
	uint8_t settings_sel;
	uint16_t typ_meas_time;

    /* Initialise the WICED device */
    wiced_init();

    WPRINT_APP_INFO( ( "--- BME280 Temperature, Humidity, and Pressure Sensor Snippet ---\n" ) );

#ifndef BME280_USE_SPI
    wres = bme280_wiced_init_i2c(&dev_bme280, BME280_I2C, BME280_I2C_ADDR_PRIM);
#else
    wres = bme280_wiced_init_spi(&dev_bme280, BME280_SPI, BME280_SPI_CS);
#endif

	if(wres != WICED_SUCCESS){
		WPRINT_APP_INFO( ( "BME280 successfully initialized.\n") );
	}
	else{
		WPRINT_APP_INFO( ( "Error %u while initializing BME280!\n", (unsigned)wres ) );
	}

	/* BME280 datasheet recommended mode of operation: Indoor navigation */
	dev_bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev_bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev_bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev_bme280.settings.filter = BME280_FILTER_COEFF_16;

	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

	if((bme_rslt = bme280_set_sensor_settings(settings_sel, &dev_bme280)) != BME280_OK){
		WPRINT_APP_INFO(("Error %d while configuring BME280!\n", bme_rslt));
	}

	typ_meas_time = bme280_wiced_get_meas_time(&dev_bme280);
	WPRINT_APP_INFO(("Typical measurement time for current settings: %ums\n", (unsigned)typ_meas_time));

	/* One-shot read of temperature, humidity, and pressure */
	if((bme_rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev_bme280)) == BME280_OK){
		wiced_rtos_delay_milliseconds(typ_meas_time);
		bme280_get_sensor_data(BME280_ALL, &comp_data, &dev_bme280);
		WPRINT_APP_INFO(("One-Shot Forced Measurement: "));
		print_sensor_data(&comp_data);
	}
	else{
		WPRINT_APP_INFO(("Error %d setting BME280 sensor mode!\n", bme_rslt));
	}

	/* Start periodic measurements */
	dev_bme280.settings.standby_time = BME280_STANDBY_TIME_500_MS;
	settings_sel = BME280_STANDBY_SEL;
	if((bme_rslt = bme280_set_sensor_settings(settings_sel, &dev_bme280)) != BME280_OK){
		WPRINT_APP_INFO(("Error %d while configuring BME280!\n", bme_rslt));
	}
	bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev_bme280);
	wiced_rtos_delay_milliseconds(typ_meas_time);

    while ( 1 )
    {
    	if((bme_rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev_bme280)) == BME280_OK){
			WPRINT_APP_INFO(("Normal Mode Measurement: "));
			print_sensor_data(&comp_data);
		}
		else{
			WPRINT_APP_INFO(("Error %d reading BME280 sensor data!\n", bme_rslt));
		}
    	/* Delay for t_meas (40ms) + t_standby (500ms) */
    	wiced_rtos_delay_milliseconds(500+typ_meas_time);
    }
}


static void print_sensor_data(struct bme280_data *comp_data)
{
	WPRINT_APP_INFO(("Temperature = %.2f\xf8""C, Humidity = %.2f%%, Pressure = %.2fPa\n", comp_data->temperature, comp_data->humidity, comp_data->pressure));
}



