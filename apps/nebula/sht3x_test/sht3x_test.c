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
 * SHT3x API Application
 *
 * This application demonstrates how to use the SHT3x library API
 * to read temperature and humidity
 *
 * Features demonstrated
 *  - SHT3x API
 *
 */

#include "wiced.h"
#include "sht3x.h"

/******************************************************
 *                      Macros
 ******************************************************/

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
typedef struct{
    float high_set;   /**< Alert limit high set */
    float high_clear; /**< Alert limit high clear */
    float low_clear;  /**< Alert limit low clear */
    float low_set;    /**< Alert limit low set */
} alert_limits_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
	wiced_result_t wres;
	uint32_t serial_num = 0;
	uint16_t status = 0;
	float temperature, humidity;
	alert_limits_t temp_limits, humidity_limits;

    sht3x_device_t sht3x_dev = {
    		.i2c_dev = {
    				.port = PLATFORM_MIKRO_I2C,
					.address = SHT3X_DEFAULT_I2C_ADDR,
					.address_width = I2C_ADDRESS_WIDTH_7BIT,
					.flags         = 0x00,
					.speed_mode    = I2C_HIGH_SPEED_MODE,
    		},
			.disable_dma = WICED_TRUE,
			.read_mode = SHT3X_MODE_POLLING
    };

    /* Initialise the WICED device */
    wiced_init();

    WPRINT_APP_INFO( ( "--- SHT3X Temperature and Humidity Sensor Snippet ---\n" ) );

    sht3x_hard_reset(PLATFORM_MIKRO_RST, WICED_FALSE);

    wiced_i2c_init(&sht3x_dev.i2c_dev);

	if((wres = sht3x_read_serial_number(&sht3x_dev, &serial_num)) == WICED_SUCCESS){
		WPRINT_APP_INFO( ( "Serial number is %08X\n", (unsigned)serial_num ) );
	}
	else{
		WPRINT_APP_INFO( ( "Error %u while reading serial number!\n", (unsigned)wres ) );
	}

	if((wres = sht3x_clear_alert_flags(&sht3x_dev)) != WICED_SUCCESS){
		WPRINT_APP_INFO( ( "Error %u while clearing alert flags!\n", (unsigned)wres ) );
	}

	/* Read alert limits */
	if((wres = sht3x_get_alert_limits(&sht3x_dev, &temp_limits.high_set, &humidity_limits.high_set, &temp_limits.high_clear, &humidity_limits.high_clear,
			&temp_limits.low_clear, &humidity_limits.low_clear, &temp_limits.low_set, &humidity_limits.low_set)) == WICED_SUCCESS){
		WPRINT_APP_INFO( ( "Temperature Alerts: High Set = %.1f\xf8""C, High Clear = %.1f\xf8""C, Low Clear = %.1f\xf8""C, Low Set = %.1f\xf8""C\n",
				temp_limits.high_set, temp_limits.high_clear, temp_limits.low_clear, temp_limits.low_set ) );
		WPRINT_APP_INFO( ( "Humidity Alerts: High Set = %.1f%%, High Clear = %.1f%%, Low Clear = %.1f%%, Low Set = %.1f%%\n",
				humidity_limits.high_set, humidity_limits.high_clear, humidity_limits.low_clear, humidity_limits.low_set ) );
	}
	else{
		WPRINT_APP_INFO( ( "Error %u while reading alert limits!\n", (unsigned)wres ) );
	}

	/* One-shot read of temperature and humidity */
	if((wres = sht3x_get_temp_humdity(&sht3x_dev, &temperature, &humidity, SHT3X_REPEATAB_HIGH)) == WICED_SUCCESS){
		WPRINT_APP_INFO( ( "One-shot temperature = %.1f\xf8""C, humidity = %.1f%%\n", temperature, humidity ) );
	}
	else{
		WPRINT_APP_INFO( ( "Error %u while reading temperature and humidity!\n", (unsigned)wres ) );
	}

	if((wres = sht3x_read_status(&sht3x_dev, &status)) == WICED_SUCCESS){
		WPRINT_APP_INFO( ( "Status: 0x%04X\n", (unsigned)status ) );
	}
	else{
		WPRINT_APP_INFO( ( "Error %u while reading status!\n", (unsigned)wres ) );
	}

	if((wres = sht3x_set_heater(&sht3x_dev, WICED_FALSE)) == WICED_SUCCESS){
		WPRINT_APP_INFO( ( "Set heater OFF.\n" ) );
	}
	else{
		WPRINT_APP_INFO( ( "Error %u while setting heater!\n", (unsigned)wres ) );
	}

	if((wres = sht3x_read_status(&sht3x_dev, &status)) == WICED_SUCCESS){
		WPRINT_APP_INFO( ( "Status: 0x%04X\n", (unsigned)status ) );
	}
	else{
		WPRINT_APP_INFO( ( "Error %u while reading status!\n", (unsigned)wres ) );
	}

	/* Start periodic measurements */
	sht3x_start_periodic(&sht3x_dev, SHT3X_REPEATAB_HIGH, SHT3X_FREQUENCY_1HZ);

    while ( 1 )
    {
    	wiced_rtos_delay_milliseconds( 1000 );

    	if((wres = sht3x_read_status(&sht3x_dev, &status)) != WICED_SUCCESS){
    		WPRINT_APP_INFO( ( "Error %u while reading status!\n", (unsigned)wres ) );
		}

    	if((wres = sht3x_read_measurement_buffer(&sht3x_dev, &temperature, &humidity)) == WICED_SUCCESS){
			WPRINT_APP_INFO( ( "Temperature = %.1f\xf8""C, Humidity = %.1f%%, Status = 0x%04X\n", temperature, humidity, status ) );
		}
		else{
			WPRINT_APP_INFO( ( "Error %u while reading temperature and humidity buffer!\n", (unsigned)wres ) );
		}
    }
}
