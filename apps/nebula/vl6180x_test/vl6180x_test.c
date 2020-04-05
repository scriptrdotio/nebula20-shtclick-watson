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
 * VL6180X API Application
 *
 * This application demonstrates how to use the VL6180X API
 * to sense ambient light and object proximity
 *
 * Features demonstrated
 *  - VL6180X API
 *
 */

#include "wiced.h"
#include "vl6180x_api.h"

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
	const wiced_i2c_device_t vl6180x_i2c = {
		.port          = PLATFORM_MIKRO_I2C,
		.address       = VL6180X_DEFAULT_I2C_ADDR,
		.address_width = I2C_ADDRESS_WIDTH_7BIT,
		.flags         = 0,
		.speed_mode    = I2C_HIGH_SPEED_MODE,
	};

	struct wiced_vl6180x_dev_t vl6180x_dev;
	VL6180x_RangeData_t range;
	VL6180x_AlsData_t als;
	int retries;

    /* Initialise the WICED device */
    wiced_init();

    wiced_i2c_init(&vl6180x_i2c);
    vl6180x_dev.i2c_dev = &vl6180x_i2c;

    /* Allow the VL6180X to start up */
    wiced_rtos_delay_milliseconds(2);
    /* Init VL6180X */
    VL6180x_InitData(&vl6180x_dev);
	VL6180x_Prepare(&vl6180x_dev);

    WPRINT_APP_INFO( ( "--- VL6180X Proximity and Ambient Light Sensor Snippet ---\n" ) );

    while ( 1 )
    {
        /* Read the ambient light sensor data */
    	VL6180x_AlsPollMeasurement(&vl6180x_dev, &als);
		if (als.errorStatus == 0 ){
			WPRINT_APP_INFO( ( "Ambient Light = %u lux\n", (unsigned)als.lux ) );
		}
		else{
			WPRINT_APP_INFO( ( "Error reading ambient light! Err = %d\n", (int)als.errorStatus ) );
		}

		retries = 5;
		do{
            /* Read the range data */
            VL6180x_RangePollMeasurement(&vl6180x_dev, &range);
            VL6180x_AutoScaleRange(&vl6180x_dev, &range);
		}while((range.errorStatus == Raw_Ranging_Algo_Overflow || range.errorStatus == RangingFiltered) && retries--);

		if (range.errorStatus == 0 ){
			WPRINT_APP_INFO( ( "Range = %d mm\n", (int)range.range_mm ) );
		}
		else{
			if(range.errorStatus == Early_Convergence_Estimate || range.errorStatus == Max_Convergence || range.errorStatus == Max_Signal_To_Noise_Ratio){
				WPRINT_APP_INFO( ( "Nothing within detectable range.\n" ) );
			}
			else if(range.errorStatus == Raw_Ranging_Algo_Overflow || range.errorStatus == RangingFiltered){
				WPRINT_APP_INFO( ( "Need to autoscale range!\n" ) );
			}
			else{
				WPRINT_APP_INFO( ( "Error reading range! Err = %d\n", (int)range.errorStatus ) );
			}
		}

        wiced_rtos_delay_milliseconds( 250 );
    }
}
