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
 *  Implements functions to use the VL6180x with the Bluemix project.
 */

#include "bluemix_vl6180x.h"
#include "vl6180x_api.h"

/******************************************************
 *               Variable Definitions
 ******************************************************/

/** VL6180x device */
static struct wiced_vl6180x_dev_t vl6180x_dev;

/** VL6180x I2C device */
const wiced_i2c_device_t vl6180x_i2c = {
	.port          = VL6180X_I2C,
	.address       = VL6180X_DEFAULT_I2C_ADDR,
	.address_width = I2C_ADDRESS_WIDTH_7BIT,
	.flags         = 0,
	.speed_mode    = I2C_HIGH_SPEED_MODE,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t init_vl6180x(void)
{
	int vl_res = 0;
	wiced_i2c_init(&vl6180x_i2c);
	vl6180x_dev.i2c_dev = &vl6180x_i2c;

	if(wiced_i2c_probe_device(&vl6180x_i2c, 4) != WICED_TRUE){
		WPRINT_APP_INFO( ("No VL6180x Proximity and Ambient Light Sensor\n" ));
		return WICED_ERROR;
	}

	WPRINT_APP_INFO(("Found VL6180x Proximity and Ambient Light Sensor, initializing...\n" ));

	if((vl_res = VL6180x_InitData(&vl6180x_dev)) < 0){
		WPRINT_APP_INFO(("Error %d while initializing VL6180x!\n", vl_res));
		return WICED_ERROR;
	}

	if((vl_res = VL6180x_Prepare(&vl6180x_dev)) != 0){
		WPRINT_APP_INFO(("Error %d while preparing VL6180x!\n", vl_res));
		return WICED_ERROR;
	}

	return WICED_SUCCESS;
}

wiced_result_t read_vl6180x(char *mqtt_msg, wiced_bool_t first_data)
{
	/** Range sensor data */
	VL6180x_RangeData_t range;
	/** Ambient light sensor data */
	VL6180x_AlsData_t als;
	int retries = 0;
	uint16_t msg_len, start_len;
	int add_pos;

	start_len = msg_len = strlen(mqtt_msg);

	/* Read the ambient light sensor data */
	VL6180x_AlsPollMeasurement(&vl6180x_dev, &als);
	if (als.errorStatus == 0 ){
		WPRINT_APP_INFO( ( " VL6180x Ambient Light: %u lux\n", (unsigned)als.lux ) );
		if(!first_data){
			strcat(mqtt_msg, ",");
			msg_len++;
		}
		if((add_pos = sprintf(&mqtt_msg[msg_len], "\"vl6180x_amb_light\":%u",(unsigned)als.lux)) >= 0){
			msg_len += add_pos;
		}

		first_data = WICED_FALSE;
	}
	else{
		WPRINT_APP_INFO( ( " Error reading VL6180x ambient light! Err = %d\n", (int)als.errorStatus ) );
	}

	/* Read the range data. May need to adjust auto-scale and re-read. */
	retries = 0;
	do
	{
		VL6180x_RangePollMeasurement(&vl6180x_dev, &range);
		VL6180x_AutoScaleRange(&vl6180x_dev, &range);
		if (range.errorStatus == 0 ){
			WPRINT_APP_INFO( ( " VL6180x Range: %d mm\n", (int)range.range_mm ) );
			if(!first_data){
				strcat(mqtt_msg, ",");
				msg_len++;
			}
			if((add_pos = sprintf(&mqtt_msg[msg_len], "\"vl6180x_range_mm\":%d",(int)range.range_mm)) >= 0){
				msg_len += add_pos;
			}
		}
		else{
			if(range.errorStatus == Early_Convergence_Estimate || range.errorStatus == Max_Convergence || range.errorStatus == Max_Signal_To_Noise_Ratio){
				WPRINT_APP_INFO( ( " VL6180x Range: nothing within detectable range (-1)\n" ) );
				if(!first_data){
					strcat(mqtt_msg, ",");
					msg_len++;
				}
				if((add_pos = sprintf(&mqtt_msg[msg_len], "\"vl6180x_range_mm\":-1")) >= 0){
					msg_len += add_pos;
				}
			}
			else if(range.errorStatus == Raw_Ranging_Algo_Overflow || range.errorStatus == RangingFiltered){
				WPRINT_APP_INFO( ( " VL6180x Need to autoscale range!\n" ) );
			}
			else{
				WPRINT_APP_INFO( ( " Error reading VL6180x range! Err = %d\n", (int)range.errorStatus ) );
			}
		}
	}while((range.errorStatus == Raw_Ranging_Algo_Overflow || range.errorStatus == RangingFiltered) && ++retries<4);

	if(start_len == msg_len){
		return WICED_ERROR;
	}

	return WICED_SUCCESS;
}
