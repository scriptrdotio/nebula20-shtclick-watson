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
 *  Implements functions to use the MLX90316 with the Bluemix project.
 */

#include "bluemix_mlx90316.h"

#ifndef SENSORS_DISABLE_MLX90316
#include "mlx90316.h"

/** MLX90316 bit-bang device */
mlx90316_bb_device_t mlx_bb_dev = {
	.sck = MLX90316_BB_SCK,
	.mosi = MLX90316_BB_MOSI,
	.miso = MLX90316_BB_MISO,
	.cs = MLX90316_BB_CS
};

wiced_result_t init_mlx90316(void)
{
	wiced_result_t wres;

	if((wres = mlx90316_bb_init(&mlx_bb_dev)) != WICED_SUCCESS){
		WPRINT_APP_INFO( ( "No MLX90316 bit-banged device detected (%u).\n", (unsigned)wres ) );
		return WICED_ERROR;
	}

	WPRINT_APP_INFO(("MLX90316 BB device initialized successfully.\n" ));

	return wres;
}

wiced_result_t read_mlx90316(char *mqtt_msg, wiced_bool_t first_data)
{
	wiced_result_t wres;
	uint16_t raw_angle;
	uint16_t mlx_err;
	float angle;
	uint16_t msg_len;

	msg_len = strlen(mqtt_msg);

	mlx_err = MLX_ERR_OK;
	if((wres = mlx90316_bb_read(&mlx_bb_dev, &raw_angle, &mlx_err)) == WICED_SUCCESS)
	{
		angle = mlx90316_raw_angle_to_float(raw_angle);
		WPRINT_APP_INFO((" MLX90316 Angle = %.3f\xf8""\n", angle));
		if(!first_data){
			strcat(mqtt_msg, ",");
			msg_len++;
		}
		sprintf(&mqtt_msg[msg_len], "\"mlx90316_angle\":%.3f", angle);
	}
	else{
		WPRINT_APP_INFO((" Error %u reading MLX90316 sensor data! MLX90316 reported error: 0x%04X\n", (unsigned)wres, (unsigned)mlx_err));
		return wres;
	}

	return WICED_SUCCESS;
}

#else

wiced_result_t init_mlx90316(void)
{
	return WICED_UNSUPPORTED;
}

wiced_result_t read_mlx90316(char *mqtt_msg, wiced_bool_t first_data)
{
	UNUSED_PARAMETER(mqtt_msg);
	UNUSED_PARAMETER(first_data);
	return WICED_UNSUPPORTED;
}

#endif /* ifndef SENSORS_DISABLE_MLX90316 */
