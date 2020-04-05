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
 * MLX90316 API Application
 *
 * This application demonstrates how to use the MLX90316 library API
 * to read rotary position.
 *
 * Features demonstrated
 *  - MLX90316 API
 *
 */

#include "wiced.h"
#include "mlx90316.h"

/******************************************************
 *                    Constants
 ******************************************************/
#define MLX90316_USE_BB

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
	wiced_result_t wres;
	uint16_t raw_angle;
	uint16_t mlx_err;
	float angle;

#ifdef MLX90316_USE_BB
	mlx90316_bb_device_t mlx_bb_dev = {
		.sck = WICED_GPIO_34,
		.mosi = WICED_GPIO_37,
		.miso = WICED_GPIO_36,
		.cs = PLATFORM_MIKRO_SPI_CS
	};
#else
	wiced_spi_device_t mlx_dev = {
		.port        = PLATFORM_MIKRO_SPI,
		.chip_select = PLATFORM_MIKRO_SPI_CS,
		.speed       = MLX90316_MAX_SPI_CLK,
		.mode        = (SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_HIGH | SPI_NO_DMA | SPI_MSB_FIRST),
		.bits        = 8
	};
#endif

    /* Initialise the WICED device */
    wiced_init();

    WPRINT_APP_INFO( ( "--- MLX90316 Rotary Position Sensor Snippet ---\n" ) );

#ifdef MLX90316_USE_BB
    if((wres = mlx90316_bb_init(&mlx_bb_dev)) != WICED_SUCCESS){
		WPRINT_APP_INFO( ( "Error %u while initializing MLX90316 SPI!\n", (unsigned)wres ) );
	}

	while ( 1 )
	{
		mlx_err = MLX_ERR_OK;
		if((wres = mlx90316_bb_read(&mlx_bb_dev, &raw_angle, &mlx_err)) == WICED_SUCCESS)
		{
			angle = mlx90316_raw_angle_to_float(raw_angle);
			WPRINT_APP_INFO(("Angle = %.3f\xf8""\n", angle));
		}
		else{
			WPRINT_APP_INFO(("Error %u reading MLX90316 sensor data! MLX90316 reported error: 0x%04X\n", (unsigned)wres, (unsigned)mlx_err));
		}
		wiced_rtos_delay_milliseconds(500);
	}
#else
    if((wres = wiced_spi_init(&mlx_dev)) != WICED_SUCCESS){
    	WPRINT_APP_INFO( ( "Error %u while initializing MLX90316 SPI!\n", (unsigned)wres ) );
	}

    while ( 1 )
    {
    	mlx_err = MLX_ERR_OK;
    	if((wres = mlx90316_read(&mlx_dev, &raw_angle, &mlx_err)) == WICED_SUCCESS)
    	{
    		angle = mlx90316_raw_angle_to_float(raw_angle);
    		WPRINT_APP_INFO(("Angle = %.3f\xf8""\n", angle));
    	}
		else{
			WPRINT_APP_INFO(("Error %u reading MLX90316 sensor data! MLX90316 reported error: 0x%04X\n", (unsigned)wres, (unsigned)mlx_err));
		}
    	wiced_rtos_delay_milliseconds(500);
    }
#endif
}
