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
 *  Implements functions to use the MLX90316.
 */

#include "mlx90316.h"

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/**
 * Execute the MLX90316 bit-bang SPI communications.
 *
 * @param[in]  dev : The MLX90316 Bit-Bang device
 * @param[out] out : The byte to send to the device
 * @param[in]  in  : Pointer to the data read from the device
 *
 * @return @ref wiced_result_t
 */
static wiced_result_t mlx90316_bb_xfer(mlx90316_bb_device_t *dev, uint8_t out, uint8_t *in);

/**
 * Read one data frame from the bit-bang device.
 *
 * @param[in] dev  : The MLX90316 Bit-Bang device
 * @param[in] data : Pointer to the data read from the device
 *
 * @return @ref wiced_result_t
 */
static wiced_result_t mlx90316_bb_read_data_frame(mlx90316_bb_device_t *dev, uint16_t *data);

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t mlx90316_read(wiced_spi_device_t *dev, uint16_t *raw_angle, uint16_t *mlx_err)
{
	wiced_result_t wres;
	wiced_spi_message_segment_t msg;
	uint8_t tx_buf[6] = {0xAA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	uint8_t rx_buf[6] = {0x00};
	uint16_t dataval;
	uint16_t invdataval;

	if(raw_angle == NULL){
        return WICED_BADARG;
    }

    if(mlx_err != NULL){
        *mlx_err = MLX_ERR_OK;
    }

	msg.length    = sizeof(tx_buf);
	msg.tx_buffer = tx_buf;
	msg.rx_buffer = (uint8_t*)rx_buf;

	if((wres = wiced_spi_transfer(dev, &msg, 1 )) != WICED_SUCCESS){
		return wres;
	}

	dataval = ((uint16_t)rx_buf[2])<<8;
	dataval |= ((uint16_t)rx_buf[3]);

	/* Two data bytes are followed by two inverted data bytes. The data is valid if they align. */
	invdataval = ((uint16_t)rx_buf[4])<<8;
	invdataval |= ((uint16_t)rx_buf[5]);

	if(dataval != ((uint16_t)~invdataval)){
		return WICED_PACKET_BUFFER_CORRUPT;
	}

	if((dataval & MLX90316_DATA_TYPE_MASK) != MLX90316_DATA_TYPE_ANGLE){
        if(mlx_err != NULL){
            *mlx_err = dataval;
        }
        return WICED_ERROR;
    }

	*raw_angle = dataval >> 2;

	return WICED_SUCCESS;
}


float mlx90316_raw_angle_to_float(uint16_t raw_angle)
{
	float angle = (float)raw_angle;
	/* 2^14/360.0 = 45.511 */
	angle /= 45.511;
	return angle;
}

wiced_result_t mlx90316_bb_init(mlx90316_bb_device_t *dev)
{
    uint16_t dataframe;

	if(dev == NULL){
		return WICED_BADARG;
	}

	wiced_gpio_init(dev->cs, OUTPUT_PUSH_PULL);
	wiced_gpio_output_high(dev->cs);
	wiced_gpio_init(dev->sck, OUTPUT_PUSH_PULL);
	wiced_gpio_output_low(dev->sck);
	wiced_gpio_init(dev->mosi, OUTPUT_PUSH_PULL);
	wiced_gpio_output_low(dev->mosi);
	wiced_gpio_init(dev->miso, INPUT_HIGH_IMPEDANCE);

	wiced_rtos_delay_milliseconds(3);

	// Make sure we can receive a data frame in the proper format
	return mlx90316_bb_read_data_frame(dev, &dataframe);
}

static wiced_result_t mlx90316_bb_xfer(mlx90316_bb_device_t *dev, uint8_t out, uint8_t *in)
{
	uint8_t i;
	if(dev == NULL){
		return WICED_BADARG;
	}

	*in = 0;
	for(i=0; i<8; i++)
	{
		if(out & 0x80){
			wiced_gpio_output_high(dev->mosi);
		}
		else{
			wiced_gpio_output_low(dev->mosi);
		}

		out = (out << 1);
		*in = (*in << 1);

		wiced_gpio_output_high(dev->sck);
		wiced_rtos_delay_microseconds(MLX90316_BB_CLK_HIGH_DELAY_US);
		if(wiced_gpio_input_get(dev->miso) == WICED_TRUE){
			*in |= 0x01;
		}
		wiced_gpio_output_low(dev->sck);
		wiced_rtos_delay_microseconds(MLX90316_BB_CLK_LOW_DELAY_US);
	}

	wiced_gpio_output_low(dev->mosi);

	return WICED_SUCCESS;
}


static wiced_result_t mlx90316_bb_read_data_frame(mlx90316_bb_device_t *dev, uint16_t *data)
{
    uint8_t pos = 0;
    uint8_t rx_buf[6] = {0x00};
    uint16_t dataval;
    uint16_t invdataval;

    if(dev == NULL || data == NULL){
        return WICED_BADARG;
    }

    wiced_gpio_output_low(dev->cs);
    wiced_rtos_delay_microseconds(MLX90316_BB_CS_DELAY_US);

    mlx90316_bb_xfer(dev, 0xAA, &rx_buf[0]);

    for(pos=1; pos<sizeof(rx_buf); pos++){
        wiced_rtos_delay_microseconds(MLX90316_BB_BYTE_DELAY_US);
        mlx90316_bb_xfer(dev, 0xFF, &rx_buf[pos]);
    }
    wiced_gpio_output_high(dev->cs);

    dataval = ((uint16_t)rx_buf[2])<<8;
    dataval |= ((uint16_t)rx_buf[3]);

    /* Two data bytes are followed by two inverted data bytes. The data is valid if they align. */
    invdataval = ((uint16_t)rx_buf[4])<<8;
    invdataval |= ((uint16_t)rx_buf[5]);

    if(dataval != ((uint16_t)~invdataval)){
        return WICED_PACKET_BUFFER_CORRUPT;
    }

    *data = dataval;

    return WICED_SUCCESS;
}


wiced_result_t mlx90316_bb_read(mlx90316_bb_device_t *dev, uint16_t *raw_angle, uint16_t *mlx_err)
{
	wiced_result_t wres;
	uint16_t dataval;

	if(raw_angle == NULL){
        return WICED_BADARG;
    }

	if(mlx_err != NULL){
	    *mlx_err = MLX_ERR_OK;
	}

	if((wres = mlx90316_bb_read_data_frame(dev, &dataval)) != WICED_SUCCESS){
	    return wres;
	}

	if((dataval & MLX90316_DATA_TYPE_MASK) != MLX90316_DATA_TYPE_ANGLE){
	    if(mlx_err != NULL){
	        *mlx_err = dataval;
	    }
		return WICED_ERROR;
	}

	*raw_angle = dataval >> 2;

	return WICED_SUCCESS;
}

