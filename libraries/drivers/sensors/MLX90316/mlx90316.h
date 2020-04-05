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
 *  Defines functions to use the MLX90316.
 */

#ifndef LIBRARIES_DRIVERS_SENSORS_MLX90316_MLX90316_H_
#define LIBRARIES_DRIVERS_SENSORS_MLX90316_MLX90316_H_

#include "wiced.h"

/** Defines whether the device is using fast mode or slow mode (default is fast mode) */
#define MLX90316_FAST_MODE

#ifdef MLX90316_FAST_MODE
	#define MLX90316_MAX_SPI_CLK  			(66000)	/**< Max SPI clock speed in fast mode (byte delay = 15us) */
	#define MLX90316_BB_CS_DELAY_US			(3)		/**< Bit-bang delay (in microseconds) after setting chip select pin */
	#define MLX90316_BB_CLK_HIGH_DELAY_US	(2)		/**< Bit-bang delay (in microseconds) after setting CLK pin high */
	#define MLX90316_BB_CLK_LOW_DELAY_US	(1)		/**< Bit-bang delay (in microseconds) after setting CLK pin low */
	#define MLX90316_BB_BYTE_DELAY_US		(15)	/**< Bit-bang delay (in microseconds) between bytes */
#else
	#define MLX90316_MAX_SPI_CLK  			(22000)	/**< Max SPI clock speed in slow mode (byte delay = 45us) */
	#define MLX90316_BB_CS_DELAY_US			(7)		/**< Bit-bang delay (in microseconds) after setting chip select pin */
	#define MLX90316_BB_CLK_HIGH_DELAY_US	(4)		/**< Bit-bang delay (in microseconds) after setting CLK pin high */
	#define MLX90316_BB_CLK_LOW_DELAY_US	(3)		/**< Bit-bang delay (in microseconds) after setting CLK pin low */
	#define MLX90316_BB_BYTE_DELAY_US		(45)	/**< Bit-bang delay (in microseconds) between bytes */
#endif

#define MLX90316_DATA_TYPE_MASK  (0x0003) /**< Read serial data type mask */
#define MLX90316_DATA_TYPE_ANGLE (0x0001) /**< Angle data type */
#define MLX90316_DATA_TYPE_ERROR (0x0002) /**< Error data type */

/** Maximum raw angle value */
#define MLX90316_MAX_RAW_ANGLE   (16383)

#define MLX_ERR_OK         (0x0000) /**< No error */
#define MLX_ERR_ERROR_BIT  (0x0002) /**< There is some error */
#define MLX_ERR_ADC_FAIL   (0x0004) /**< ADC failure */
#define MLX_ERR_ADC_SAT    (0x0008) /**< ADC saturation (electrical failure or field too strong) */
#define MLX_ERR_RGTOOLOW   (0x0010) /**< Analog gain below trimmed threshold (likely field too weak) */
#define MLX_ERR_MAGTOOLOW  (0x0020) /**< Magnetic field too weak */
#define MLX_ERR_MAGTOOHIGH (0x0040) /**< Magnetic field too strong */
#define MLX_ERR_RGTOOHIGH  (0x0080) /**< Analog gain above trimmed threshold (likely field too strong) */
#define MLX_ERR_ROCLAMP    (0x0200) /**< Analog chain rough offset compensation: clipping */
#define MLX_ERR_MT7V       (0x0400) /**< Device supply VDD greater than 7V */

/**
 * This struct is used when the SPI communications to the MLX90316 device
 * needs to be bit-banged. This could be due to the MLX90316 device needing
 * SPI speeds that are too slow for the MCU to produce.
 */
typedef struct{
	wiced_gpio_t sck;  /**< SPI Clock Pin */
	wiced_gpio_t mosi; /**< SPI MOSI Pin */
	wiced_gpio_t miso; /**< SPI MISO Pin */
	wiced_gpio_t cs;   /**< SPI CS Pin */
} mlx90316_bb_device_t;

/**
 * Read the MLX90316 angle using the initialized SPI device.
 *
 * @param[in] dev       : The MLX90316 SPI device
 * @param[in] raw_angle : The raw angle reported from the device
 * @param[in] mlx_err   : The error reported by the MLX90316
 *
 * @return @ref wiced_result_t
 */
wiced_result_t mlx90316_read(wiced_spi_device_t *dev, uint16_t *raw_angle, uint16_t *mlx_err);

/**
 * Initialize the pins used for bit-banging MLX90316 SPI communications.
 *
 * @param[in] dev : The MLX90316 Bit-Bang device
 *
 * @return @ref wiced_result_t
 */
wiced_result_t mlx90316_bb_init(mlx90316_bb_device_t *dev);

/**
 * Read the MLX90316 angle using bit-bang SPI communications.
 *
 * @param[in] dev       : The MLX90316 Bit-Bang device
 * @param[in] raw_angle : The raw angle reported from the device
 * @param[in] mlx_err   : The error reported by the MLX90316
 *
 * @return @ref wiced_result_t
 */
wiced_result_t mlx90316_bb_read(mlx90316_bb_device_t *dev, uint16_t *raw_angle, uint16_t *mlx_err);

/**
 * Convert the raw angle read from the device to a float angle.
 *
 * @param[in] raw_angle : The raw angle reported from the device
 *
 * @return float The angle as a float
 */
float mlx90316_raw_angle_to_float(uint16_t raw_angle);

#endif /* LIBRARIES_DRIVERS_SENSORS_MLX90316_MLX90316_H_ */
