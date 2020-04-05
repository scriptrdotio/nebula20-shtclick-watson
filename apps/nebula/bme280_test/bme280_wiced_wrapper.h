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
 *  Defines functions to get the BME280 driver from Bosch to work with WICED.
 */

#ifndef APPS_SNIP_BME280_TEST_BME280_WICED_WRAPPER_H_
#define APPS_SNIP_BME280_TEST_BME280_WICED_WRAPPER_H_

#include "wiced.h"
#include "bme280.h"

#define BME280_I2C_DISABLE_DMA (WICED_FALSE)

/**
 * Initialize the BME280 with I2C communications.
 *
 * @param[in] dev      : The BME280 device
 * @param[in] i2c_port : The I2C port to use for the device
 * @param[in] i2c_addr : The I2C address to use for the device
 *
 * @return @ref wiced_result_t
 */
wiced_result_t bme280_wiced_init_i2c(struct bme280_dev *dev, wiced_i2c_t i2c_port, uint8_t i2c_addr);

/**
 * Initialize the BME280 with SPI communications.
 *
 * @param[in] dev         : The BME280 device
 * @param[in] spi_port    : The SPI port to use for the device
 * @param[in] chip_select : The GPIO to use for the chip select pin of the device
 *
 * @return @ref wiced_result_t
 */
wiced_result_t bme280_wiced_init_spi(struct bme280_dev *dev, wiced_spi_t spi_port, wiced_gpio_t chip_select);

/**
 * Calculate the typical measurement time based on the current device settings. According to the
 * datasheet, t_meas,typ = 1 + [2*T_oversampling] + [2*P_oversampling + 0.5] + [2*H_oversampling + 0.5].
 *
 * @param[in] dev : The BME280 device
 *
 * @return The calculated typical measurement time in milliseconds
 */
uint16_t bme280_wiced_get_meas_time(const struct bme280_dev *dev);



#endif /* APPS_SNIP_BME280_TEST_BME280_WICED_WRAPPER_H_ */
