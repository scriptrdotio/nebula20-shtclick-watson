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
 *  Defines functions to read various IoT sensors and put the data into a JSON string.
 */

#ifndef APPS_NEBULA_BLUEMIX_IOT_SENSORS_BLUEMIX_SENSORS_H_
#define APPS_NEBULA_BLUEMIX_IOT_SENSORS_BLUEMIX_SENSORS_H_

#include "wiced.h"

/** Disable reporting thermistor temperature */
//#define SENSORS_DISABLE_THERMISTOR

/**
 * Disable MLX90316 sensor communications.
 *
 * The MLX90316 should be disabled if it is not connected since it uses the SPI pins
 * as GPIO (bit-banged) and can cause configuration issues for devices using that SPI port.
 * The SPI communications to the MLX90316 need to be bit-banged because the minimum time between
 * the CS pin going low and the first clock edge is 15us so the max SPI clock speed would
 * need to be 1/15e-6 = 66.67kHz. The Nebula board is clocked too fast to be able to reach
 * SPI speeds that low. The slowest SPI clock speed the Nebula board can reach is
 * 84MHz/256 = 328.125kHz.
 */
#define SENSORS_DISABLE_MLX90316


#define THERMISTOR_ADC       (WICED_THERMISTOR_JOINS_ADC) /**< Handle for Thermistor                   */
#define VL6180X_I2C          (PLATFORM_MIKRO_I2C)         /**< Handle for VL6180X I2C                  */
#define BME280_I2C           (PLATFORM_MIKRO_I2C)         /**< Handle for BME280 I2C                   */
#define SHT3X_I2C            (PLATFORM_MIKRO_I2C)         /**< Handle for SHT3X I2C                    */
#define SHT3X_RESET          (PLATFORM_MIKRO_RST)         /**< SHT3X Reset pin                         */
#define MLX90316_BB_SCK      (WICED_GPIO_34)              /**< Handle for MLX90316 bit-banged SCK pin  */
#define MLX90316_BB_MOSI     (WICED_GPIO_37)              /**< Handle for MLX90316 bit-banged MOSI pin */
#define MLX90316_BB_MISO     (WICED_GPIO_36)              /**< Handle for MLX90316 bit-banged MISO pin */
#define MLX90316_BB_CS       (PLATFORM_MIKRO_SPI_CS)      /**< Handle for MLX90316 bit-banged CS pin   */

/**
 * Initialize the sensors that are connected to the host. This uses the defines above as well as
 * trying to communicate with some sensors to figure out which ones are connected.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sensors_init(void);

/**
 * Read the connected sensors and put their values in the MQTT message string.
 *
 * @param[in] mqtt_msg   : The current MQTT message string
 * @param[in] first_data : Whether or not this is the first set of data in the MQTT string
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sensors_read(char *mqtt_msg, wiced_bool_t first_data);


#endif /* APPS_NEBULA_BLUEMIX_IOT_SENSORS_BLUEMIX_SENSORS_H_ */
