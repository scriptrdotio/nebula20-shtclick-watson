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
 *  Defines functions to use SHT3x driver
 */

#ifndef LIBRARIES_DRIVERS_SENSORS_SHT3X_SHT3X_H_
#define LIBRARIES_DRIVERS_SENSORS_SHT3X_SHT3X_H_

#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                    Constants
 ******************************************************/

/** Default SHT3x I2C address */
#define SHT3X_DEFAULT_I2C_ADDR		(0x44)
/**Alternative SHT3x I2C address if ADDR pin is logic high */
#define SHT3X_ALTERNATE_I2C_ADDR	(0x45)

/** Timeout in milliseconds when polling for single shot measurement results */
#define SHT3X_T_RH_MEASURE_TIMEOUT	(50)

/** Write data checksum status */
#define SHT3X_STAT_CRC_STAT			(0x0001)
/** Command Status */
#define SHT3X_STAT_CMD_STAT			(0x0002)
/** System reset detected */
#define SHT3X_STAT_RESET_DETECTED	(0x0010)
/** Temperature tracking alert */
#define SHT3X_STAT_T_ALERT			(0x0400)
/** Humidity tracking alert */
#define SHT3X_STAT_RH_ALERT			(0x0800)
/** Heater status */
#define SHT3X_STAT_HEATER_STAT		(0x2000)
/** Alert pending status */
#define SHT3X_STAT_ALERT_PEND		(0x8000)

/** Delay before retrying I2C comms */
#define SHT3X_RETRY_DELAY_MS        (2)
/** Delay after doing a hard reset */
#define SHT3X_HARD_RESET_DELAY_MS   (5)
/** Delay after doing a soft reset */
#define SHT3X_SOFT_RESET_DELAY_MS   (2)

/******************************************************
 *                   Enumerations
 ******************************************************/

/** SHT3x Sensor Commands */
typedef enum{
	SHT3X_CMD_READ_SERIALNBR  = 0x3780, /**< read serial number */
	SHT3X_CMD_READ_STATUS     = 0xF32D, /**< read status register */
	SHT3X_CMD_CLEAR_STATUS    = 0x3041, /**< clear status register */
	SHT3X_CMD_HEATER_ENABLE   = 0x306D, /**< enabled heater */
	SHT3X_CMD_HEATER_DISABLE  = 0x3066, /**< disable heater */
	SHT3X_CMD_SOFT_RESET      = 0x30A2, /**< soft reset */
	SHT3X_CMD_MEAS_CLOCKSTR_H = 0x2C06, /**< measurement: clock stretching, high repeatability */
	SHT3X_CMD_MEAS_CLOCKSTR_M = 0x2C0D, /**< measurement: clock stretching, medium repeatability */
	SHT3X_CMD_MEAS_CLOCKSTR_L = 0x2C10, /**< measurement: clock stretching, low repeatability */
	SHT3X_CMD_MEAS_POLLING_H  = 0x2400, /**< measurement: polling, high repeatability */
	SHT3X_CMD_MEAS_POLLING_M  = 0x240B, /**< measurement: polling, medium repeatability */
	SHT3X_CMD_MEAS_POLLING_L  = 0x2416, /**< measurement: polling, low repeatability */
	SHT3X_CMD_MEAS_PERI_05_H  = 0x2032, /**< measurement: periodic 0.5 mps, high repeatability */
	SHT3X_CMD_MEAS_PERI_05_M  = 0x2024, /**< measurement: periodic 0.5 mps, medium repeatability */
	SHT3X_CMD_MEAS_PERI_05_L  = 0x202F, /**< measurement: periodic 0.5 mps, low repeatability */
	SHT3X_CMD_MEAS_PERI_1_H   = 0x2130, /**< measurement: periodic 1 mps, high repeatability */
	SHT3X_CMD_MEAS_PERI_1_M   = 0x2126, /**< measurement: periodic 1 mps, medium repeatability */
	SHT3X_CMD_MEAS_PERI_1_L   = 0x212D, /**< measurement: periodic 1 mps, low repeatability */
	SHT3X_CMD_MEAS_PERI_2_H   = 0x2236, /**< measurement: periodic 2 mps, high repeatability */
	SHT3X_CMD_MEAS_PERI_2_M   = 0x2220, /**< measurement: periodic 2 mps, medium repeatability */
	SHT3X_CMD_MEAS_PERI_2_L   = 0x222B, /**< measurement: periodic 2 mps, low repeatability */
	SHT3X_CMD_MEAS_PERI_4_H   = 0x2334, /**< measurement: periodic 4 mps, high repeatability */
	SHT3X_CMD_MEAS_PERI_4_M   = 0x2322, /**< measurement: periodic 4 mps, medium repeatability */
	SHT3X_CMD_MEAS_PERI_4_L   = 0x2329, /**< measurement: periodic 4 mps, low repeatability */
	SHT3X_CMD_MEAS_PERI_10_H  = 0x2737, /**< measurement: periodic 10 mps, high repeatability */
	SHT3X_CMD_MEAS_PERI_10_M  = 0x2721, /**< measurement: periodic 10 mps, medium repeatability */
	SHT3X_CMD_MEAS_PERI_10_L  = 0x272A, /**< measurement: periodic 10 mps, low repeatability */
	SHT3X_CMD_FETCH_DATA      = 0xE000, /**< readout measurements for periodic mode */
	SHT3X_CMD_R_AL_LIM_LS     = 0xE102, /**< read alert limits, low set */
	SHT3X_CMD_R_AL_LIM_LC     = 0xE109, /**< read alert limits, low clear */
	SHT3X_CMD_R_AL_LIM_HS     = 0xE11F, /**< read alert limits, high set */
	SHT3X_CMD_R_AL_LIM_HC     = 0xE114, /**< read alert limits, high clear */
	SHT3X_CMD_W_AL_LIM_HS     = 0x611D, /**< write alert limits, high set */
	SHT3X_CMD_W_AL_LIM_HC     = 0x6116, /**< write alert limits, high clear */
	SHT3X_CMD_W_AL_LIM_LC     = 0x610B, /**< write alert limits, low clear */
	SHT3X_CMD_W_AL_LIM_LS     = 0x6100, /**< write alert limits, low set */
	SHT3X_CMD_NO_SLEEP        = 0x303E,
	SHT3X_CMD_BREAK           = 0x3093, /**< break / stop periodic measurements */
} sht3x_command_t;

/** Measurement Repeatability */
typedef enum{
	SHT3X_REPEATAB_HIGH,   /**< high repeatability */
	SHT3X_REPEATAB_MEDIUM, /**< medium repeatability */
	SHT3X_REPEATAB_LOW,    /**< low repeatability */
} sht3x_repeatability_t;

/** Read Measurement Mode */
typedef enum{
	SHT3X_MODE_CLKSTRETCH, /**< clock stretching */
	SHT3X_MODE_POLLING,    /**< polling */
} sht3x_read_mode_t;

/** Periodic Measurement Frequency */
typedef enum{
	SHT3X_FREQUENCY_HZ5,  /**< 0.5 measurements per seconds */
	SHT3X_FREQUENCY_1HZ,  /**< 1.0 measurements per seconds */
	SHT3X_FREQUENCY_2HZ,  /**< 2.0 measurements per seconds */
	SHT3X_FREQUENCY_4HZ,  /**< 4.0 measurements per seconds */
	SHT3X_FREQUENCY_10HZ, /**< 10.0 measurements per seconds */
} sht3x_frequency_t;

/******************************************************
 *                    Structures
 ******************************************************/

/**
 * The SHT3x device details.
 */
typedef struct{
    wiced_i2c_device_t i2c_dev;     /**< I2C device for the SHT3x sensor */
    wiced_bool_t       disable_dma; /**< Whether to disable using the I2C DMA during transfers */
    sht3x_read_mode_t  read_mode;   /**< The read mode to use when doing a single shot measurement */
} sht3x_device_t;

/******************************************************
 *              Function Declarations
 ******************************************************/

/**
 * Reset the SHT3x sensor using the reset pin
 *
 * @param[in] gpio   : The GPIO pin to use
 * @param[in] invert : Invert the normal reset action (normally active low)
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_hard_reset(wiced_gpio_t gpio, wiced_bool_t invert);

/**
 * Reset the SHT3x sensor using the soft reset command
 *
 * @param[in] device : SHT3x device
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_soft_reset(sht3x_device_t* device);

/**
 * Read the serial number of the SHT3x sensor
 *
 * @param[in]  device        : SHT3x device
 * @param[out] serial_number : pointer to the serial number
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_read_serial_number(sht3x_device_t* device, uint32_t* serial_number);

/**
 * Read the SHT3x sensor status register
 *
 * @param[in]  device : SHT3x device
 * @param[out] status : pointer to the status value
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_read_status(sht3x_device_t* device, uint16_t* status);

/**
 * Clear the SHT3x sensor alert flags
 *
 * @param[in] device : SHT3x device
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_clear_alert_flags(sht3x_device_t* device);

/**
 * Get the SHT3x sensor temperature and humidity using a single-shot read
 *
 * @param[in]  device        : SHT3x device
 * @param[out] temperature   : pointer to the temperature value
 * @param[out] humidity      : pointer to the humidity value
 * @param[in]  repeatability : the repeatability setting to use
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_get_temp_humdity(sht3x_device_t* device, float* temperature, float* humidity, sht3x_repeatability_t repeatability);

/**
 * Start measuring temperature and humidity in periodic data acquisition mode
 *
 * @param[in] device        : SHT3x device
 * @param[in] repeatability : the repeatability setting to use
 * @param[in] frequency     : the frequency of the periodic measurements
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_start_periodic(sht3x_device_t* device, sht3x_repeatability_t repeatability, sht3x_frequency_t frequency);

/**
 * Stop measuring temperature and humidity in periodic data acquisition mode
 *
 * @param[in] device : SHT3x device
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_stop_periodic(sht3x_device_t* device);

/**
 * Read the measurement buffer that contains the latest temperature and humidity values
 * obtained from the periodic data acquisition
 *
 * @param[in]  device        : SHT3x device
 * @param[out] temperature   : pointer to the temperature value
 * @param[out] humidity      : pointer to the humidity value
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_read_measurement_buffer(sht3x_device_t* device, float* temperature, float* humidity);

/**
 * Set the SHT3x internal heater on or off
 *
 * @param[in] device : SHT3x device
 * @param[in] enable : enable or disable the heater
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_set_heater(sht3x_device_t* device, wiced_bool_t enable);

/**
 * Set temperature and humidity alert limits
 *
 * @param[in] device              : SHT3x device
 * @param[in] temp_high_set       : high temperature alert set value
 * @param[in] humidity_high_set   : high humidity alert set value
 * @param[in] temp_high_clear     : high temperature alert clear value
 * @param[in] humidity_high_clear : high humidity alert clear value
 * @param[in] temp_low_clear      : low temperature alert clear value
 * @param[in] humidity_low_clear  : low humidity alert clear value
 * @param[in] temp_low_set        : low temperature alert set value
 * @param[in] humidity_low_set    : low humidity alert set value
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_set_alert_limits(sht3x_device_t* device, float temp_high_set, float humidity_high_set, float temp_high_clear, float humidity_high_clear, float temp_low_clear, float humidity_low_clear, float temp_low_set, float humidity_low_set);

/**
 * Get temperature and humidity alert limits
 *
 * @param[in] device              : SHT3x device
 * @param[in] temp_high_set       : pointer to high temperature alert set value
 * @param[in] humidity_high_set   : pointer to high humidity alert set value
 * @param[in] temp_high_clear     : pointer to high temperature alert clear value
 * @param[in] humidity_high_clear : pointer to high humidity alert clear value
 * @param[in] temp_low_clear      : pointer to low temperature alert clear value
 * @param[in] humidity_low_clear  : pointer to low humidity alert clear value
 * @param[in] temp_low_set        : pointer to low temperature alert set value
 * @param[in] humidity_low_set    : pointer to low humidity alert set value
 *
 * @return @ref wiced_result_t
 */
wiced_result_t sht3x_get_alert_limits(sht3x_device_t* device, float* temp_high_set, float* humidity_high_set, float* temp_high_clear, float* humidity_high_clear, float* temp_low_clear, float* humidity_low_clear, float* temp_low_set, float* humidity_low_set);

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* LIBRARIES_DRIVERS_SENSORS_SHT3X_SHT3X_H_ */
