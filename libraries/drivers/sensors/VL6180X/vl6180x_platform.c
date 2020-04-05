/*
 * vl6180x_platform.c
 *
 *  Created on: May 19, 2017
 *      Author: Kyle Harman
 */

#include "vl6180x_platform.h"
#include "vl6180x_api.h"

const wiced_i2c_device_t vl6180x_default_i2c =
{
    .port          = WICED_I2C_1,
    .address       = 0x29,
    .address_width = I2C_ADDRESS_WIDTH_7BIT,
    .flags         = 0,
    .speed_mode    = I2C_HIGH_SPEED_MODE,
};


int VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t  *buff, uint8_t len)
{
	wiced_i2c_message_t msg;

	wiced_i2c_init_tx_message(&msg, buff, (uint16_t)len, 1, WICED_FALSE);
	/* Write data to the I2C device */
	if (wiced_i2c_transfer(dev->i2c_dev, &msg, 1) != WICED_SUCCESS)
	{
		return WICED_ERROR;
	}

	return 0;
}

int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len)
{
	wiced_i2c_message_t msg;

	wiced_i2c_init_rx_message(&msg, buff, (uint16_t)len, 1, WICED_FALSE);
	/* Read data from the I2C device */
	if (wiced_i2c_transfer(dev->i2c_dev, &msg, 1) != WICED_SUCCESS)
	{
		return WICED_ERROR;
	}

	return 0;
}

int VL6180x_AutoScaleRange(VL6180xDev_t dev, VL6180x_RangeData_t* recent_range)
{
	#define AutoThreshHigh  80  /*auto scale high thresh => AutoThreshHigh *  max_raneg => scale ++  */
	#define AutoThreshLow   33  /*auto scale low thresh  => AutoThreshHigh *  max_raneg => scale ++  */

	uint16_t hlimit;
	uint8_t scaling;
	int status = 0;

	scaling = VL6180x_UpscaleGetScaling(dev);
	hlimit = VL6180x_GetUpperLimit(dev);
	if (recent_range->range_mm >= (hlimit * AutoThreshHigh) / 100 && scaling < 3) {
		status = VL6180x_UpscaleSetScaling(dev, scaling + 1);
	}
	if (recent_range->range_mm < (hlimit * AutoThreshLow) / 100 && scaling > 1) {
		status = VL6180x_UpscaleSetScaling(dev, scaling - 1);
	}

	return status;
}



