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
 *  Implements functions to read various IoT sensors and put the data into a JSON string.
 */

#include "bluemix_sensors.h"

#include "bluemix_vl6180x.h"
#include "bluemix_bme280.h"
#include "bluemix_sht3x.h"
#include "bluemix_mlx90316.h"

#ifndef SENSORS_DISABLE_THERMISTOR
#include "thermistor.h" /* Using Panasonic ERTJ1VR103J thermistor */
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define NUM_IOT_SENSORS     (sizeof(iot_sensors)/sizeof(iot_sensors[0]))

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

/**
 * IoT Sensor structure
 */
typedef struct
{
    /** Whether the sensor is attached or not */
    wiced_bool_t attached;
    /** Pointer to the function to initialize the sensor */
    wiced_result_t (*sensor_init)(void);
    /** Pointer to the function to read the sensor */
    wiced_result_t (*sensor_read)(char *mqtt_msg, wiced_bool_t first_data);
} iot_sensor_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/
#ifndef SENSORS_DISABLE_THERMISTOR
/**
 * Read the thermistor data and put it into the MQTT message string.
 *
 * @param[in] mqtt_msg   : The current MQTT message string
 * @param[in] first_data : Whether or not this is the first set of data in the MQTT string
 *
 * @return @ref wiced_result_t
 */
static wiced_result_t read_thermistor(char *mqtt_msg, wiced_bool_t first_data);
#endif /* ifndef SENSORS_DISABLE_THERMISTOR */

/******************************************************
 *               Variable Definitions
 ******************************************************/
/**
 * Array of IoT sensors that could be connected to the system.
 */
static iot_sensor_t iot_sensors[] =
{
        {.attached = WICED_FALSE, .sensor_init = init_vl6180x,  .sensor_read = read_vl6180x},
        {.attached = WICED_FALSE, .sensor_init = init_bme280,   .sensor_read = read_bme280},
        {.attached = WICED_FALSE, .sensor_init = init_sht3x,    .sensor_read = read_sht3x},
        {.attached = WICED_FALSE, .sensor_init = init_mlx90316, .sensor_read = read_mlx90316}
};


/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t sensors_init(void)
{
	wiced_result_t wres = WICED_ERROR;
	uint32_t i;

#ifndef SENSORS_DISABLE_THERMISTOR
	WPRINT_APP_INFO( ("Initializing Thermistor..." ));
	if(wiced_adc_init( THERMISTOR_ADC, 5 ) == WICED_SUCCESS){
		wres = WICED_SUCCESS;
		WPRINT_APP_INFO( (" Success!\n" ));
	}
	else{
		WPRINT_APP_INFO( (" Error!\n" ));
	}
#endif

	for(i=0; i<NUM_IOT_SENSORS; i++)
	{
	    /* Check if we can properly initialize the sensor */
	    if(iot_sensors[i].sensor_init() == WICED_SUCCESS){
	        iot_sensors[i].attached = WICED_TRUE;
	        wres = WICED_SUCCESS;
	    }
	}

	return wres;
}


wiced_result_t sensors_read(char *mqtt_msg, wiced_bool_t first_data)
{
	wiced_bool_t data_added = WICED_FALSE;
	uint32_t i;

#ifndef SENSORS_DISABLE_THERMISTOR
	if(read_thermistor(mqtt_msg, first_data) == WICED_SUCCESS){
		first_data = WICED_FALSE;
		data_added = WICED_TRUE;
	}
#endif

	/* Read all of the attached sensors */
	for(i=0; i<NUM_IOT_SENSORS; i++)
    {
        if(iot_sensors[i].attached && iot_sensors[i].sensor_read(mqtt_msg, first_data) == WICED_SUCCESS){
            first_data = WICED_FALSE;
            data_added = WICED_TRUE;
        }
    }

	return (data_added ? WICED_SUCCESS : WICED_ERROR);
}


#ifndef SENSORS_DISABLE_THERMISTOR
static wiced_result_t read_thermistor(char *mqtt_msg, wiced_bool_t first_data)
{
	int16_t               temp_celcius_tenths;
	float				  temp_celcius;
	uint16_t msg_len;

	msg_len = strlen(mqtt_msg);

	/* Read thermistor */
	if(thermistor_take_sample( THERMISTOR_ADC, &temp_celcius_tenths ) == WICED_SUCCESS){
		temp_celcius = (float)temp_celcius_tenths/10.0;
		WPRINT_APP_INFO( (" Thermistor Temperature: %.1f\xf8""C\n", temp_celcius) );
		if(!first_data){
			strcat(mqtt_msg, ",");
			msg_len++;
		}
		sprintf(&mqtt_msg[msg_len], "\"thermistor_temp\":%.1f",temp_celcius);

		return WICED_SUCCESS;
	}

	return WICED_ERROR;
}
#endif /* ifndef SENSORS_DISABLE_THERMISTOR */




