#
# Copyright 2017, Future Electronics Inc. or a subsidiary of 
# Future Electronics Inc. All Rights Reserved.
# 
# This software, associated documentation and materials ("Software"),
# is owned by Future Electronics Inc. or one of its
# subsidiaries ("Future Electronics") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Future Electronics hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Future Electronics's
# integrated circuit products. Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Future Electronics.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Future Electronics
# reserves the right to make changes to the Software without notice. Future Electronics
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Future Electronics does
# not authorize its products for use in any products where a malfunction or
# failure of the Future Electronics product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Future Electronics's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Future Electronics against all liability.
#

NAME := App_Bluemix_sensors

$(NAME)_SOURCES :=  bluemix_iot_sensors.c \
					bluemix_sensors.c \
					sensors/bluemix_vl6180x.c \
					sensors/bme280_wiced_wrapper.c \
					sensors/bluemix_bme280.c \
					sensors/bluemix_sht3x.c \
					sensors/bluemix_mlx90316.c
					
$(NAME)_INCLUDES   := ./sensors

$(NAME)_COMPONENTS := protocols/MQTT \
					  drivers/sensors/ERTJ1VR103J \
					  drivers/sensors/VL6180X \
					  drivers/sensors/BME280 \
					  drivers/sensors/SHT3X \
					  drivers/sensors/MLX90316 \
					  utilities/command_console \
					  utilities/command_console/wifi

WIFI_CONFIG_DCT_H := wifi_config_dct.h
APPLICATION_DCT := bluemix_dct.c

$(NAME)_RESOURCES  := apps/bluemix_iot/rootca.cer