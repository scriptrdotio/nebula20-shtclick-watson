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
 * A rudimentary IBM Bluemix IoT publisher application which demonstrates how to connect to
 * IBM Bluemix IoT cloud (MQTT Broker) and publish MQTT messages for a given topic.
 *
 * This application periodically publishes sensor information to the MQTT broker and
 * topic configured. By default, it publishes to the IBM Bluemix Quickstart page
 * (https://quickstart.internetofthings.ibmcloud.com).
 *
 * To run the app, work through the following steps.
 *  1. Connect the desired sensor board.
 *  2. Build and run this application.
 *  3. Configure the WiFi settings with the set_wifi command.
 *  4. Optionally, configure the MQTT settings with the set_mqtt_settings command.
 *  5. Optionally, configure the MQTT topic with the set_mqtt_pub_topic command.
 *  6. Optionally, set the delay between publishing with the set_mqtt_pub_delay command.
 *  7. If the MQTT settings were changed and need to be saved, use the save_mqtt_settings command.
 *  8. Use the start_mqtt command to start publishing the sensor data using MQTT.
 *
 */

#include "wiced.h"
#include "mqtt_api.h"
#include "resources.h"
#include "wiced_crypto.h"
#include "command_console.h"
#include "command_console_wifi.h"
#include "bluemix_dct.h"
#include "bluemix_sensors.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define MQTT_DOMAIN                 "messaging.internetofthings.ibmcloud.com"

#define MQTT_REQUEST_TIMEOUT            (5000)
#define MQTT_RECONNECT_DELAY_MS         (2000)
#define MQTT_MAX_RESOURCE_SIZE          (0x7fffffff)
#define MQTT_PUBLISH_RETRY_COUNT        (3)
#define MQTT_SUBSCRIBE_RETRY_COUNT      (3)

#define MQTT_DATA_START_STR				"{\"d\":{"
#define MQTT_DATA_START_STR_LEN			(6)

#define BLUEMIX_CONSOLE_COMMAND_HISTORY_LENGTH  (10)
#define BLUEMIX_SENSOR_COMMAND_LENGTH           (85)

#define MQTT_THREAD_STACKSIZE    ( WICED_DEFAULT_APPLICATION_STACK_SIZE )

#define WIFI_CONSOLE_COMMANDS \
	{ (char*) "set_wifi", set_wifi, 2, NULL, NULL, (char*) "<ssid> <open|wpa_aes|wpa_tkip|wpa2|wpa2_tkip> [key] [channel] [ip netmask gateway]\n\t--Encapsulate SSID in quotes in order to include spaces", (char*) "Join an AP. DHCP assumed if no IP address provided"}, \

/** Bluemix Console commands */
#define BLUEMIX_CONSOLE_COMMANDS \
    { (char*) "stop_mqtt",          stop_mqtt,          0, NULL, NULL, (char *)"",                                             (char *)"Stop MQTT publishing" }, \
	{ (char*) "start_mqtt",         start_mqtt,         0, NULL, NULL, (char *)"",                                             (char *)"Start MQTT publishing" }, \
	{ (char*) "get_mqtt_settings",  get_mqtt_settings,  0, NULL, NULL, (char *)"",                                             (char *)"Get MQTT settings" }, \
	{ (char*) "set_mqtt_settings",  set_mqtt_settings,  3, NULL, NULL, (char *)"<org> <device_type> <device_id> [auth_token]", (char *)"Set MQTT org, device type, device ID, and (optionally) auth token" }, \
	{ (char*) "set_mqtt_pub_topic", set_mqtt_pub_topic, 1, NULL, NULL, (char *)"<publish_topic>",                              (char *)"Set the MQTT topic to publish to" }, \
	{ (char*) "set_mqtt_pub_delay", set_mqtt_pub_delay, 1, NULL, NULL, (char *)"<publish_delay_ms>",                           (char *)"Set the delay in milliseconds between MQTT publish tries" }, \
	{ (char*) "save_mqtt_settings", save_mqtt_settings, 0, NULL, NULL, (char *)"",                                             (char *)"Save all the MQTT/Bluemix settings" }, \
	{ (char*) "sub_topic",          sub_topic,          1, NULL, NULL, (char *)"<subscribe_topic>",                            (char *)"Subscribe to an MQTT topic (try \"iot-2/cmd/+/fmt/+\")" }, \

#define WICED_MQTT_EVENT_TYPE_SUBSCRIBED 4

/******************************************************************************
 *                                Structures
 ******************************************************************************/
/**
 * MQTT application settings
 */
typedef bluemix_app_dct_t mqtt_info_t;

/******************************************************************************
 *                           Function Prototypes
 ******************************************************************************/
/**
 * Console command to set the WiFi settings.
 */
int set_wifi( int argc, char* argv[] );

/**
 * Console command to stop the MQTT thread and disconnect from the broker.
 */
int stop_mqtt( int argc, char* argv[] );

/**
 * Console command to start the MQTT thread and start publishing data.
 */
int start_mqtt( int argc, char* argv[] );

/**
 * Console command to get the current MQTT settings.
 */
int get_mqtt_settings( int argc, char* argv[] );

/**
 * Console command to set the MQTT settings.
 */
int set_mqtt_settings( int argc, char* argv[] );

/**
 * Console command to set the topic to publish to for the MQTT.
 */
int set_mqtt_pub_topic( int argc, char* argv[] );

/**
 * Console command to set the delay between publishing MQTT data.
 */
int set_mqtt_pub_delay( int argc, char* argv[] );

/**
 * Console command to save all of the MQTT settings.
 */
int save_mqtt_settings( int argc, char* argv[] );

/**
 * Console command to set the MQTT topic to subscribe to.
 */
int sub_topic( int argc, char* argv[] );

/**
 * The function that the MQTT thread runs.
 */
void mqttThread(wiced_thread_arg_t arg);

/**
 * Subscribe to a MQTT topic.
 *
 * @param[in] topic : The topic to subscribe to
 *
 * @return @ref wiced_result_t
 */
static wiced_result_t subscribe_topic( char* topic );

/**
 * Copy the application DCT information to the RAM variable.
 *
 * @return wiced_result_t
 */
static wiced_result_t copy_app_dct( void );

/**
 * Write the App DCT to flash.
 *
 * @return wiced_result_t
 */
static wiced_result_t write_app_dct( void );

/**
 * Connect to a WiFi AP if one has been properly set in the DCT.
 *
 * @param[in] report_if_not_set : Whether or not to report out that the WiFi has not been configured.
 *
 * @return void
 */
static void connect_wifi_if_set(wiced_bool_t report_if_not_set);

/******************************************************
 *               Variable Definitions
 ******************************************************/
/** Received MQTT event */
static wiced_mqtt_event_type_t    received_event;
/** MQTT event received semaphore */
static wiced_semaphore_t          msg_semaphore;

/** MQTT username to use for authentication */
static uint8_t mqtt_username[] = "use-token-auth";

/** Console command table */
const command_t bluemix_console_command_table[] =
{
	WIFI_CONSOLE_COMMANDS
	BLUEMIX_CONSOLE_COMMANDS
	CMD_TABLE_END
};

/** Console command buffer */
static char bluemix_command_buffer[BLUEMIX_SENSOR_COMMAND_LENGTH];
/** Console history buffer */
static char bluemix_command_history_buffer[BLUEMIX_SENSOR_COMMAND_LENGTH * BLUEMIX_CONSOLE_COMMAND_HISTORY_LENGTH];
/** Flag that tells if the MQTT thread should be running */
static wiced_bool_t g_mqtt_run = WICED_TRUE;
/** Flag that tells if the MQTT thread is actually running */
static wiced_bool_t g_mqtt_running = WICED_FALSE;
/** Flag that tells if it is connected to the MQTT broker */
static wiced_bool_t g_mqtt_connected = WICED_FALSE;
/** MQTT settings information */
static mqtt_info_t g_mqtt_info;
/** MQTT object */
static wiced_mqtt_object_t   g_mqtt_object;

/** MQTT Thread handle */
wiced_thread_t g_mqttThreadHandle;

/******************************************************
 *               Static Function Definitions
 ******************************************************/

/*
 * A blocking call to an expected event.
 */
static wiced_result_t wait_for_response( wiced_mqtt_event_type_t event, uint32_t timeout )
{
    if ( wiced_rtos_get_semaphore( &msg_semaphore, timeout ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    else
    {
        if ( event != received_event )
        {
            return WICED_ERROR;
        }
    }
    return WICED_SUCCESS;
}

/*
 * Open a connection and wait for MQTT_REQUEST_TIMEOUT period to receive a connection open OK event
 */
static wiced_result_t mqtt_conn_open( wiced_mqtt_object_t mqtt_obj, wiced_ip_address_t *address, wiced_interface_t interface, wiced_mqtt_callback_t callback, wiced_mqtt_security_t *security, char *clientId )
{
    wiced_mqtt_pkt_connect_t conninfo;
    wiced_result_t ret = WICED_SUCCESS;

    memset( &conninfo, 0, sizeof( conninfo ) );
    conninfo.port_number = 0;
    conninfo.mqtt_version = WICED_MQTT_PROTOCOL_VER3;
    conninfo.clean_session = 1;
    conninfo.client_id = (uint8_t*) clientId;
    conninfo.keep_alive = 5;

    if(strlen(g_mqtt_info.auth_token) != 0){
		conninfo.password = (uint8_t*)g_mqtt_info.auth_token;
		conninfo.username = mqtt_username;
    }
    else{
		conninfo.password = NULL;
		conninfo.username = NULL;
    }

    WPRINT_APP_INFO( ( "Client ID...%s\n",clientId ) );
    ret = wiced_mqtt_connect( mqtt_obj, address, interface, callback, security, &conninfo );
    if ( ret != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( wait_for_response( WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS, MQTT_REQUEST_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Publish (send) message to MQTT topic and wait for 5 seconds to receive a PUBCOMP (as it is QoS=2).
 */
static wiced_result_t mqtt_app_publish( wiced_mqtt_object_t mqtt_obj, uint8_t qos, uint8_t *topic, uint8_t *data, uint32_t data_len )
{
    wiced_mqtt_msgid_t pktid;

    pktid = wiced_mqtt_publish( mqtt_obj, topic, data, data_len, qos );

    if ( pktid == 0 )
    {
        WPRINT_APP_INFO(("\n\n pktid=0 error"));
        return WICED_ERROR;
    }

    if ( wait_for_response( WICED_MQTT_EVENT_TYPE_PUBLISHED, MQTT_REQUEST_TIMEOUT ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("\n\n Wait for response error"));
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Subscribe to a MQTT topic and wait for 5 seconds to receive an ACM.
 */
static wiced_result_t mqtt_app_subscribe( wiced_mqtt_object_t mqtt_obj, char *topic, uint8_t qos )
{
    wiced_mqtt_msgid_t pktid;
    pktid = wiced_mqtt_subscribe( mqtt_obj, topic, qos );
    if ( pktid == 0 )
    {
        return WICED_ERROR;
    }
    if ( wait_for_response( WICED_MQTT_EVENT_TYPE_SUBSCRIBED, MQTT_REQUEST_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Close a connection and wait for 5 seconds to receive a connection close OK event
 */
static wiced_result_t mqtt_conn_close( wiced_mqtt_object_t mqtt_obj )
{
    if ( wiced_mqtt_disconnect( mqtt_obj ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( wait_for_response( WICED_MQTT_EVENT_TYPE_DISCONNECTED, MQTT_REQUEST_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Call back function to handle connection events.
 */
static wiced_result_t mqtt_connection_event_cb( wiced_mqtt_object_t mqtt_object, wiced_mqtt_event_info_t *event )
{
    switch ( event->type )
    {
        case WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS:
        case WICED_MQTT_EVENT_TYPE_DISCONNECTED:
        case WICED_MQTT_EVENT_TYPE_PUBLISHED:
        case WICED_MQTT_EVENT_TYPE_SUBSCRIBED:
        case WICED_MQTT_EVENT_TYPE_UNSUBSCRIBED:
        {
        	if(event->type == WICED_MQTT_EVENT_TYPE_DISCONNECTED){
        		g_mqtt_connected = WICED_FALSE;
        	}
            received_event = event->type;
            wiced_rtos_set_semaphore( &msg_semaphore );
        }
            break;
        case WICED_MQTT_EVENT_TYPE_PUBLISH_MSG_RECEIVED:
		{
			wiced_mqtt_topic_msg_t msg = event->data.pub_recvd;
			WPRINT_APP_INFO(( "[MQTT] Received %.*s  for TOPIC : %.*s\n", (int) msg.data_len, msg.data, (int) msg.topic_len, msg.topic ));
		}
			break;
        default:
            break;
    }
    return WICED_SUCCESS;
}

/*
 * Try to subscribe to a MQTT topic
 */
static wiced_result_t subscribe_topic( char* topic )
{
    wiced_result_t               ret = WICED_SUCCESS;
    int                          retries = 0;

    if(!g_mqtt_connected){
        WPRINT_APP_INFO( ( "MQTT not connected!\n" ) );
        return WICED_NOT_CONNECTED;
    }

    do
    {
        ret = mqtt_app_subscribe( g_mqtt_object, topic, WICED_MQTT_QOS_DELIVER_AT_MOST_ONCE );
        retries++ ;
    } while ( ( ret != WICED_SUCCESS ) && ( retries < MQTT_SUBSCRIBE_RETRY_COUNT ) );

    return ret;
}

int set_wifi( int argc, char* argv[] )
{
	if(g_mqtt_running == WICED_TRUE){
		stop_mqtt(0, NULL);
	}
	/* Take down the network before modifying parameters */
	wiced_network_down(WICED_STA_INTERFACE);

	return join(argc, argv);
}

int stop_mqtt( int argc, char* argv[] )
{
	g_mqtt_run = WICED_FALSE;
	if(g_mqtt_running == WICED_FALSE){
		return ERR_CMD_OK;
	}

	wiced_rtos_thread_force_awake(&g_mqttThreadHandle);
	wiced_rtos_thread_join(&g_mqttThreadHandle);

	return ERR_CMD_OK;
}

int start_mqtt( int argc, char* argv[] )
{
	g_mqtt_run = WICED_TRUE;
	if(g_mqtt_running == WICED_TRUE){
		return ERR_CMD_OK;
	}

	wiced_rtos_create_thread(&g_mqttThreadHandle, WICED_APPLICATION_PRIORITY, "mqttThread", mqttThread, MQTT_THREAD_STACKSIZE, NULL);

	return ERR_CMD_OK;
}

int get_mqtt_settings( int argc, char* argv[] )
{
	WPRINT_APP_INFO( (" Org: %s\n", g_mqtt_info.org) );
	WPRINT_APP_INFO( (" Device Type: %s\n", g_mqtt_info.dev_type) );
	WPRINT_APP_INFO( (" Device ID: %s\n", g_mqtt_info.dev_id) );
	WPRINT_APP_INFO( (" Auth Token: %s\n", g_mqtt_info.auth_token) );
	WPRINT_APP_INFO( (" Publish Topic: %s\n", g_mqtt_info.pub_topic) );
	WPRINT_APP_INFO( (" Subscribe Topic: %s\n", g_mqtt_info.sub_topic) );
	WPRINT_APP_INFO( (" Publish Delay: %ums\n", (unsigned)g_mqtt_info.delay) );
	return ERR_CMD_OK;
}

int set_mqtt_settings( int argc, char* argv[] )
{
	strncpy(g_mqtt_info.org, argv[1], sizeof(g_mqtt_info.org));
	strncpy(g_mqtt_info.dev_type, argv[2], sizeof(g_mqtt_info.dev_type));
	strncpy(g_mqtt_info.dev_id, argv[3], sizeof(g_mqtt_info.dev_id));

	if(argc > 4){
		strncpy(g_mqtt_info.auth_token, argv[4], sizeof(g_mqtt_info.auth_token));
	}
	else{
		memset(g_mqtt_info.auth_token, 0, sizeof(g_mqtt_info.auth_token));
	}

	return ERR_CMD_OK;
}

int set_mqtt_pub_topic( int argc, char* argv[] )
{
	strncpy(g_mqtt_info.pub_topic, argv[1], sizeof(g_mqtt_info.pub_topic));

	return ERR_CMD_OK;
}

int set_mqtt_pub_delay( int argc, char* argv[] )
{
	string_to_unsigned( argv[1], strlen(argv[1]), &g_mqtt_info.delay, 0);

	return ERR_CMD_OK;
}

int sub_topic( int argc, char* argv[] )
{
	strncpy(g_mqtt_info.sub_topic, argv[1], sizeof(g_mqtt_info.sub_topic));

	if(subscribe_topic(g_mqtt_info.sub_topic) != WICED_SUCCESS){
		return ERR_UNKNOWN;
	}

	return ERR_CMD_OK;
}

int save_mqtt_settings( int argc, char* argv[] )
{
    write_app_dct();

    return ERR_CMD_OK;
}


/* Define the thread function that perform the MQTT comms */
void mqttThread(wiced_thread_arg_t arg)
{
	wiced_result_t        ret = WICED_SUCCESS;
	int                   connection_retries = 0;
	int                   retries = 0;
	/** Buffer for the whole client ID */
	char                  clientId_buffer[101];
	/** Buffer to hold the MQTT message data */
	char                  msg[512];
	char                  endMsg[60];//just holds the scriptr_dev_id
	/** Buffer for the full hostname */
	char                  hostname[101];
	/** IP address of the MQTT broker */
	wiced_ip_address_t broker_address;
	wiced_mqtt_security_t     security;

	memset(&security, 0x00, sizeof(wiced_mqtt_security_t));
	/* Read security parameters from DCT */
	resource_get_readonly_buffer( &resources_apps_DIR_bluemix_iot_DIR_rootca_cer, 0, MQTT_MAX_RESOURCE_SIZE, &security.ca_cert_len, (const void **) &security.ca_cert );

	g_mqtt_running = WICED_TRUE;

	 /* Allocate memory for MQTT object*/
	g_mqtt_object = (wiced_mqtt_object_t) malloc( WICED_MQTT_OBJECT_MEMORY_SIZE_REQUIREMENT );
	if ( g_mqtt_object == NULL )
	{
		WPRINT_APP_ERROR("Don't have memory to allocate for mqtt object...\n");
		g_mqtt_running = WICED_FALSE;
		return;
	}

	WPRINT_APP_INFO( ( "Resolving IP address of MQTT broker...\n" ) );
	sprintf(hostname, "%s.%s", g_mqtt_info.org, MQTT_DOMAIN);
	ret = wiced_hostname_lookup( hostname, &broker_address, 10000, WICED_STA_INTERFACE );
	WPRINT_APP_INFO(("Resolved Broker IP: %u.%u.%u.%u\n\n", (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 24),
					(uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 16),
					(uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 8),
					(uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 0)));
	if ( ret == WICED_ERROR || broker_address.ip.v4 == 0 )
	{
		WPRINT_APP_INFO(("Error in resolving DNS\n"));
		free( g_mqtt_object );
		g_mqtt_object = NULL;
		g_mqtt_running = WICED_FALSE;
		return;
	}

	if(strlen(g_mqtt_info.dev_id) == 0){
	    /* Set the device ID to be the MAC address */
		wiced_mac_t mac;
		wwd_wifi_get_mac_address( &mac, WWD_STA_INTERFACE );
		sprintf(g_mqtt_info.dev_id, "%02X%02X%02X%02X%02X%02X", mac.octet[0],mac.octet[1],mac.octet[2],mac.octet[3],mac.octet[4],mac.octet[5]);
	}
	sprintf(clientId_buffer, "d:%s:%s:%s", g_mqtt_info.org, g_mqtt_info.dev_type, g_mqtt_info.dev_id);

	wiced_mqtt_init( g_mqtt_object );

	while(g_mqtt_run == WICED_TRUE)
	{
		g_mqtt_connected = WICED_FALSE;
		WPRINT_APP_INFO(("[MQTT] Opening connection..."));
		do
		{
			ret = mqtt_conn_open( g_mqtt_object, &broker_address, WICED_STA_INTERFACE, mqtt_connection_event_cb, &security ,clientId_buffer);
			connection_retries++ ;
		} while ( ( ret != WICED_SUCCESS ) && ( connection_retries < WICED_MQTT_CONNECTION_NUMBER_OF_RETRIES ) );

		if ( ret != WICED_SUCCESS )
		{
			WPRINT_APP_INFO(("Failed\n"));
			break;
		}
		g_mqtt_connected = WICED_TRUE;
		WPRINT_APP_INFO(("---- Success\n"));

		if(strlen(g_mqtt_info.sub_topic) != 0){
			WPRINT_APP_INFO(("[MQTT] Subscribing to topic..."));
			if(subscribe_topic(g_mqtt_info.sub_topic) == WICED_SUCCESS){
				WPRINT_APP_INFO((" Success\n"));
			}
			else{
				WPRINT_APP_INFO((" Failed\n"));
			}
		}

		while ( g_mqtt_run == WICED_TRUE )
		{
			WPRINT_APP_INFO(("[MQTT] Publishing...\n"));

			memcpy(msg, MQTT_DATA_START_STR, MQTT_DATA_START_STR_LEN+1);
			msg[MQTT_DATA_START_STR_LEN] = 0x00;

			/* Read the sensors and only send data if the sensors were properly read. */
			if(sensors_read(msg, WICED_TRUE) == WICED_SUCCESS)
			{
			    sprintf(endMsg,",\"id\":\"%s\"}}",g_mqtt_info.scriptr_dev_id);
			    strcat(msg, endMsg);
			    //strcat(msg, "}}");
				WPRINT_APP_INFO((" Message :%s\n",msg));
				retries = 0;

				do
				{
					ret = mqtt_app_publish( g_mqtt_object, WICED_MQTT_QOS_DELIVER_AT_MOST_ONCE, (uint8_t*) g_mqtt_info.pub_topic, (uint8_t*) msg, strlen( msg ) );
					retries++ ;
				} while ( ( ret != WICED_SUCCESS ) && ( retries < MQTT_PUBLISH_RETRY_COUNT ) );
				if ( ret != WICED_SUCCESS )
				{
					WPRINT_APP_INFO((" Failed publish\n"));
					break;
				}
				else
				{
					WPRINT_APP_INFO((" Success publish\n"));
				}
			}

			if(g_mqtt_run == WICED_TRUE){
				wiced_rtos_delay_milliseconds( g_mqtt_info.delay );
			}
		}

		WPRINT_APP_INFO(("[MQTT] Closing connection..."));
		g_mqtt_connected = WICED_FALSE;
		mqtt_conn_close( g_mqtt_object );

		wiced_rtos_delay_milliseconds(MQTT_RECONNECT_DELAY_MS);
	}

	WPRINT_APP_INFO(("[MQTT] Deinit MQTT...\n"));
	ret = wiced_mqtt_deinit( g_mqtt_object );
	free( g_mqtt_object );
	g_mqtt_object = NULL;

	g_mqtt_running = WICED_FALSE;

	return;
}

/**
 * Copy the application DCT information to the RAM variable.
 */
static wiced_result_t copy_app_dct( void )
{
    bluemix_app_dct_t* dct_app = NULL;

    if ( wiced_dct_read_lock( (void**) &dct_app, WICED_FALSE, DCT_APP_SECTION, 0, sizeof( *dct_app ) ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    /* since we passed ptr_is_writable as WICED_FALSE, we are not allowed to write in to memory pointed by dct_security */

    memcpy(&g_mqtt_info, dct_app, sizeof(bluemix_app_dct_t));

    /* Here ptr_is_writable should be same as what we passed during wiced_dct_read_lock() */
    wiced_dct_read_unlock( dct_app, WICED_FALSE );

    return WICED_SUCCESS;
}

/**
 * Write the App DCT to flash.
 */
static wiced_result_t write_app_dct( void )
{
    bluemix_app_dct_t* app_dct = NULL;

    /* get the App config section for modifying, any memory allocation required would be done inside wiced_dct_read_lock() */
	wiced_dct_read_lock( (void**) &app_dct, WICED_TRUE, DCT_APP_SECTION, 0, sizeof( *app_dct ) );

	memcpy(app_dct, &g_mqtt_info, sizeof(bluemix_app_dct_t));

	wiced_dct_write( (const void*) app_dct, DCT_APP_SECTION, 0, sizeof(bluemix_app_dct_t) );

	wiced_dct_read_unlock( app_dct, WICED_TRUE );

    return WICED_SUCCESS;
}

/**
 * Connect to a WiFi AP if one has been properly set in the DCT.
 */
static void connect_wifi_if_set(wiced_bool_t report_if_not_set)
{
    wiced_result_t        ret = WICED_SUCCESS;
    platform_dct_wifi_config_t*  wifi_config;
    wiced_bool_t ap_info_valid = WICED_FALSE;

    /* Get a copy of the WIFT config from the DCT into RAM */
    wiced_dct_read_lock((void**) &wifi_config, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t));

    if(wifi_config->stored_ap_list[0].details.SSID.length == 0){
        if(report_if_not_set){
            WPRINT_APP_INFO(("WiFi AP info not configured! Use the 'set_wifi' command to configured it.\n"));
        }
    }
    else{
        ap_info_valid = WICED_TRUE;
    }

    /* Free RAM buffer */
    wiced_dct_read_unlock(wifi_config, WICED_FALSE);

    if(ap_info_valid)
    {
        /* Bring up the network interface */
        ret = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
        if ( ret != WICED_SUCCESS ){
            WPRINT_APP_INFO( ( "\nNot able to join the requested AP\n\n" ) );
        }
    }
}


/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( void )
{
    wiced_result_t        ret = WICED_SUCCESS;

    wiced_init( );

    /* Disable roaming to other access points */
    wiced_wifi_set_roam_trigger( -99 ); /* -99dBm ie. extremely low signal level */

    connect_wifi_if_set(WICED_TRUE);

    WPRINT_APP_INFO( ("Initializing sensors...\n" ));
    sensors_init();

	copy_app_dct();

	ret = command_console_init(STDIO_UART, sizeof(bluemix_command_buffer), bluemix_command_buffer,
								  BLUEMIX_CONSOLE_COMMAND_HISTORY_LENGTH, bluemix_command_history_buffer, " ");

	if (ret != WICED_SUCCESS)
	{
		WPRINT_APP_INFO(("Error starting the command console\n"));
	}
	console_add_cmd_table( bluemix_console_command_table );

    wiced_rtos_init_semaphore( &msg_semaphore );

    /* Start the MQTT thread if the WiFi is working */
    if(wiced_network_is_up(WICED_STA_INTERFACE)){
    	start_mqtt(0, NULL);
    }

    while(1){
    	wiced_rtos_delay_milliseconds( 1000000 );
    	WPRINT_APP_INFO(("PING!"));

    	if(!wiced_network_is_up(WICED_STA_INTERFACE)){
    		connect_wifi_if_set(WICED_FALSE);
    		/* Start MQTT if now connected and it is supposed to be running */
    		if(wiced_network_is_up(WICED_STA_INTERFACE) && g_mqtt_run == WICED_TRUE && !g_mqtt_running){
    			start_mqtt(0, NULL);
    		}
		}
    }

    wiced_rtos_deinit_semaphore( &msg_semaphore );

    return;
}
