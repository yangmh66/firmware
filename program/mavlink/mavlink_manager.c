#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "_math.h"
#include "delay.h"

#include "usart.h"
#include "radio_control.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "mavlink.h"

#include "global.h"

#include "mavlink_manager.h"
#include "generic.h"
#include "parameter.h"
#include "mission.h"
#include "eeprom_task.h"
#include "FreeRTOS.h"
#include "system_time.h"
#include "io.h"

#define SEND_DEBUG_MAVLINK_STATUS_MSG 1

static void send_heartbeat_info(void);
static void send_gps_info(void);
static void send_attitude_info(void);
static void send_reached_waypoint(void);
static void send_current_waypoint(void);
static void send_debug_status_text_message(void);

extern int16_t __nav_roll,__nav_pitch;
extern uint32_t __pAcc,__numSV;
extern int32_t __altitude_Zd;

/* USART TX DMA buffer */
uint8_t receiver_task_buffer[MAVLINK_MAX_PAYLOAD_LEN];
uint8_t broadcast_task_buffer[MAVLINK_MAX_PAYLOAD_LEN];

/* Mavlink receiver task data read sleep time */
uint32_t receiver_sleep_time;

/* Mavlink broadcast information */
broadcast_message_t broadcast_message_list[] = {
	BROADCAST_MSG_DEF(send_heartbeat_info, RATE_HZ(1)),
	BROADCAST_MSG_DEF(send_gps_info, RATE_HZ(1)),
	BROADCAST_MSG_DEF(send_debug_status_text_message, RATE_HZ(1)),
	BROADCAST_MSG_DEF(send_attitude_info, RATE_HZ(20)),
	BROADCAST_MSG_DEF(send_reached_waypoint, RATE_HZ(20)),
	BROADCAST_MSG_DEF(send_current_waypoint, RATE_HZ(20))
};

void receiver_task_send_package(mavlink_message_t *msg)
{
	uint16_t len = mavlink_msg_to_send_buffer(receiver_task_buffer, msg);
	
	mavlink_receiver_serial_write(receiver_task_buffer, len);
}

static void broadcast_task_send_package(mavlink_message_t *msg)
{
	uint16_t len = mavlink_msg_to_send_buffer(broadcast_task_buffer, msg);

	status_mavlink_serial_write(broadcast_task_buffer, len);
}

static void send_heartbeat_info(void)
{
	mavlink_message_t msg;
	uint8_t current_flight_mode,current_safety_switch;
	uint8_t current_MAV_mode = MAV_MODE_PREFLIGHT;

	read_global_data_value(MODE_BUTTON, DATA_POINTER_CAST(&current_flight_mode));
	read_global_data_value(SAFTY_BUTTON, DATA_POINTER_CAST(&current_safety_switch));

	if(current_safety_switch == 0){
		/* ENGINE ON */

		if(current_flight_mode == 0){
			/* Mode 1 */
			current_MAV_mode = MAV_MODE_STABILIZE_ARMED;

		}else if(current_flight_mode == 1){
			/* Mode 2 */
			current_MAV_mode = MAV_MODE_GUIDED_ARMED;

		}else if(current_flight_mode == 2){
			/* Mode 3 */
			current_MAV_mode = MAV_MODE_AUTO_ARMED;

		}


	}else if(current_safety_switch == 1){
		/* ENGINE OFF */

		if(current_flight_mode == 0){
			/* Mode 1 */
			current_MAV_mode = MAV_MODE_STABILIZE_DISARMED;

		}else if(current_flight_mode == 1){
			/* Mode 2 */
			current_MAV_mode = MAV_MODE_GUIDED_DISARMED;

		}else if(current_flight_mode == 2){
			/* Mode 3 */
			current_MAV_mode = MAV_MODE_AUTO_DISARMED;

		}
	}

	mavlink_msg_heartbeat_pack(1, 200, &msg,
		MAV_TYPE_QUADROTOR, 
		MAV_AUTOPILOT_GENERIC, 
		current_MAV_mode, 
		0, MAV_STATE_ACTIVE
	);

	broadcast_task_send_package(&msg);
}

static void send_gps_info(void)
{
	int32_t latitude, longitude, altitude;
	int16_t gps_vx, gps_vy, gps_vz;
	float true_yaw;

	/* Prepare the GPS data */
	read_global_data_value(GPS_LAT, DATA_POINTER_CAST(&latitude));
	read_global_data_value(GPS_LON, DATA_POINTER_CAST(&longitude));
	read_global_data_value(GPS_ALT, DATA_POINTER_CAST(&altitude));
	read_global_data_value(GPS_VX, DATA_POINTER_CAST(&gps_vx));
	read_global_data_value(GPS_VY, DATA_POINTER_CAST(&gps_vy));
	read_global_data_value(GPS_VZ, DATA_POINTER_CAST(&gps_vz));
	read_global_data_value(TRUE_YAW, DATA_POINTER_CAST(&true_yaw));

	mavlink_message_t msg;

	mavlink_msg_global_position_int_pack(1, 220, &msg, 
		get_system_time_ms(),   		       //time 
		latitude ,  //Latitude
		longitude ,  //Longitude
		altitude, //Altitude
		0,
		gps_vx * 1,   //Speed-Vx
		gps_vy * 1,   //Speed-Vy
		gps_vz * 1,   //Speed-Vz
		(uint16_t)true_yaw
	);

	broadcast_task_send_package(&msg);
}

static void send_attitude_info(void)
{
	mavlink_message_t msg;
	float attitude_roll, attitude_pitch, attitude_yaw;

	/* Prepare the attitude data */
	read_global_data_value(TRUE_ROLL, DATA_POINTER_CAST(&attitude_roll));
	read_global_data_value(TRUE_PITCH, DATA_POINTER_CAST(&attitude_pitch));
	read_global_data_value(TRUE_YAW, DATA_POINTER_CAST(&attitude_yaw));

	mavlink_msg_attitude_pack(1, 200, &msg,
		get_system_time_ms(),
		toRad(attitude_roll), 
		toRad(attitude_pitch), 
		toRad(attitude_yaw), 
		0.0, 0.0, 0.0
	);

	broadcast_task_send_package(&msg);
}

#if 0
static void send_system_info(void)
{
	mavlink_message_t msg;

	mavlink_msg_sys_status_pack(1, 0, &msg,
		0,
		0,
		0,
		0,
		12.5 * 1000, //Battery voltage
		-1,
		100,         //Battery remaining
		0,
		0,
		0,
		0,
		0,
		0
	);

	send_package(&msg);
}
#endif

static void send_reached_waypoint(void)
{
	if(mission_info.reached_waypoint.is_update == true) {
		mavlink_message_t msg;		

		/* Notice the ground station that the vehicle is reached at the 
	   	waypoint */
		mavlink_msg_mission_item_reached_pack(1, 0, &msg,
			mission_info.reached_waypoint.number);
		broadcast_task_send_package(&msg);

		mission_info.reached_waypoint.is_update = false;
	}
}

static void send_current_waypoint(void)
{
	if(mission_info.current_waypoint.is_update == true) {
		mavlink_message_t msg;		

		/* Update the new current waypoint */
		mavlink_msg_mission_current_pack(1, 0, &msg,
			mission_info.current_waypoint.number);
		broadcast_task_send_package(&msg);

		mission_info.current_waypoint.is_update = false;
	}
}

void send_status_text_message(char *text, uint8_t severity)
{
	mavlink_message_t msg;

	mavlink_msg_statustext_pack(1, 0, &msg, severity, text);
	broadcast_task_send_package(&msg);
}

static void send_debug_status_text_message(void)
{
#if SEND_DEBUG_MAVLINK_STATUS_MSG != 0
	char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];

	sprintf(text, "Zd:%ld NAV: %d,%d,%ld,%ld",
		__altitude_Zd,
		__nav_roll,
		__nav_pitch,
		__pAcc,
		__numSV
	);

	send_status_text_message(text, MAV_SEVERITY_DEBUG);
#endif	
}

void set_mavlink_receiver_delay_time(uint32_t time)
{
	receiver_sleep_time = time;
}

static void handle_message(mavlink_message_t *mavlink_message)
{
	if(generic_handle_message(mavlink_message) == true) {
		return;
	}
	
	if(mission_handle_message(mavlink_message) == true) {
		return;
	}

	/* If still return a false value, this is a parser undefined mavlink message */
	if(parameter_handle_message(mavlink_message) == false) {
		MAVLINK_DEBUG_PRINT("[Parser undefined message]msgid:%d\n\r", mavlink_message->msgid);
	}
}

void mavlink_receiver_task(void)
{
	int buffer;
	receiver_sleep_time = portMAX_DELAY; //Sleep until someone wake the task up
	
	mavlink_message_t mavlink_message;
	mavlink_status_t message_status;

	init_global_data_with_eeprom();

	while(1) {
		//Try to receive a byte, if there is no data, the task won't be waken
		buffer = usart3_read(receiver_sleep_time);

		//Parse and handle the mavlink message if the data is available
		if(buffer != USART_NOT_AVAILABLE) {
			if(mavlink_parse_char(MAVLINK_COMM_0, buffer, &mavlink_message, &message_status)) {
				handle_message(&mavlink_message);
			}
		}

		mavlink_mission_timeout_check();

		parameter_send(); //Will only be executed if parser received the request
	}
}

void mavlink_broadcast_task()
{
	uint32_t current_time;

	unsigned int i;
	uint32_t compare_tick_time = broadcast_message_list[0].period_tick;
	for(i = 1; i < BROADCAST_MESSAGE_CNT; i++) {
		//Find the minimum tick value in the list
		if(broadcast_message_list[i].period_tick < compare_tick_time) {
			compare_tick_time = broadcast_message_list[i].period_tick;	
		}
	}

	//Set task delay time to 1/10 minimum broadcast period time
	uint32_t broadcast_delay_time = compare_tick_time / 10;

	while(1) {
		for(i = 0; i < BROADCAST_MESSAGE_CNT; i++) {
			current_time = get_system_time_ms();

			/* Timeout check */
			if((current_time - broadcast_message_list[i].last_send_time) >=
				broadcast_message_list[i].period_tick)
			{
				/* Send message and update timer */
				broadcast_message_list[i].send_message();
				broadcast_message_list[i].last_send_time = current_time;
			}
		}

		vTaskDelay(broadcast_delay_time);
	}
}
