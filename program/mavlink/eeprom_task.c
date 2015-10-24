#include <stdint.h>
#include <stdbool.h>

#include "gpio.h"
#include "led.h"
#include "usart.h"

#include "AT24C04C.h"

#include "FreeRTOS.h"
#include "task.h"

#include "mavlink.h"

#include "global.h"
#include "eeprom_task.h"

extern xTaskHandle eeprom_save_task_handle;

uint8_t eeprom_task_buffer[MAVLINK_MAX_PAYLOAD_LEN];

bool save_request;
bool new_save_request; //The old save request isn't finished but get a new request
bool task_has_been_suspended;
bool eeprom_task_is_running;

/**
  * @brief  Request flight control task to save the data into the EEPROM
  * @param  None
  * @retval None
  */
void eeprom_save_request(void)
{
	if(save_request == true) {
		new_save_request = true;
	}

	save_request = true;
}

bool check_eeprom_save_request(void)
{
	return save_request;
}

/* Don't call this function directly, call the function "eeprom_save_request" if
 * you want to save the global data into into the EEPROM!
 * This function is create for flight control task to mantain the EEPROM resource! */
void eeprom_task_execute_by_flight_control(void)
{
	vTaskResume(eeprom_save_task_handle);
}

/* Don't call this function directly!
 * This function is create for flight control task to mantain the EEPROM resource! */
void eeprom_task_suspend_by_flight_control(void)
{
	task_has_been_suspended = true;

	vTaskSuspend(eeprom_save_task_handle);
}

bool is_eeprom_task_running(void)
{
	return eeprom_task_is_running;
}

static void send_mavlink_status_text_message(char *text, uint8_t severity)
{
	mavlink_message_t msg;
	mavlink_msg_statustext_pack(1, 0, &msg, severity, text);

	uint16_t len = mavlink_msg_to_send_buffer(eeprom_task_buffer, &msg);
	eeprom_manager_task_serial_write(eeprom_task_buffer, len);
}

/* This task is use for saving global datas into the EEPROM without interrupt
 * the communication with ground station */
void eeprom_save_task(void)
{
	while(1) {
		bool eeprom_failed;

		eeprom_task_is_running = true;

		/* Ensure the data will fully writting into the eeprom by checking
		 * the first byte */
		eeprom.write('\0', 0, 1); //Clear the first byte

		int i;
		for(i = 0; i < get_global_data_count(); i++) {
			bool parameter_config;
			get_global_data_parameter_config_status(i, &parameter_config);

			if(parameter_config == true) {
				int save_status = save_global_data_into_eeprom(i);

				if(save_status != GLOBAL_EEPROM_SUCCESS) {
					eeprom_failed = true;
				}
			}
		}

		uint8_t global_data_count = get_global_data_count();
		eeprom.write(&global_data_count, 0, 1);

		if(eeprom_failed == false) {
			if(task_has_been_suspended == false) {
				send_mavlink_status_text_message("Successfully saved datas into EEPROM", MAV_SEVERITY_INFO);
			}
		} else {
			send_mavlink_status_text_message("EEPROM save is failed", MAV_SEVERITY_ERROR);
		}

		eeprom_task_is_running = false;

		/* If the task has been suspended and a new eeprom save request is exist,
		 * then save global datas into the EEPROM again to make sure the data in 
		 * EEPROM is the newest */
		if(task_has_been_suspended == true && save_request == true) {
			task_has_been_suspended = false;
			save_request = false;
		/* If the old request isn't finished and get a new request, run the process again*/
		} else if (new_save_request == true) {
			new_save_request = false;
		} else {
			save_request = false;
			vTaskSuspend(NULL);
		}
	}
}
