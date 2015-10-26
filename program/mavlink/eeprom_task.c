#include <stdio.h>
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
bool new_save_request; //The old save request isn't finished then receive a new request

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
	eeprom_task_is_running = true;

	vTaskResume(eeprom_save_task_handle);
}

/* Don't call this function directly!
 * This function is create for flight control task to mantain the EEPROM resource! */
void eeprom_task_suspend_by_flight_control(void)
{
	eeprom_task_is_running = false;

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

/* This task is create for saving global datas into the EEPROM without interrupt
 * the communication with ground station */
void eeprom_save_task(void)
{
	while(1) {
		eeprom_task_is_running = true;

		bool parameter_config;

		bool eeprom_failed;
		int eeprom_status;

		eeprom.write('\0', 0, 1); //Clear the EEPROM check byte

		/* Save all the global datas into the EEPROM */
		int i;
		for(i = 0; i < get_global_data_count(); i++) {
			get_global_data_parameter_config_status(i, &parameter_config);

			//Not a user-modifiable parameter
			if(parameter_config == true) {
				continue;
			}

			//Save current global data into the EEPROM
			eeprom_status = save_global_data_into_eeprom(i);

			if(eeprom_status != GLOBAL_EEPROM_SUCCESS) {
				eeprom_failed = true;
				break;
			}
		}

		if(eeprom_failed == false) {
			/* Not finished yet */
			if(new_save_request == false) {
				send_mavlink_status_text_message("Successfully saved data into EEPROM", MAV_SEVERITY_INFO);

				/* Setup the EEPROM check byte */
				uint8_t global_data_count = get_global_data_count();
				eeprom.write(&global_data_count, 0, 1);
			}
		} else {
			new_save_request = false; //Force to cancel the new save request

			/* Print the Error message and the error code */
			char error_message[51] = {'\0'};
			sprintf(error_message, "Failed to save data into EEPROM [Error code: %d]", eeprom_status);
			send_mavlink_status_text_message(error_message, MAV_SEVERITY_ERROR);
		}

		eeprom_task_is_running = false;

		/* The job is not finished yet */
		if (new_save_request == true) {
			new_save_request = false;
		/* Nothing to do, suspend the task */
		} else {
			save_request = false;
			vTaskSuspend(NULL);
		}
	}
}
