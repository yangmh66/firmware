#include <stdbool.h>

#include "gpio.h"
#include "led.h"

#include "AT24C04C.h"

#include "FreeRTOS.h"
#include "task.h"

#include "global.h"
#include "eeprom_task.h"

extern xTaskHandle eeprom_save_task_handle;

bool save_request;
bool task_has_been_suspended;
bool eeprom_task_is_running;

/**
  * @brief  Request flight control task to save the data into the EEPROM
  * @param  None
  * @retval None
  */
void eeprom_save_request(void)
{
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
	clear_eeprom_pending_flag();

	vTaskResume(eeprom_save_task_handle);
}

/* Don't call this function directly!
 * This function is create for flight control task to mantain the EEPROM resource! */
void eeprom_task_suspend_by_flight_control(void)
{
	set_eeprom_pending_flag();

	task_has_been_suspended = true;

	vTaskSuspend(eeprom_save_task_handle);
}

bool is_eeprom_task_running(void)
{
	return eeprom_task_is_running;
}

/* This task is use for saving global datas into the EEPROM without interrupt
 * the communication with ground station */
void eeprom_save_task(void)
{
	while(1) {
		eeprom_task_is_running = true;

		/* Ensure the data will fully writting into the eeprom by checking
		 * the first byte */
		eeprom.write('\0', 0, 1); //Clear the first byte

		int i;
		for(i = 0; i < get_global_data_count(); i++) {
			bool parameter_config;
			get_global_data_parameter_config_status(i, &parameter_config);

			if(parameter_config == true) {
				LED_OFF(TOGGLE_DEBUG);

				save_global_data_into_eeprom(i);

				LED_ON(TOGGLE_DEBUG);
			}
		}

		uint8_t global_data_count = get_global_data_count();
		eeprom.write(&global_data_count, 0, 1);

		eeprom_task_is_running = false;

		/* If the task has been suspended and a new eeprom save request is exist,
		 * then save global datas into the EEPROM again to make sure the data in 
		 * EEPROM is the newest */
		if(task_has_been_suspended == true && save_request == true) {
			task_has_been_suspended = false;
			save_request = false;
		} else {
			save_request = false;
			vTaskSuspend(NULL);
		}
	}
}
