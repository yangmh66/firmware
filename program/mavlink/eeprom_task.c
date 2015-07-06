#include "AT24C04C.h"

#include "FreeRTOS.h"
#include "task.h"

#include "global.h"
#include "eeprom_task.h"

extern xTaskHandle eeprom_save_task_handle;

void eeprom_task_execute(void)
{
	vTaskResume(eeprom_save_task_handle);
}

/* This task is use for saving global datas into the EEPROM without interrupt
 * the communication with ground station */
void eeprom_save_task(void)
{
	while(1) {
		/* Ensure the data will fully writting into the eeprom by checking
		 * the first byte */
		eeprom.write('\0', 0, 1); //Clear the first byte

		int i;
		for(i = 0; i < get_global_data_count(); i++) {
			bool parameter_config;
			get_global_data_parameter_config_status(i, &parameter_config);

			/* Ensure the data will fully writting into the eeprom by checking
			 * the first byte */
			if(parameter_config == true)
				save_global_data_into_eeprom(i);
		}

		uint8_t global_data_count = get_global_data_count();
		eeprom.write(&global_data_count, 0, 1);

		vTaskSuspend(NULL);
	}
}
