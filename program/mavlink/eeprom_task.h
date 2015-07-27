#ifndef __EEPROM_TASK_H
#define __EEPROM_TASK_H


void eeprom_save_request(void);
bool check_eeprom_save_request(void);
void eeprom_task_execute_by_flight_control(void);
void eeprom_task_suspend_by_flight_control(void);
bool is_eeprom_task_running(void);
void eeprom_save_task(void);

#endif
