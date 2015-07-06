#ifndef __EEPROM_TASK_H
#define __EEPROM_TASK_H

void eeprom_task_execute(void);
void eeprom_task_suspend(void);
bool is_eeprom_task_running(void);
void eeprom_save_task(void);

#endif
