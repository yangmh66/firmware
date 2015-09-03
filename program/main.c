//#define DEBUG
#include "stm32f4xx_conf.h"
#include "../common/delay.h"
#include "gpio.h"
#include "led.h"
#include "i2c.h"
#include "usart.h"
#include "spi.h"
#include "can.h"
#include "tim.h"
#include "flight_controller.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "global.h"
#include "communication.h"
#include "eeprom_task.h"
#include "system_time.h"
#include "lea6h_ubx.h"

extern uint8_t estimator_trigger_flag;

/* FreeRTOS */
xTaskHandle eeprom_save_task_handle;
xTimerHandle xTimers[1];
extern xSemaphoreHandle serial_tx_wait_sem;
extern xSemaphoreHandle usart3_dma_send_sem;
extern xSemaphoreHandle eeprom_sem;
extern xQueueHandle serial_rx_queue;
extern xQueueHandle gps_serial_queue;

void vApplicationStackOverflowHook( xTaskHandle xTask, signed char *pcTaskName );
void vApplicationIdleHook(void);
void vApplicationMallocFailedHook(void);
void boot_time_timer(void);
void gpio_rcc_init(void);

void gpio_rcc_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | 
	RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE,  ENABLE);	
}

void vApplicationStackOverflowHook( xTaskHandle Task __attribute__ ((unused)), signed char *pcTaskName __attribute__ ((unused)))
{
	while(1);

}
void vApplicationIdleHook(void)
{

}
void vApplicationMallocFailedHook(void)
{
	while(1);
}

int main(void)
{
	vSemaphoreCreateBinary(serial_tx_wait_sem);
	vSemaphoreCreateBinary(usart3_dma_send_sem);
	vSemaphoreCreateBinary(flight_control_sem);
	vSemaphoreCreateBinary(eeprom_sem);

	serial_rx_queue = xQueueCreate(256, sizeof(serial_msg));
	gps_serial_queue = xQueueCreate(5, sizeof(serial_msg));

	/* Hardware initialization */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	gpio_rcc_init();
	led_init();
	usart_init();
	spi_init();
	pwm_input_output_init();
	init_pwm_motor();
	i2c_Init();
	usart2_dma_init();
	CAN2_Config();
	CAN2_NVIC_Config();

	/* Global data initialazition */
	init_global_data();

	/* Register the FreeRTOS task */
	/* Flight control task */
	xTaskCreate(
		(pdTASK_CODE)flight_control_task,
		(signed portCHAR*)"flight control task",
		4096,
		NULL,
		tskIDLE_PRIORITY + 9,
		NULL
	);

	/* Navigation task */
	xTaskCreate(
		(pdTASK_CODE)navigation_task,
		(signed portCHAR*)"navigation task",
		512,
		NULL,
		tskIDLE_PRIORITY + 7,
		NULL
	);

	/* Ground station communication task */	
	xTaskCreate(
		(pdTASK_CODE)mavlink_receiver_task,
		(signed portCHAR *)"mavlink receiver task",
		2048,
		NULL,
		tskIDLE_PRIORITY + 6,
		NULL
	);

	xTaskCreate(
		(pdTASK_CODE)mavlink_broadcast_task,
		(signed portCHAR *)"mavlink broadcast task",
		1024,
		NULL,
		tskIDLE_PRIORITY + 5,
		NULL
	);

	xTaskCreate(
		(pdTASK_CODE)eeprom_save_task,
		(signed portCHAR *)"eeprom save task",
		1024,
		NULL,
		tskIDLE_PRIORITY + 4,
		&eeprom_save_task_handle
	);

	xTaskCreate(
		(pdTASK_CODE)gps_receive_task,

		(signed portCHAR *) "gps receive task",
		2048,
		NULL,
		tskIDLE_PRIORITY + 8, NULL

	);

	vTaskSuspend(eeprom_save_task_handle);

	vTaskStartScheduler();

	return 0;
}


