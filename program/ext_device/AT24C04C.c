#include <stdbool.h>
#include <string.h>

#include "stm32f4xx_conf.h"

#include "i2c.h"

#include "AT24C04C.h"

#include "delay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* EEPROM Timeout exception */
int timeout;
#define TIMED(x) timeout = 0xFF; while(x) \
	{ if(timeout-- == 0) {break; i2c_bus_reset(); return EEPROM_I2C_FAILED;} }

int eeprom_read(uint8_t *data, uint16_t eeprom_address, uint16_t count);
int eeprom_write(uint8_t *data, uint16_t eeprom_address, uint16_t count);
void eeprom_clear(void);

eeprom_t eeprom = {
	.read = eeprom_read,
	.write = eeprom_write,
	.clear = eeprom_clear
};

xSemaphoreHandle eeprom_sem = NULL;

eeprom_device_info_t eeprom_device_info;

static void i2c_bus_reset(void)
{
	//i2c1_reinit();
}

static void handle_eeprom_write_request(void)
{
	/* [Procedure]
	 * 1.Generate start condition -> wait for event "I2C_EVENT_MASTER_MODE_SELECT"
	 * 2.Send I2C device address -> wait for event "I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED"
	 * 3.Send EEPROM address -> wait for event "I2C_EVENT_MASTER_BYTE_TRANSMITTED" 
	 * 4.Send a byte to EEPROM -> wait for event "I2C_EVENT_MASTER_BYTE_TRANSMITTED"
	 * 5.Loop step 4 until finish sending the data
	 * 6.Generate stop condition -> wait for stop event
	 */

	//Please carefully view procedure first then read the code below

	if(eeprom_device_info.operating_type != EEPROM_DEVICE_WRITE) {
		return;
	}

	uint32_t current_event = I2C_GetLastEvent(I2C1);

	long higher_priority_task_woken = pdFALSE;

	/* I2C Event handler (Step by step) */
	switch(eeprom_device_info.state) {
	    case GENERATE_START_CONDITION:
	    {
		if(current_event == I2C_EVENT_MASTER_MODE_SELECT) {
			//Step2: send I2C device address
			I2C_Send7bitAddress(I2C1, eeprom_device_info.device_address, I2C_Direction_Transmitter);

			/* Update device information */
			eeprom_device_info.state = SEND_DEVICE_ADDRESS;
		}
		break;
	    }
	    case SEND_DEVICE_ADDRESS: //I2C device address
	    {
		if(current_event == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) {
			//Step3: send EEPROM word address
			I2C_SendData(I2C1, eeprom_device_info.word_address);

			/* Update device information */
			eeprom_device_info.state = SEND_WORD_ADDRESS;
		}
    		break;
	    }
	    case SEND_WORD_ADDRESS: //EEPROM address
	    {
		if(current_event == I2C_EVENT_MASTER_BYTE_TRANSMITTED) {
			//Step4: send first byte to the EEPROM
			I2C_SendData(I2C1, eeprom_device_info.buffer[0]);
			eeprom_device_info.sent_count++;

			/* Update device information */
			eeprom_device_info.state = SEND_DATA;
		}
		break;
	    }
	    case SEND_DATA:
	    {
		if(current_event == I2C_EVENT_MASTER_BYTE_RECEIVED) {
			/* Finish sending all datas */
			if(eeprom_device_info.sent_count == eeprom_device_info.buffer_count) {
				//Step6: generate the stop condition
				I2C_GenerateSTOP(I2C1, ENABLE);

				/* Clear device information */
				eeprom_device_info.operating_type = EEPROM_DEVICE_IDLE;
				eeprom_device_info.sent_count = 0;
				eeprom_device_info.exit_status = I2C_SUCCESS;

				xSemaphoreGiveFromISR(eeprom_sem, &higher_priority_task_woken);

				break;
			}

			//Step4-5: Keep sending the data
			I2C_SendData(I2C1, eeprom_device_info.buffer[eeprom_device_info.sent_count]);
			eeprom_device_info.sent_count++;
		}
		break;
	    }
	}

	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

static void handle_eeprom_read_request(void)
{
	/* [Procedure]
	 * 1.Generate start condition -> wait for event "I2C_EVENT_MASTER_MODE_SELECT"
	 * 2.Send I2C device address -> wait for event "I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED"
	 * 3.Send EEPROM address -> wait for event "I2C_EVENT_MASTER_BYTE_TRANSMITTED" 
	 * 4.Generate start condition again -> wait for event "I2C_EVENT_MASTER_MODE_SELECT"
	 * 5.Send I2C device address again -> wait for event "I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED"
	 * 6.Read a byte from EEPROM -> wait for event "I2C_EVENT_MASTER_BYTE_RECEIVED"
	 * 7.Loop step 6 until finish receiving the data
	 * 8.Generate stop condition -> wait for stop event
	 */

	//Please carefully view the procedure first then read the code below

	if(eeprom_device_info.operating_type != EEPROM_DEVICE_READ) {
		return;
	}

	long higher_priority_task_woken = pdFALSE;

	/* I2C Event handler (Step by step) */
	switch(eeprom_device_info.state) {
	    case GENERATE_START_CONDITION:
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
			/* Step2 : send I2C device address  */
			I2C_Send7bitAddress(I2C1, eeprom_device_info.device_address, I2C_Direction_Transmitter);

			/* Update device information */
			eeprom_device_info.state = SEND_DEVICE_ADDRESS;
		}
		break;
	    }
	    case SEND_DEVICE_ADDRESS: //I2C device address
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			I2C_Cmd(I2C1, ENABLE);
			//Step3: send EEPROM word address
			I2C_SendData(I2C1, eeprom_device_info.word_address);

			/* Update device information */
			eeprom_device_info.state = SEND_WORD_ADDRESS;
		}
		break;
	    }
	    case SEND_WORD_ADDRESS: //EEPROM address
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
			//Step4: generate the start condition again
			I2C_GenerateSTART(I2C1, ENABLE);

			/* Update device information */
			eeprom_device_info.state = GENERATE_START_CONDITION_AGAIN;
		}
		break;
	    }
	    case GENERATE_START_CONDITION_AGAIN:
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
			//Step5: Send I2C device address again
			I2C_Send7bitAddress(I2C1, eeprom_device_info.device_address, I2C_Direction_Receiver);

			/* Update device information */
			eeprom_device_info.state = SEND_DEVICE_ADDRESS_AGAIN;
		}
		break;
	    }
	    case SEND_DEVICE_ADDRESS_AGAIN:
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			//1 byte receive reception
			if(eeprom_device_info.buffer_count == 1) {
				//Setup NACK bit and Stop condition bit
				I2C_AcknowledgeConfig(I2C1, DISABLE);
				I2C_GenerateSTOP(I2C1, ENABLE);

				//For 1 byte reception, the byte should be received in EV6(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
				eeprom_device_info.buffer[0] = I2C_ReceiveData(I2C1);

				/* Update device information */
				eeprom_device_info.operating_type = EEPROM_DEVICE_IDLE;
				eeprom_device_info.exit_status = I2C_SUCCESS;

				xSemaphoreGiveFromISR(eeprom_sem, &higher_priority_task_woken);
			}
		}
		break;
	    }
	}

	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

void EEPROM_I2C_IRQ_HANDLER(void)
{
	handle_eeprom_write_request();
	handle_eeprom_read_request();
}

/**
  * @brief  EEPROM low level i2c writting
  * @param  Data array (pass with pointer), i2c device address, eeprom word address and count
  *	    of the data
  * @retval Operating result
  */
static int eeprom_page_write(uint8_t *data, uint8_t device_address, uint8_t word_address,
	int data_count)
{
	//XXX:Check IDLE state?
	if(eeprom_device_info.operating_type != EEPROM_DEVICE_IDLE) {
		while(1); //Debug code
	}

	//Wait until I2C is not busy anymore
	if(i2c_flag_loop_check(I2C1, I2C_FLAG_BUSY, 0xFFFF) != 0) {
		return I2C_BUSY_FAILED;
	}

	/* Set EEPROM device information */
	eeprom_device_info.operating_type = EEPROM_DEVICE_WRITE;
	eeprom_device_info.state = GENERATE_START_CONDITION;
	eeprom_device_info.device_address = device_address;
	eeprom_device_info.word_address = word_address;
	eeprom_device_info.buffer = data;
	eeprom_device_info.buffer_count = data_count;
	eeprom_device_info.sent_count = 0;

	//Step1: generate the start condition
	I2C_GenerateSTART(I2C1, ENABLE); //Trigger the I2C communication

	//I2C interrupt handler should finish the work in 1 millisecond
	while (!xSemaphoreTake(eeprom_sem, MILLI_SECOND_TICK)) {
		return I2C_TIMEOUT_FAILED;
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	return eeprom_device_info.exit_status;
}

/**
  * @brief  EEPROM low level i2c reading
  * @param  Store buffer(pointer), i2c device address, eeprom word address and count
  *	    of the received data
  * @retval Operating result
  */
static int eeprom_sequential_read(uint8_t *buffer, uint8_t device_address, uint8_t word_address,
	int buffer_count)
{
	if(eeprom_device_info.operating_type != EEPROM_DEVICE_IDLE) {
		while(1); //Debug code
	}

	//Wait until I2C is not busy anymore
	if(i2c_flag_loop_check(I2C1, I2C_FLAG_BUSY, 0xFFFF) != 0) {
		return I2C_BUSY_FAILED;
	}

	/* Set EEPROM device information */
	eeprom_device_info.operating_type = EEPROM_DEVICE_READ;
	eeprom_device_info.state = GENERATE_START_CONDITION;
	eeprom_device_info.device_address = device_address;
	eeprom_device_info.word_address = word_address;
	eeprom_device_info.buffer = buffer;
	eeprom_device_info.buffer_count = buffer_count;
	eeprom_device_info.received_count = 0;

	//Step1: generate the start condition
	I2C_GenerateSTART(I2C1, ENABLE); //Trigger the I2C communication

	//I2C interrupt handler should finish the work in 1 millisecond
	while (!xSemaphoreTake(eeprom_sem, MILLI_SECOND_TICK)) {
		//XXX:Timeout! Reset the bus
		return I2C_TIMEOUT_FAILED;
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	return eeprom_device_info.exit_status;
}

/*************************************
 * EEPROM high level page management *
 *************************************/
int eeprom_write(uint8_t *data, uint16_t eeprom_address, uint16_t count)
{
	/* Check the eeprom address is valid or not */
	if(eeprom_address > 1024)
		return EEPROM_INVALID_ADDRESS;

	/* Check the assigned data count size is valid or not */
	if((eeprom_address + count) > EEPROM_MAX_SIZE)
		return EEPROM_BUFFER_OVERFLOW;

	int data_left = count;

	/* Calculate the page count to store the data */
	int page_usage = count / EEPROM_PAGE_SIZE;
	page_usage += (count % EEPROM_PAGE_SIZE) > 0 ? 1 : 0; //Need to carry or not
	//Need to calculate for first page's offset
	if((eeprom_address % EEPROM_PAGE_SIZE) + (count % EEPROM_PAGE_SIZE) > EEPROM_PAGE_SIZE)
		page_usage++;

	/* Calulate the start page and page byte offset */
	uint8_t current_write_page = eeprom_address / EEPROM_PAGE_SIZE;
	//Get the byte offset of current write page
	uint8_t current_page_write_byte = eeprom_address % EEPROM_PAGE_SIZE;

	/* Page write operation */
	int used_page_count;
	for(used_page_count = 0; used_page_count < page_usage; used_page_count++) {

		uint8_t device_address = EEPROM_DEVICE_BASE_ADDRESS, word_address = 0x00;

		/* Current page information */
		uint8_t page_buffer[EEPROM_PAGE_SIZE] = {0};
		int page_left_space = EEPROM_PAGE_SIZE - current_page_write_byte;

		/* Calculate the device adrress and the word address (Only high 4 bit) */
		//Set device address bit 2 and 3
		device_address |= (current_write_page >> 4 << 1);
		//Set word address bit 5 to 8
		word_address |= current_write_page << 4;
		//Set word address bit 1 to 4
		word_address |= current_page_write_byte;

		/* Write the data in current page */
		if(data_left >= page_left_space) {
			/* Fill the full page by writing data */
			memcpy(page_buffer, data + (count - data_left), page_left_space);
			TIMED(eeprom_page_write(page_buffer, device_address, word_address, page_left_space) != I2C_SUCCESS);

			data_left -= EEPROM_PAGE_SIZE - current_page_write_byte;

			/* Point to next page */
			current_write_page++;
			current_page_write_byte = 0;
		} else {
			/* Write the data into current page */
			memcpy(page_buffer, data + (count - data_left), data_left);
			TIMED(eeprom_page_write(page_buffer, device_address, word_address, data_left) != I2C_SUCCESS);

			/* Increase the EEPROM page offset */
			current_page_write_byte += data_left;
		}
	}

	return EEPROM_SUCCESS;
}

int eeprom_read(uint8_t *data, uint16_t eeprom_address, uint16_t count)
{
	/* Check the eeprom address is valid or not */
	if(eeprom_address > 1024)
		return EEPROM_INVALID_ADDRESS;

	/* Check the assigned data count size is valid or not */
	if((eeprom_address + count) > EEPROM_MAX_SIZE)
		return EEPROM_BUFFER_OVERFLOW;

	int data_left = count;

	/* Calculate the page count to store the data */
	int page_usage = count / EEPROM_PAGE_SIZE;
	page_usage += (count % EEPROM_PAGE_SIZE) > 0 ? 1 : 0; //Need to carry or not
	//Need to calculate for first page's offset
	if((eeprom_address % EEPROM_PAGE_SIZE) + (count % EEPROM_PAGE_SIZE) > EEPROM_PAGE_SIZE)
		page_usage++;

	/* Calulate the start page and page byte offset */
	uint8_t current_read_page = eeprom_address / EEPROM_PAGE_SIZE;
	//Get the byte offset of current read page
	uint8_t current_page_read_byte = eeprom_address % EEPROM_PAGE_SIZE;

	/* Page read operation */
	int used_page_count;
	for(used_page_count = 0; used_page_count < page_usage; used_page_count++) {

		uint8_t device_address = EEPROM_DEVICE_BASE_ADDRESS, word_address = 0x00;

		uint8_t buffer[EEPROM_PAGE_SIZE] = {0};
		/* Calculate how many space can read in current EEPROM page */
		int page_left_space = EEPROM_PAGE_SIZE - current_page_read_byte;

		/* Calculate the device adrress and the word address */
		//Set device address bit 2 and 3
		device_address |= (current_read_page >> 4 << 1);
		//Set word address bit 5 to 8
		word_address |= current_read_page << 4;
		//Set word address bit 1 to 4;
		word_address |= current_page_read_byte;

		/* Read the data from the page */
		if(data_left >= page_left_space) {
			/* The page is going to full */
			
			TIMED(eeprom_sequential_read(buffer, device_address, word_address, page_left_space) != I2C_SUCCESS);

			/* Return the data */
			memcpy(data + (count - data_left), buffer, page_left_space);
			data_left -= page_left_space;

			/* Point the current EEPROM page to next page */
			current_read_page++;
			current_page_read_byte = 0;			
		} else {
			/* There will be some empty space in this page after the read
			   operation */
			TIMED(eeprom_sequential_read(buffer, device_address, word_address, data_left) != I2C_SUCCESS);

			/* Return the data */
			memcpy(data + (count - data_left), buffer, data_left);

			/* Increase the current EEPROM page offset */
			current_page_read_byte += data_left;
		}
	}

	return EEPROM_SUCCESS;
}

void eeprom_clear(void)
{
        //Clear EEPROM
	uint8_t buffer[1024] = {'\0'};
	eeprom.write(buffer, 0, 1024);
}
