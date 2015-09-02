#include <string.h>

#include "stm32f4xx_conf.h"

#include "i2c.h"

#include "AT24C04C.h"

#include "delay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware abstraction */
#define EEPROM_I2C_IRQ_HANDLER I2C1_EV_IRQHandler

xSemaphoreHandle eeprom_sem = NULL;

eeprom_device_info_t eeprom_device_info;

/* I2C Timeout exception */
typedef enum {I2C_SUCCESS, I2C_TIMEOUT} I2C_Status;
int i2c_timeout;
#define I2C_TIMED(x) i2c_timeout = 0xFFFF; \
while(x) { if(i2c_timeout-- == 0) { return I2C_TIMEOUT; } }

/* EEPROM Timeout exception */
int timeout;
#define TIMED(x, restart) timeout = 0xFFFF; while(x) { if(timeout-- == 0) break; restart;}

int eeprom_read(uint8_t *data, uint16_t eeprom_address, uint16_t count);
int eeprom_write(uint8_t *data, uint16_t eeprom_address, uint16_t count);
void eeprom_clear(void);

eeprom_t eeprom = {
	.read = eeprom_read,
	.write = eeprom_write,
	.clear = eeprom_clear
};

static void eeprom_i2c_restart(void)
{
		i2c1_reinit();
}

static void handle_eeprom_write_request(void)
{
	/* [Brief procedure]
	 * 1.Generate start condition
	 * 2.Send I2C device address
	 * 3.Send EEPROM address
	 * 4.Send all datas
	 * 5.Generate stop condition
	 */

	/* [Detailed procedure]
	 * 1.Generate start condition -> wait for event "I2C_EVENT_MASTER_MODE_SELECT"
	 * 2.Send I2C device address -> wait for event "I2C_Direction_Transmitter"
	 * 3.Wait for event "I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED"
	 * 4.Send EEPROM address -> wait for event "I2C_EVENT_MASTER_BYTE_TRANSMITTED" 
	 * 5.Send a byte to EEPROM -> wait for event "I2C_EVENT_MASTER_BYTE_TRANSMITTED"
	 * 6.Loop step 5 until finish sending the data
	 * 7.Generate stop condition -> wait for stop event
	 */

	//Please carefully read the procedure first then read the code below

	if(eeprom_device_info.operating_type != EEPROM_DEVICE_WRITE) {
		return;
	}
}

static void handle_eeprom_read_request(void)
{
	/* [Brief procedure]
	 * 1.Generate start condition
	 * 2.Send I2C device address
	 * 3.Send EEPROM address
	 * 4.Generate start condition again
	 * 5.Send I2C device address again
	 * 4.Receive all datas
	 * 5.Generate stop condition
	 */

	/* [Detailed Procedure]
	 * 1.Generate start condition -> wait for event "I2C_EVENT_MASTER_MODE_SELECT"
	 * 2.Send I2C device address -> wait for event "I2C_Direction_Transmitter"
	 * 3.Wait for event "I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED"
	 * 4.Send EEPROM address -> wait for event "I2C_EVENT_MASTER_BYTE_TRANSMITTED" 
	 * 5.Generate start condition again -> wait for event "I2C_EVENT_MASTER_MODE_SELECT"
	 * 6.Send I2C device address again -> wait for event "I2C_Direction_Transmitter"
	 * 7.Wait for event "I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED" again
	 * 8.Read a byte from EEPROM -> wait for event "I2C_EVENT_MASTER_BYTE_RECEIVED"
	 * 9.Loop step 8 until finish receiving the data
	 * 10.Generate stop condition -> wait for stop event
	 */

	//Please carefully read the procedure first then read the code below

	if(eeprom_device_info.operating_type != EEPROM_DEVICE_READ) {
		return;
	}

	uint32_t current_event = I2C_GetLastEvent(I2C1);
	bool event_compare_failed = true;

	/* I2C Event handler */
	switch(eeprom_device_info.state) {
	    case GENERATE_START_CONDITION:
	    {
		if(current_event == I2C_EVENT_MASTER_MODE_SELECT) {
			I2C_Send7bitAddress(I2C1, eeprom_device_info.device_address, I2C_Direction_Transmitter);

			eeprom_device_info.state = SEND_DEVICE_ADDRESS;
			event_compare_failed = false;
		}
		break;
	    }
	    case SEND_DEVICE_ADDRESS: //I2C device address
	    {
		if(current_event == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) {
			I2C_Cmd(I2C1, ENABLE);
			I2C_SendData(I2C1, eeprom_device_info.word_address);

			eeprom_device_info.state = SEND_EEPROM_ADDRESS;
			event_compare_failed = false;
		}
		break;
	    }
	    case SEND_WORD_ADDRESS: //EEPROM address
	    {
		if(current_event == I2C_EVENT_MASTER_BYTE_TRANSMITTED) {
			I2C_GenerateSTART(I2C1, ENABLE);

			eeprom_device_info.state = GENERATE_START_CONDITION_AGAIN;
			event_compare_failed = false;
		}
		break;
	    }
	    case GENERATE_START_CONDITION_AGAIN:
	    {
		if(current_event == I2C_EVENT_MASTER_MODE_SELECT) {
			 I2C_Send7bitAddress(I2C1, eeprom_device_info.device_address, I2C_Direction_Transmitter);

			eeprom_device_info.state = SEND_DEVICE_ADDRESS;
			event_compare_failed = false;
		}
		break;
	    }
	    case SEND_DEVICE_ADDRESS_AGAIN:
	    {
		if(current_event == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) {
			*(eeprom_device_info.buffer) = I2C_ReceiveData(I2C1);
			eeprom_device_info.received_count++;

			eeprom_device_info.state = RECEIVE_DATA;
			event_compare_failed = false;
		}
		break;
	    }
	    case RECEIVE_DATA:
	    {
		if(current_event == I2C_EVENT_MASTER_BYTE_RECEIVED) {
			eeprom_device_info.buffer[eeprom_device_info.received_count] = I2C_ReceiveData(I2C1);
			eeprom_device_info.received_count++;

			if(eeprom_device_info.buffer_count == eeprom_device_info.received_count) {
				eeprom_device_info.operating_type = EEPROM_IDLE;
				xSemaphoreGiveFromISR(eeprom_sem, I_Dont_Know);
			}

			event_compare_failed = false;
			break;
		}
	    }
	}

	if(event_compare_failed == true) {
		//Unknown error, reset the I2C bus!
	}
}

void EEPROM_I2C_IRQ_HANDLER(void)
{
	printf_base("[I2C interrupt handler]");

	handle_eeprom_write_request();
	handle_eeprom_read_request();
}

/**
  * @brief  EEPROM low level i2c writting
  * @param  Data array (pass with pointer), i2c device address, eeprom word address and count
  *	    of the data
  * @retval Operating result
  */
static I2C_Status eeprom_page_write(uint8_t *data, uint8_t device_address, uint8_t word_address, 
	int data_count)
{
	//XXX:Check IDLE state?

	//Wait until I2C is not busy anymore
	I2C_TIMED(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	/* Set EEPROM device information */
	eeprom_device_info.state = EEPROM_DEVICE_WRITE;
	eeprom_device_info.address = device_address;
	eeprom_device_info.buffer = data;
	eeprom_device_info.buffer_count = data_count;

	I2C_GenerateSTART(I2C1, ENABLE);

	//TODO: Timeout check
	while (!xSemaphoreTake(eeprom_sem, portMAX_DELAY));

	return eeprom_device_info.exit_status;
}

/**
  * @brief  EEPROM low level i2c reading
  * @param  Store buffer(pointer), i2c device address, eeprom word address and count
  *	    of the received data
  * @retval Operating result
  */
static I2C_Status eeprom_sequential_read(uint8_t *buffer, uint8_t device_address, uint8_t word_address,
	int buffer_count)
{
	//Wait until I2C is not busy anymore
	I2C_TIMED(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	/* Set EEPROM device information */
	eeprom_device_info.state = EEPROM_DEVICE_READ;
	eeprom_device_info.address = device_address;
	eeprom_device_info.buffer = buffer;
	eeprom_device_info.buffer_count = buffer_count;

	I2C_GenerateSTART(I2C1, ENABLE);

	//TODO:Timeout check
	while (!xSemaphoreTake(eeprom_sem, portMAX_DELAY));

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
			TIMED(eeprom_page_write(page_buffer, device_address, word_address,
				page_left_space) == I2C_TIMEOUT, eeprom_i2c_restart());

			data_left -= EEPROM_PAGE_SIZE - current_page_write_byte;

			/* Point to next page */
			current_write_page++;
			current_page_write_byte = 0;
		} else {
			/* Write the data into current page */
			memcpy(page_buffer, data + (count - data_left), data_left);
			TIMED(eeprom_page_write(page_buffer, device_address, word_address,
				data_left) == I2C_TIMEOUT, eeprom_i2c_restart());

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
			/* The page is going to be full */
			
			TIMED(eeprom_sequential_read(buffer, device_address, word_address, page_left_space)
				== I2C_TIMEOUT, eeprom_i2c_restart());

			/* Return the data */
			memcpy(data + (count - data_left), buffer, page_left_space);
			data_left -= page_left_space;

			/* Point the current EEPROM page to next page */
			current_read_page++;
			current_page_read_byte = 0;			
		} else {
			/* There will be some empty space in this page after the read
			   operation */
			TIMED(eeprom_sequential_read(buffer, device_address, word_address, data_left)
				== I2C_TIMEOUT, eeprom_i2c_restart());

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
