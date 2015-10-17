#include <stdbool.h>
#include <string.h>

#include "stm32f4xx_conf.h"

#include "i2c.h"

#include "AT24C04C.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "delay.h"

//While-loop with timeout mechanism
int timeout, error_flag;
#define TIMED(timeout_max, x) timeout = timeout_max; error_flag = 0; \
	while(x) { if(timeout-- == 0) {error_flag = 1; break;} }

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

/***************************************
 * DMA configuration / setup functions *
 ***************************************/
static void i2c1_dma_tx_setup(uint8_t *data, uint32_t count)
{
	//check TCIF bit. It should be RESET when we ask new DMA request.
	TIMED(0xFFFF, DMA_GetFlagStatus(EEPROM_DMA_TX_STREAM, EEPROM_TX_DMA_FLAG_TCIF) == SET);

	if(error_flag) {
		return;
	}

	DMA_InitTypeDef i2c_init_struct;
	i2c_init_struct.DMA_Channel = EEPROM_DMA_TX_CHANNEL;
  	i2c_init_struct.DMA_PeripheralBaseAddr = EEPROM_I2C_DR_ADDR;
  	i2c_init_struct.DMA_Memory0BaseAddr = (uint32_t)data;    /* This parameter will be configured durig communication */;
  	i2c_init_struct.DMA_DIR = DMA_DIR_MemoryToPeripheral; /* This parameter will be configured durig communication */
  	i2c_init_struct.DMA_BufferSize = count;              /* This parameter will be configured durig communication */
  	i2c_init_struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  	i2c_init_struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	i2c_init_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  	i2c_init_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  	i2c_init_struct.DMA_Mode = DMA_Mode_Normal;
  	i2c_init_struct.DMA_Priority = DMA_Priority_VeryHigh;
  	i2c_init_struct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  	i2c_init_struct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  	i2c_init_struct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  	i2c_init_struct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  	DMA_Init(EEPROM_DMA_TX_STREAM, &i2c_init_struct);
  	DMA_Cmd(EEPROM_DMA_TX_STREAM, ENABLE);
  	I2C_DMACmd(I2C1, ENABLE);
}

static void i2c1_dma_rx_setup(uint8_t *data, uint32_t count)
{
	//check TCIF bit. It should be RESET when we ask new DMA request.
	TIMED(0xFFFF, DMA_GetFlagStatus(EEPROM_DMA_RX_STREAM, EEPROM_RX_DMA_FLAG_TCIF) == SET);

	if(error_flag) {
		return;
	}

	DMA_InitTypeDef i2c_init_struct;
	i2c_init_struct.DMA_Channel = EEPROM_DMA_RX_CHANNEL;
  	i2c_init_struct.DMA_PeripheralBaseAddr = EEPROM_I2C_DR_ADDR;
  	i2c_init_struct.DMA_Memory0BaseAddr = (uint32_t)data;    /* This parameter will be configured durig communication */;
  	i2c_init_struct.DMA_DIR = DMA_DIR_PeripheralToMemory; /* This parameter will be configured durig communication */
  	i2c_init_struct.DMA_BufferSize = count;              /* This parameter will be configured durig communication */
  	i2c_init_struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  	i2c_init_struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	i2c_init_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  	i2c_init_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  	i2c_init_struct.DMA_Mode = DMA_Mode_Normal;
  	i2c_init_struct.DMA_Priority = DMA_Priority_VeryHigh;
  	i2c_init_struct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  	i2c_init_struct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  	i2c_init_struct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  	i2c_init_struct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  	DMA_Init(EEPROM_DMA_RX_STREAM, &i2c_init_struct);
 	I2C_DMALastTransferCmd(I2C1, ENABLE);
  	DMA_Cmd(EEPROM_DMA_RX_STREAM, ENABLE);
  	I2C_DMACmd(I2C1, ENABLE);
}

/***************************************
 * Interruption handlers (I2C and DMA) *
 ***************************************/
static void handle_eeprom_write_request(void)
{
	/* [Procedure]
	 * ----------------------------------------------------------------
	 * 1.Generate start condition
	 * 2.Send I2C device address
	 * 3.Enable DMA to send word address and datas
	 * 4.Generate stop condition
	 * ----------------------------------------------------------------
	 * For detailed description you should read the ST reference manual
	 */

	long higher_priority_task_woken = pdFALSE;

	/* I2C Event handler (Step by step) */
	switch(eeprom_device_info.state) {
	    case GENERATE_START_CONDITION:
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
			//Step2: send I2C device address
			I2C_Send7bitAddress(I2C1, eeprom_device_info.device_address, I2C_Direction_Transmitter);

			eeprom_device_info.state = SEND_DEVICE_ADDRESS; //Move to next step
		}
		break;
	    }
	    case SEND_DEVICE_ADDRESS: //I2C device address
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			eeprom_device_info.state = GENERATE_STOP_CONDITION; //Move to next step

			//DMA ISR will re-enable this after the job is finished
			I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);

			//Step3. enable DMA to send word address and datas
			i2c1_dma_tx_setup(eeprom_device_info.buffer, eeprom_device_info.buffer_count);
	
			if(error_flag) {
				/* Fail to receive 1-byte data */
				eeprom_device_info.operating_type = EEPROM_DEVICE_IDLE;
				eeprom_device_info.exit_status = EEPROM_I2C_FAILED;
				xSemaphoreGiveFromISR(eeprom_sem, &higher_priority_task_woken);
			}
		}
    		break;
	    }
	    case GENERATE_STOP_CONDITION:
	    {
		if(I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF)) {
			//Step4: generate stop condition
			I2C_GenerateSTOP(I2C1, ENABLE);

			/* Clear device information */
			eeprom_device_info.operating_type = EEPROM_DEVICE_IDLE;
			eeprom_device_info.exit_status = I2C_SUCCESS;

			xSemaphoreGiveFromISR(eeprom_sem, &higher_priority_task_woken);
		}
		break;
	    }
	}

	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

static void handle_eeprom_read_request(void)
{
	/* [Procedure]
	 * ----------------------------------------------------------------
	 * 1.Generate start condition
	 * 2.Send I2C device address
	 * 3.Send EEPROM address
	 * 4.Generate start condition again
	 * 5.Send I2C device address again
	 * 6.For 1 byte case, wait for the RXNE flag and receive the data
	 *   For n byte case, enable DMA to receive all datas
	 * 7.Generate stop condition
	 * ----------------------------------------------------------------
	 * For detailed description you should read the ST reference manual
	 */

	long higher_priority_task_woken = pdFALSE;

	/* I2C Event handler (Step by step) */
	switch(eeprom_device_info.state) {
	    case GENERATE_START_CONDITION:
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
			/* Step2 : send I2C device address  */
			I2C_Send7bitAddress(I2C1, eeprom_device_info.device_address, I2C_Direction_Transmitter);

			eeprom_device_info.state = SEND_DEVICE_ADDRESS; //Move to next step
		}
		break;
	    }
	    case SEND_DEVICE_ADDRESS: //I2C device address
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			I2C_Cmd(I2C1, ENABLE);

			//Step3: send EEPROM word address
			I2C_SendData(I2C1, eeprom_device_info.word_address);

			eeprom_device_info.state = SEND_WORD_ADDRESS; //Move to next step
		}
		break;
	    }
	    case SEND_WORD_ADDRESS: //EEPROM address
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
			//Step4: generate the start condition again
			I2C_GenerateSTART(I2C1, ENABLE);

			eeprom_device_info.state = GENERATE_START_CONDITION_AGAIN; //Move to next step
		}
		break;
	    }
	    case GENERATE_START_CONDITION_AGAIN:
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
			//Step5: Send I2C device address again
			I2C_Send7bitAddress(I2C1, eeprom_device_info.device_address, I2C_Direction_Receiver);

			if(eeprom_device_info.buffer_count == 1) {
				//Step6: 1 byte reception (Polling)
				eeprom_device_info.state = RECEIVE_ONE_BYTE_DATA;
			} else {
				//Step6: N byte reception (Using DMA)
				eeprom_device_info.state = RECEIVE_N_BYTE_DATA;
			}
		}
		break;
	    }
	    case RECEIVE_ONE_BYTE_DATA:
	    {
		if(I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR)) {
			//Setup NACK bit during EV6
			I2C_AcknowledgeConfig(I2C1, DISABLE);

			//Reading Register SR1 and SR2 in order to end the EV6
			I2C_ReadRegister(I2C1, I2C_Register_SR1);
			I2C_ReadRegister(I2C1, I2C_Register_SR2);

			//STOPF bit should be set after EV6
			I2C_GenerateSTOP(I2C1, ENABLE); //Step7: Generate stop condition

			//Waiting for 1-byte data by checking RXNE flag
			TIMED(0xFFFF, I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) != SET);

			if(error_flag) {
				/* Fail to receive 1-byte data */
				eeprom_device_info.operating_type = EEPROM_DEVICE_IDLE;
				eeprom_device_info.exit_status = EEPROM_I2C_FAILED;
				xSemaphoreGiveFromISR(eeprom_sem, &higher_priority_task_woken);

				break;
			}
			
			//Receive the data
			eeprom_device_info.buffer[0] = I2C_ReceiveData(I2C1);

			/* Update device information */
			eeprom_device_info.operating_type = EEPROM_DEVICE_IDLE;
			eeprom_device_info.exit_status = I2C_SUCCESS;

			xSemaphoreGiveFromISR(eeprom_sem, &higher_priority_task_woken);
		}
		break;
	    }
	    case RECEIVE_N_BYTE_DATA:
	    {
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			//DMA ISR will re-enable this after the job is finished
			I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);

			//Enable DMA to receive the data
			i2c1_dma_rx_setup(eeprom_device_info.buffer, eeprom_device_info.buffer_count);

			if(error_flag) {
				/* Fail to receive 1-byte data */
				eeprom_device_info.operating_type = EEPROM_DEVICE_IDLE;
				eeprom_device_info.exit_status = EEPROM_I2C_FAILED;
				xSemaphoreGiveFromISR(eeprom_sem, &higher_priority_task_woken);
			}
		}
	    }
	    break;
	}

	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

void EEPROM_I2C_IRQ_HANDLER(void)
{
	if(eeprom_device_info.operating_type == EEPROM_DEVICE_WRITE) {
		handle_eeprom_write_request();
	} else if(eeprom_device_info.operating_type == EEPROM_DEVICE_READ) {
		handle_eeprom_read_request();
	}
}

void DMA1_Stream7_IRQHandler(void)
{
	if(DMA_GetFlagStatus(EEPROM_DMA_TX_STREAM, EEPROM_TX_DMA_FLAG_TCIF) != RESET) {
  		DMA_Cmd(EEPROM_DMA_TX_STREAM, DISABLE);
  		DMA_ClearFlag(EEPROM_DMA_TX_STREAM, EEPROM_TX_DMA_FLAG_TCIF);
		I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
  	}
}

void DMA1_Stream0_IRQHandler(void)
{
	long higher_priority_task_woken = pdFALSE;

	if(DMA_GetFlagStatus(EEPROM_DMA_RX_STREAM, EEPROM_RX_DMA_FLAG_TCIF) != RESET) {
    		I2C_GenerateSTOP(I2C1, ENABLE); //Step7: Generate stop condition

    		DMA_Cmd(EEPROM_DMA_RX_STREAM, DISABLE);
    		DMA_ClearFlag(EEPROM_DMA_RX_STREAM, EEPROM_RX_DMA_FLAG_TCIF);

		I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);

		eeprom_device_info.operating_type = EEPROM_DEVICE_IDLE;
		eeprom_device_info.exit_status = I2C_SUCCESS;

		xSemaphoreGiveFromISR(eeprom_sem, &higher_priority_task_woken);
  	}

	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

/**************************************************************************
 * EEPROM low level operation's interface (trigger ISR to finish the job) *
 **************************************************************************/

/**
  * @brief  EEPROM low level i2c writting
  * @param  Data array (pointer), i2c device address, eeprom word address and count
  *	    of the data
  * @retval Operating result
  */
static int eeprom_page_write(uint8_t *data, uint8_t device_address, uint8_t word_address,
	int data_count)
{
	//Wait until I2C is not busy anymore
	TIMED(0xFFFF, I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET);

	if(error_flag) {
		return I2C_BUSY_FAILED;
	}

	uint8_t send_buffer[EEPROM_PAGE_SIZE + 1] = {0}; //Reserve 1 for word address
	send_buffer[0] = word_address; //Word address
	memcpy(send_buffer + 1, data, data_count); //Payload

	/* Set EEPROM device information */
	eeprom_device_info.operating_type = EEPROM_DEVICE_WRITE;
	eeprom_device_info.state = GENERATE_START_CONDITION;
	eeprom_device_info.device_address = device_address;
	eeprom_device_info.word_address = word_address;
	eeprom_device_info.buffer = send_buffer;
	eeprom_device_info.buffer_count = data_count + 1;
	eeprom_device_info.sent_count = 0;

	//Step1: generate the start condition
	I2C_GenerateSTART(I2C1, ENABLE); //Trigger the I2C communication

	//I2C interrupt handler should finish the work in 1 millisecond
	while(!xSemaphoreTake(eeprom_sem, MILLI_SECOND_TICK)) {
		//Set the SWRST bit of I2C CR1 register to high to reset the I2C
		I2C_SoftwareResetCmd(I2C1, ENABLE);

		//XXX:Debug print

		return I2C_TIMEOUT_FAILED;
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	/* AT24C01's datasheet (page 8, byte write): http://www.atmel.com/Images/doc0134.pdf */
	vTaskDelay(MILLI_SECOND_TICK * 10); //Delay 10ms until the writting process is finished

	return eeprom_device_info.exit_status;
}

/**
  * @brief  EEPROM low level i2c reading
  * @param  buffer to store (pointer), i2c device address, eeprom word address and count
  *	    of the received data
  * @retval Operating result
  */
static int eeprom_sequential_read(uint8_t *buffer, uint8_t device_address, uint8_t word_address,
	int buffer_count)
{
	//Wait until I2C is not busy anymore
	TIMED(0xFFFF, I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET);

	if(error_flag) {
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
	while(!xSemaphoreTake(eeprom_sem, MILLI_SECOND_TICK)) {
		//Set the SWRST bit of I2C CR1 register to high to reset the I2C
		I2C_SoftwareResetCmd(I2C1, ENABLE);

		//XXX:Debug print

		return I2C_TIMEOUT_FAILED;
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	return eeprom_device_info.exit_status;
}

/*************************************************************************************************
 * EEPROM high level interface (page management and call low level functions to finish the work) *
 *************************************************************************************************/
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
			TIMED(0xFF, eeprom_page_write(&data[count - data_left], device_address, word_address, page_left_space) != I2C_SUCCESS);

			data_left -= EEPROM_PAGE_SIZE - current_page_write_byte;

			/* Point to next page */
			current_write_page++;
			current_page_write_byte = 0;
		} else {
			/* Write the data into current page */
			TIMED(0xFF, eeprom_page_write(&data[count - data_left], device_address, word_address, data_left) != I2C_SUCCESS);

			/* Increase the EEPROM page offset */
			current_page_write_byte += data_left;
		}
	}

	if(error_flag) {
		return EEPROM_I2C_FAILED;
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
			TIMED(0xFF, eeprom_sequential_read(buffer, device_address, word_address, page_left_space) != I2C_SUCCESS);

			/* Return the data */
			memcpy(data + (count - data_left), buffer, page_left_space);
			data_left -= page_left_space;

			/* Point the current EEPROM page to next page */
			current_read_page++;
			current_page_read_byte = 0;			
		} else {
			/* There will be some empty space in this page after the read
			   operation */
			TIMED(0xFF, eeprom_sequential_read(buffer, device_address, word_address, data_left) != I2C_SUCCESS);

			/* Return the data */
			memcpy(data + (count - data_left), buffer, data_left);

			/* Increase the current EEPROM page offset */
			current_page_read_byte += data_left;
		}
	}

	if(error_flag) {
		return EEPROM_I2C_FAILED;
	}

	return EEPROM_SUCCESS;
}

/****************************
 * EEPROM utility functions *
 ****************************/
void eeprom_clear(void)
{
        //Clear EEPROM
	uint8_t buffer[1024] = {'\0'};
	eeprom.write(buffer, 0, 1024);
}

void test_eeprom()
{
	char *str = "hello world";
	uint16_t size = strlen(str);
	uint16_t addr = 0;
	int16_t i = 0;
	uint16_t err_count=0;

	eeprom.clear();

	for (i = 0; i<50; i++) {
		eeprom.write((uint8_t * )str, addr, size);
		addr = addr + size;
	}

	char input_str[20];
	
	addr = 0;
	for (i = 0; i<50; i++) {
		eeprom.read((uint8_t * )input_str, addr, size);
		if( strcmp(str, input_str) != 0 ) {
			printf("ADDR:%u has problem!\r\n", addr);
			err_count++;
		}
		addr = addr+ size;
	}
	printf("finish eeprom testing. We have %u wrong sentences.\r\n", err_count);
}
