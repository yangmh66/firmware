#include <string.h>
#include "stm32f4xx_conf.h"
#include "i2c.h"
#include "AT24C04C.h"
#include "delay.h"
void DMA1_Stream7_IRQHandler(void);
void I2C1_EV_IRQHandler(void);

/* I2C Timeout exception */
typedef enum {I2C_SUCCESS, I2C_TIMEOUT} I2C_Status;
int i2c_timeout;
#define I2C_TIMED(x) i2c_timeout = 0xFFFF; \
while(x) { if(i2c_timeout-- == 0) { return I2C_TIMEOUT; } }

/* EEPROM Timeout exception */
int timeout;
#define TIMED(x, restart) timeout = 0xFFFF; while(x) { if(timeout-- == 0) break; restart;}

/* EEPROM Information */
#define EEPROM_DEVICE_BASE_ADDRESS 0xA8
#define EEPROM_WORD_BASE_ADDRESS 0x00

#define EEPROM_PAGE_SIZE 16
#define EEPROM_MAX_SIZE 1024

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

void i2c1_dma_tx_setup(uint8_t *data, uint32_t count)
{
	//check TCIF bit. It should be RESET when we ask new DMA request.
	I2C_TIMED( DMA_GetFlagStatus(EEPROM_DMA_TX_STREAM, EEPROM_TX_DMA_FLAG_TCIF) == SET);

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

void i2c1_dma_rx_setup(uint8_t *data, uint32_t count)
{
	//check TCIF bit. It should be RESET when we ask new DMA request.
	I2C_TIMED( DMA_GetFlagStatus(EEPROM_DMA_RX_STREAM, EEPROM_RX_DMA_FLAG_TCIF) == SET);

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
static I2C_Status eeprom_page_write(uint8_t *data, uint8_t device_address, uint8_t word_address, 
	int data_count)
{
	I2C_TIMED(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	/* Send the I2C start condition */
	I2C_GenerateSTART(I2C1, ENABLE);
 	 
	/* Test on I2C EV5 and clear it */
	I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 
  
	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Transmitter);
  
	/* Test on I2C EV6 and clear it */
	I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  

	/* Send the EEPROM word address */    
	I2C_SendData(I2C1, word_address);  

	/* Test on I2C EV8 and clear it */
	I2C_TIMED(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));

	i2c1_dma_tx_setup(data, data_count);
	Delay_1us(5000);
	return I2C_SUCCESS;
}

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
			eeprom_page_write(page_buffer, device_address, word_address,
				page_left_space);

			data_left -= EEPROM_PAGE_SIZE - current_page_write_byte;

			/* Point to next page */
			current_write_page++;
			current_page_write_byte = 0;
		} else {
			/* Write the data into current page */
			memcpy(page_buffer, data + (count - data_left), data_left);
			eeprom_page_write(page_buffer, device_address, word_address,
				data_left);

			/* Increase the EEPROM page offset */
			current_page_write_byte += data_left;
		}
	}

	return EEPROM_SUCCESS;
}

static I2C_Status eeprom_sequential_read(uint8_t *buffer, uint8_t device_address, uint8_t word_address,
	int buffer_count)
{
	I2C_AcknowledgeConfig(I2C1,ENABLE);

	I2C_TIMED(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
 
	/* Send the I2C start condition */
	I2C_GenerateSTART(I2C1, ENABLE);
  
	/* Test on I2C EV5 and clear it */
	I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send the device address */
	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Transmitter);

	/* Test on I2C EV6 and clear it */
	I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
	/* Clear the I2C EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);

	/* Send the word_address */
	I2C_SendData(I2C1, word_address);  

	/* Test on I2C EV8 and clear it */
	I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
	/* Send the start condition a second time */  
	I2C_GenerateSTART(I2C1, ENABLE);
  
	/* Test on I2C EV5 and clear it */
	I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  
	/* Send the device address */
	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Receiver);
  
	/* Test on I2C EV6 and clear it */
	I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
	if(buffer_count < 2) {
		/* Disable Acknowledgement */
		I2C_AcknowledgeConfig(I2C1, DISABLE);
		I2C1->SR2;
		/* Send STOP Condition */
		I2C_GenerateSTOP(I2C1, ENABLE);

		/* Test on EV7 and clear it */
		I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

		/* Read a byte from the EEPROM */
		*buffer = I2C_ReceiveData(I2C1);

		/* Wait to make sure that STOP control bit has been cleared */
		I2C_TIMED(I2C1->CR1 & I2C_CR1_STOP);
		I2C_AcknowledgeConfig(I2C1, ENABLE);
	} else {

		i2c1_dma_rx_setup(buffer, buffer_count);
	}

	Delay_1us(5000);
	return I2C_SUCCESS;;
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
			
			eeprom_sequential_read(buffer, device_address, word_address, page_left_space);

			/* Return the data */
			memcpy(data + (count - data_left), buffer, page_left_space);
			data_left -= page_left_space;

			/* Point the current EEPROM page to next page */
			current_read_page++;
			current_page_read_byte = 0;			
		} else {
			/* There will be some empty space in this page after the read
			   operation */
			eeprom_sequential_read(buffer, device_address, word_address, data_left);

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
	if(DMA_GetFlagStatus(EEPROM_DMA_RX_STREAM, EEPROM_RX_DMA_FLAG_TCIF) != RESET) {
		
    		I2C_GenerateSTOP(I2C1, ENABLE);
    		DMA_Cmd(EEPROM_DMA_RX_STREAM, DISABLE);
    		DMA_ClearFlag(EEPROM_DMA_RX_STREAM, EEPROM_RX_DMA_FLAG_TCIF);
  	}
}
void I2C1_EV_IRQHandler(void)
{
	if(I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF) !=RESET) {

		I2C_GenerateSTOP(I2C1, ENABLE);
		I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);
	}
}
void test_eeprom()
{
	char *str = "hellow world";
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