#include <stdio.h>

#include <math.h>

#include "usart.h"
#include "led.h"
#include "hmc5983.h"

#include "global.h"

void hmc5983_delay(volatile uint32_t count)
{

	while (count--) {

	}
}

uint8_t hmc5983_readByte(uint8_t addr){

	uint8_t data ;
	HMC5983_SELECT();
	SPI_xfer(HMC5983_SPI , addr| 0x80);
	data = SPI_xfer(HMC5983_SPI , 0x00);
	HMC5983_DESELECT();

	return data;
}



void hmc5983_initialize_config(void){

	HMC5983_SELECT();
	/* Writing configuration register A */
	SPI_xfer(HMC5983_SPI , HMC5983_CONFIG_REG_A| 0x00);
	/* Enabled temperature sensor, 8-sample averaged, 220Hz ODR */
	SPI_xfer(HMC5983_SPI , 0xFC);
	HMC5983_DESELECT();
	hmc5983_delay(100);

	HMC5983_SELECT();
	/* Writing configuration register B */
	SPI_xfer(HMC5983_SPI , HMC5983_CONFIG_REG_B| 0x00);
	/* Highest gain setting */
	SPI_xfer(HMC5983_SPI , 0x00);
	HMC5983_DESELECT();
	hmc5983_delay(100);

	HMC5983_SELECT();
	/* Writing configuration register B */
	SPI_xfer(HMC5983_SPI , HMC5983_MODE_REG| 0x00);
	/* Highest gain setting */
	SPI_xfer(HMC5983_SPI , 0x00);
	HMC5983_DESELECT();
	hmc5983_delay(100);

}


void hmc5983_update(imu_unscaled_data_t *imu_unscaledData){

 uint8_t hmc5983_buffer[6];

 	HMC5983_SELECT();
 	SPI_xfer(HMC5983_SPI , HMC5983_DATA_OUTOUT_X_H | 0xC0);
	hmc5983_buffer[0] = SPI_xfer(HMC5983_SPI , 0x00);
	hmc5983_buffer[1] = SPI_xfer(HMC5983_SPI , 0x00);
	hmc5983_buffer[2] = SPI_xfer(HMC5983_SPI , 0x00);
	hmc5983_buffer[3] = SPI_xfer(HMC5983_SPI , 0x00);
	hmc5983_buffer[4] = SPI_xfer(HMC5983_SPI , 0x00);
	hmc5983_buffer[5] = SPI_xfer(HMC5983_SPI , 0x00);

	HMC5983_DESELECT();

	imu_unscaledData->mag[0] = -(int16_t)(((uint16_t)hmc5983_buffer[0] << 8) | (uint16_t)hmc5983_buffer[1]);
	imu_unscaledData->mag[2] = -(int16_t)(((uint16_t)hmc5983_buffer[2] << 8) | (uint16_t)hmc5983_buffer[3]);
	imu_unscaledData->mag[1] =  (int16_t)(((uint16_t)hmc5983_buffer[4] << 8) | (uint16_t)hmc5983_buffer[5]);



}

void hmc5983_convert_to_scale(imu_unscaled_data_t *imu_unscaledData, imu_data_t *imu_scaledData, imu_calibrated_offset_t *imu_offset){



	imu_scaledData->mag[0]	= (float)(imu_unscaledData->mag[0]-imu_offset->mag[0]) * imu_offset->mag_scale[0];
	imu_scaledData->mag[1]	= (float)(imu_unscaledData->mag[1]-imu_offset->mag[1]) * imu_offset->mag_scale[1]; // correct with board orientation
	imu_scaledData->mag[2]	= (float)(imu_unscaledData->mag[2]-imu_offset->mag[2]) * imu_offset->mag_scale[2];


}


void hmc5983_apply_mag_calibration(imu_calibrated_offset_t *imu_offset){


	/* Example of data for current board

	Raw_Axis |  min   | max  |  average(offset) | 1-north scale	|>

	    X	   -728		664	-32		4087 //4094 (fine calibrated)
	    Y	   -881		534 	-174 		4091 // 4095 (fine calibrated)
	    Z	   -871  	442 	-215		4129 //4xxx (fine calibrated)
	
	*/

	float mag_x_min, mag_x_max;
	float mag_y_min, mag_y_max;
	float mag_z_min, mag_z_max;

	/* Get the true value from global data */
	read_global_data_value(MAG_X_MIN, DATA_POINTER_CAST(&mag_x_min));
	read_global_data_value(MAG_X_MAX, DATA_POINTER_CAST(&mag_x_max));
	read_global_data_value(MAG_Y_MIN, DATA_POINTER_CAST(&mag_y_min));
	read_global_data_value(MAG_Y_MAX, DATA_POINTER_CAST(&mag_y_max));
	read_global_data_value(MAG_Z_MIN, DATA_POINTER_CAST(&mag_z_min));
	read_global_data_value(MAG_Z_MAX, DATA_POINTER_CAST(&mag_z_max));

	/* Calculate the offset */
	imu_offset->mag[0] = (int16_t)((mag_x_min + mag_x_max) / 2);
	imu_offset->mag[1] = (int16_t)((mag_y_min + mag_y_max) / 2);
	imu_offset->mag[2]= (int16_t)((mag_z_min + mag_z_max) / 2);

	//Assume the scalue is 1.0
	imu_offset->mag_scale[0] = 1.0;
	imu_offset->mag_scale[1] = 1.0;
	imu_offset->mag_scale[2] = 1.0;
}


void hmc5983_initialize_system(imu_calibrated_offset_t *imu_offset){

	#ifndef USE_CAN_MAGNETOMETER
		hmc5983_initialize_config();
	#endif
	hmc5983_apply_mag_calibration(imu_offset);

}

void hmc5983_CAN_UpdateIMU(imu_unscaled_data_t *imu_raw_data){

	CanRxMsg RxMessage;
 	uint8_t hmc5983_buffer[6];

 		if( CAN2_CheckMessageStatusFlag(CAN_MESSAGE_MAGNETOMETER) == 1){

    			RxMessage =  CAN2_PassRXMessage(CAN_MESSAGE_MAGNETOMETER);
				CAN2_ClearMessageStatusFlag(CAN_MESSAGE_MAGNETOMETER);

				hmc5983_buffer[0] = RxMessage.Data[0];
				hmc5983_buffer[1] = RxMessage.Data[1];
				hmc5983_buffer[2] = RxMessage.Data[2];
				hmc5983_buffer[3] = RxMessage.Data[3];
				hmc5983_buffer[4] = RxMessage.Data[4];
				hmc5983_buffer[5] = RxMessage.Data[5];


				imu_raw_data->mag[0] = -(int16_t)(((uint16_t)hmc5983_buffer[0] << 8) | (uint16_t)hmc5983_buffer[1]);
				imu_raw_data->mag[2] = -(int16_t)(((uint16_t)hmc5983_buffer[2] << 8) | (uint16_t)hmc5983_buffer[3]);
				imu_raw_data->mag[1] =  (int16_t)(((uint16_t)hmc5983_buffer[4] << 8) | (uint16_t)hmc5983_buffer[5]);


 		}



}
