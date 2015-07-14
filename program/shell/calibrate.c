#include <string.h>

#include "usart.h"
#include "imu.h"

#include "parser.h"
#include "calibrate.h"

enum {ACCEL_CALIBRATE, MAG_CALIBRATE, RC_CALIBRATE};

extern imu_unscaled_data_t imu_unscaled_data;

imu_data_t filtered_unscaled_data;
imu_data_t calibrate_unscaled_data_max;
imu_data_t calibrate_unscaled_data_min;

static void imu_calibrate_low_pass_filter(imu_unscaled_data_t *new_unscaled_data, imu_data_t *filtered_data)
{
	float offset_read_alpha = 0.001f;

	filtered_data->acc[0] = 
		(float)(new_unscaled_data->acc[0]) * offset_read_alpha + filtered_data->acc[0] * (1.0f - offset_read_alpha);
	filtered_data->acc[1] = 
		(float)(new_unscaled_data->acc[1]) * offset_read_alpha + filtered_data->acc[1] * (1.0f - offset_read_alpha);
	filtered_data->acc[2] = 
		(float)(new_unscaled_data->acc[2]) * offset_read_alpha + filtered_data->acc[2] * (1.0f - offset_read_alpha);
}

static void accel_calibrate(void)
{
	calibrate_unscaled_data_max.acc[0] = calibrate_unscaled_data_min.acc[0] = (float)imu_unscaled_data.acc[0];
	calibrate_unscaled_data_max.acc[1] = calibrate_unscaled_data_min.acc[1] = (float)imu_unscaled_data.acc[1];
	calibrate_unscaled_data_max.acc[2] = calibrate_unscaled_data_min.acc[2] = (float)imu_unscaled_data.acc[2];

	while(1) {
		//Low pass filter
		imu_calibrate_low_pass_filter(&imu_unscaled_data, &filtered_unscaled_data);

		//Search for maximum unscaled value of x axis
		if(filtered_unscaled_data.acc[0] > calibrate_unscaled_data_max.acc[0])
			calibrate_unscaled_data_max.acc[0] = filtered_unscaled_data.acc[0];
		//Search for minimum unscaled value of x axis
		else if(filtered_unscaled_data.acc[0] < calibrate_unscaled_data_min.acc[0])
			calibrate_unscaled_data_min.acc[0] = filtered_unscaled_data.acc[0];

		//Search for maximum unscaled value of y axis
		if(filtered_unscaled_data.acc[1] > calibrate_unscaled_data_max.acc[1])
			calibrate_unscaled_data_max.acc[1] = filtered_unscaled_data.acc[1];
		//Search for minimum unscaled value of y axis
		else if(filtered_unscaled_data.acc[1] < calibrate_unscaled_data_min.acc[1])
			calibrate_unscaled_data_min.acc[1] = filtered_unscaled_data.acc[1];

		//Search for maximum unscaled value of z axis
		if(filtered_unscaled_data.acc[2] > calibrate_unscaled_data_max.acc[2])
			calibrate_unscaled_data_max.acc[2] = filtered_unscaled_data.acc[2];
		//Search for minimum unscaled value of a axis
		else if(filtered_unscaled_data.acc[2] < calibrate_unscaled_data_min.acc[2])
			calibrate_unscaled_data_min.acc[2] = filtered_unscaled_data.acc[2];
		
		serial1.printf("\x1b[H\x1b[2J");
		serial1.printf("x max:%f x min:%f\n\r", calibrate_unscaled_data_max.acc[0], calibrate_unscaled_data_min.acc[0]);
		serial1.printf("y max:%f y min:%f\n\r", calibrate_unscaled_data_max.acc[1], calibrate_unscaled_data_min.acc[1]);
		serial1.printf("z max:%f z min:%f", calibrate_unscaled_data_max.acc[2], calibrate_unscaled_data_min.acc[2]);
		vTaskDelay(10);
	}
}

void shell_calibrate(char parameter[][MAX_CMD_LEN], int par_cnt)
{
	int calibrate_mode;

	if(par_cnt != 0) {
		if(strcmp(parameter[0], "accel") == 0) calibrate_mode = ACCEL_CALIBRATE;
		else if(strcmp(parameter[0], "mag") == 0) calibrate_mode = MAG_CALIBRATE;
		else if(strcmp(parameter[0], "rc") == 0) calibrate_mode = RC_CALIBRATE;
		else calibrate_mode = -1;
	};

	if(calibrate_mode == -1 || par_cnt == 0) {
		serial1.printf("type \"calibrate accel\" to calibrate the accelerometer\n\r");
		serial1.printf("type \"calibrate mag\" to calibrate the magnetometer\n\r");
		serial1.printf("type \"calibrate rc\" to calibrate the radio controller\n\r");
		return;
	}

	uint8_t confirm_result;
	do {
		serial1.printf("Are you sure you want do the calibration? (y/n):");
		confirm_result = serial1.getch();
		serial1.printf("%c\n\r", confirm_result);

		if(confirm_result == 'n' || confirm_result == 'N') return;
	} while(confirm_result != 'y' && confirm_result != 'Y');

	switch(calibrate_mode) {
	    case ACCEL_CALIBRATE:
		accel_calibrate();
		break;
	    case MAG_CALIBRATE:
		break;
	    case RC_CALIBRATE:
		break;
	}
}

