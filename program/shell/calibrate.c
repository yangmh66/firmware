#include <string.h>

#include "usart.h"
#include "imu.h"

#include "global.h"
#include "eeprom_task.h"

#include "parser.h"
#include "shell.h"
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
	filtered_data->mag[0] = 
		(float)(new_unscaled_data->mag[0]) * offset_read_alpha + filtered_data->mag[0] * (1.0f - offset_read_alpha);
	filtered_data->mag[1] = 
		(float)(new_unscaled_data->mag[1]) * offset_read_alpha + filtered_data->mag[1] * (1.0f - offset_read_alpha);
	filtered_data->mag[2] = 
		(float)(new_unscaled_data->mag[2]) * offset_read_alpha + filtered_data->mag[2] * (1.0f - offset_read_alpha);
}

static void accel_calibrate(void)
{
	calibrate_unscaled_data_max.acc[0] = calibrate_unscaled_data_min.acc[0] = (float)imu_unscaled_data.acc[0];
	calibrate_unscaled_data_max.acc[1] = calibrate_unscaled_data_min.acc[1] = (float)imu_unscaled_data.acc[1];
	calibrate_unscaled_data_max.acc[2] = calibrate_unscaled_data_min.acc[2] = (float)imu_unscaled_data.acc[2];

	int print_delay = 0;

	char axis[3] = {'x', 'y', 'z'};
	char buffer, confirm_result;

	int i;
	for(i = 0; i < 3; i++) {

		serial1.printf("Turn %c axis of drone to the ground and hold still\n\r", axis[i]);
		serial1.printf("Press any key to start the calibration\n\r");

		serial1.getch();

		while(1) {
			//Low pass filter
			imu_calibrate_low_pass_filter(&imu_unscaled_data, &filtered_unscaled_data);

			//Search for maximum unscaled value
			if(filtered_unscaled_data.acc[i] > calibrate_unscaled_data_max.acc[i])
				calibrate_unscaled_data_max.acc[i] = filtered_unscaled_data.acc[i];
			//Search for minimum unscaled value
			else if(filtered_unscaled_data.acc[i] < calibrate_unscaled_data_min.acc[i])
				calibrate_unscaled_data_min.acc[i] = filtered_unscaled_data.acc[i];

			/* Interacting with the user */
			buffer = serial1.receive();
			if(buffer == 'n' || buffer == 'N') {
				if(i != 2) {
					confirm_result = shell_confirm("Finish calibrating and want to go to the next step? (y/n):");
					
					if(confirm_result == 'y' || confirm_result == 'Y') break;
				} else {
					confirm_result = shell_confirm("Do you want to save the calibration result? (y/n):");

					if(confirm_result == 'y' || confirm_result == 'Y') {
						set_global_data_value(ACCEL_X_MAX, FLOAT, DATA_CAST(calibrate_unscaled_data_max.acc[0]));
						set_global_data_value(ACCEL_X_MIN, FLOAT, DATA_CAST(calibrate_unscaled_data_min.acc[0]));
						set_global_data_value(ACCEL_Y_MAX, FLOAT, DATA_CAST(calibrate_unscaled_data_max.acc[1]));
						set_global_data_value(ACCEL_Y_MIN, FLOAT, DATA_CAST(calibrate_unscaled_data_min.acc[1]));
						set_global_data_value(ACCEL_Z_MAX, FLOAT, DATA_CAST(calibrate_unscaled_data_max.acc[2]));
						set_global_data_value(ACCEL_Z_MIN, FLOAT, DATA_CAST(calibrate_unscaled_data_min.acc[2]));

						eeprom_task_execute();
					}

					return;
				}
			}	

			print_delay++;

			if(print_delay == 20000) {
				serial1.printf("\x1b[H\x1b[2J");
				serial1.printf("[%c max]%f\t[%c min:%f]\n\r",
					axis[i],
					axis[i],
					calibrate_unscaled_data_max.acc[i],
					calibrate_unscaled_data_min.acc[i]
				);
				serial1.printf("Please press \'n\' if you are satisfy with these calibration result\n\r");

				print_delay = 0;
			}
		}
	}
}

static void mag_calibrate(void)
{
	calibrate_unscaled_data_max.mag[0] = calibrate_unscaled_data_min.mag[0] = (float)imu_unscaled_data.mag[0];
	calibrate_unscaled_data_max.mag[1] = calibrate_unscaled_data_min.mag[1] = (float)imu_unscaled_data.mag[1];
	calibrate_unscaled_data_max.mag[2] = calibrate_unscaled_data_min.mag[2] = (float)imu_unscaled_data.mag[2];

	int print_delay = 0;
	char buffer;

	while(1) {
		//Low pass filter
		imu_calibrate_low_pass_filter(&imu_unscaled_data, &filtered_unscaled_data);

		//Search for maximum unscaled value of x axis
		if(filtered_unscaled_data.mag[0] > calibrate_unscaled_data_max.mag[0])
			calibrate_unscaled_data_max.mag[0] = filtered_unscaled_data.mag[0];
		//Search for minimum unscaled value of x axis
		else if(filtered_unscaled_data.mag[0] < calibrate_unscaled_data_min.mag[0])
			calibrate_unscaled_data_min.mag[0] = filtered_unscaled_data.mag[0];

		//Search for maximum unscaled value of y axis
		if(filtered_unscaled_data.mag[1] > calibrate_unscaled_data_max.mag[1])
			calibrate_unscaled_data_max.mag[1] = filtered_unscaled_data.mag[1];
		//Search for minimum unscaled value of y axis
		else if(filtered_unscaled_data.mag[1] < calibrate_unscaled_data_min.mag[1])
			calibrate_unscaled_data_min.mag[1] = filtered_unscaled_data.mag[1];

		//Search for maximum unscaled value of z axis
		if(filtered_unscaled_data.mag[2] > calibrate_unscaled_data_max.mag[2])
			calibrate_unscaled_data_max.mag[2] = filtered_unscaled_data.mag[2];
		//Search for minimum unscaled value of a axis
		else if(filtered_unscaled_data.mag[2] < calibrate_unscaled_data_min.mag[2])
			calibrate_unscaled_data_min.mag[2] = filtered_unscaled_data.mag[2];

		buffer = serial1.receive();
		if(buffer == 'q' || buffer == 'Q') {
			/* Confirm to save calibration results */
			char confirm_result = shell_confirm("Are you sure you want to save these calibration results? (y/n):");

			if(confirm_result == 'y' || confirm_result == 'Y') {
				set_global_data_value(MAG_X_MAX, FLOAT, DATA_CAST(calibrate_unscaled_data_max.mag[0]));
				set_global_data_value(MAG_X_MIN, FLOAT, DATA_CAST(calibrate_unscaled_data_min.mag[0]));
				set_global_data_value(MAG_Y_MAX, FLOAT, DATA_CAST(calibrate_unscaled_data_max.mag[1]));
				set_global_data_value(MAG_Y_MIN, FLOAT, DATA_CAST(calibrate_unscaled_data_min.mag[1]));
				set_global_data_value(MAG_Z_MAX, FLOAT, DATA_CAST(calibrate_unscaled_data_max.mag[2]));
				set_global_data_value(MAG_Z_MIN, FLOAT, DATA_CAST(calibrate_unscaled_data_min.mag[2]));

				eeprom_task_execute();

				return;
			}
		}

		print_delay++;

		if(print_delay == 20000) {
			serial1.printf("\x1b[H\x1b[2J");
			serial1.printf("[x max]%f\t[x min]%f\n\r", calibrate_unscaled_data_max.mag[0], calibrate_unscaled_data_min.mag[0]);
			serial1.printf("[y max]%f\t[y min]%f\n\r", calibrate_unscaled_data_max.mag[1], calibrate_unscaled_data_min.mag[1]);
			serial1.printf("[z max]%f\t[z min]%f\n\r", calibrate_unscaled_data_max.mag[2], calibrate_unscaled_data_min.mag[2]);
			serial1.printf("Please press \'q\' to save the calibration results\n\r");

			print_delay = 0;
		}
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

	char confirm_result = shell_confirm("Are you sure you want do the calibration? (y/n):");

	if(confirm_result == 'n' || confirm_result == 'N') return;

	switch(calibrate_mode) {
	    case ACCEL_CALIBRATE:
		accel_calibrate();
		break;
	    case MAG_CALIBRATE:
		mag_calibrate();
		break;
	    case RC_CALIBRATE:
		break;
	}
}
