#include <string.h>

#include "usart.h"
#include "input_capture.h"
#include "imu.h"

#include "rc_config.h"

#include "global.h"
#include "eeprom_task.h"

#include "parser.h"
#include "shell.h"
#include "calibrate.h"

enum {ACCEL_CALIBRATE, MAG_CALIBRATE, RC_CALIBRATE};

extern imu_unscaled_data_t imu_unscaled_data;

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
	imu_data_t filtered_unscaled_data;
	imu_data_t calibrate_unscaled_data_max;
	imu_data_t calibrate_unscaled_data_min;

	calibrate_unscaled_data_max.acc[0] = calibrate_unscaled_data_min.acc[0] = (float)imu_unscaled_data.acc[0];
	calibrate_unscaled_data_max.acc[1] = calibrate_unscaled_data_min.acc[1] = (float)imu_unscaled_data.acc[1];
	calibrate_unscaled_data_max.acc[2] = calibrate_unscaled_data_min.acc[2] = (float)imu_unscaled_data.acc[2];

	int print_delay = 0;
	int current_index = 0; //0:x, 1:y, 2:z
	int face_side = 0; //0: Find the max value, 1: find the min value

	char axis[3] = {'x', 'y', 'z'};
	char buffer, confirm_result;

	int i;
	for(i = 0; i < 6; i++) {
		face_side = i % 2;
		current_index = i / 2;

		serial1.printf("\x1b[H\x1b[2J");
		if(face_side == 0)
			serial1.printf("Make %c axis face to the ground straightly and hold still\n\r", axis[current_index]);
		else
			serial1.printf("Reverse to make %c axis face upward and hold still\n\r", axis[current_index]);
		serial1.printf("Press any key to start the calibration\n\r");

		serial1.getch();

		//Pre-filtering
		int j;
		for(j = 0; j < 1000; j++)
			imu_calibrate_low_pass_filter(&imu_unscaled_data, &filtered_unscaled_data);

		while(1) {
			//Low pass filter
			imu_calibrate_low_pass_filter(&imu_unscaled_data, &filtered_unscaled_data);

			/* Calibration, find out the max value and the min value */
			if(face_side == 0) {
				//Search for maximum unscaled value
				if(filtered_unscaled_data.acc[current_index] > calibrate_unscaled_data_max.acc[current_index])
					calibrate_unscaled_data_max.acc[current_index] = filtered_unscaled_data.acc[current_index];
			} else {
				//Search for minimum unscaled value
				if(filtered_unscaled_data.acc[current_index] < calibrate_unscaled_data_min.acc[current_index])
					calibrate_unscaled_data_min.acc[current_index] = filtered_unscaled_data.acc[current_index];
			}

			/* Interacting with the user */
			buffer = serial1.receive();
			if(buffer == 'n' || buffer == 'N') {
				confirm_result = shell_confirm("Finish and want to go to the next step? (y/n):");

				/* Is this the last step? */
				if(current_index == 2 && face_side == 1 && (confirm_result == 'y' || confirm_result == 'Y')) {
					/* Yes, save the data */
					serial1.printf("\x1b[H\x1b[2J");
					serial1.printf("Calibration result:\n\r");
		                        serial1.printf("[x max]%f\t[x min]%f\n\r",
						calibrate_unscaled_data_max.acc[0], calibrate_unscaled_data_min.acc[0]);
                        		serial1.printf("[y max]%f\t[y min]%f\n\r",
						calibrate_unscaled_data_max.acc[1], calibrate_unscaled_data_min.acc[1]);
		                        serial1.printf("[z max]%f\t[z min]%f\n\r",
						calibrate_unscaled_data_max.acc[2], calibrate_unscaled_data_min.acc[2]);

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
				} else {
					/* No, keep going */
					if(confirm_result == 'y' || confirm_result == 'Y') break;
				}

			}	

			print_delay++;

			if(print_delay == 20000) {
				serial1.printf("\x1b[H\x1b[2J");
				
				if(face_side == 0) {
					serial1.printf("[%c max]%f\n\r",
						axis[current_index], calibrate_unscaled_data_max.acc[current_index]);
				} else {
					serial1.printf("[%c min]%f\n\r",
						axis[current_index], calibrate_unscaled_data_min.acc[current_index]);
				}

				serial1.printf("Please press \'n\' if you are satisfy with these calibration results\n\r");

				print_delay = 0;
			}
		}
	}
}

static void mag_calibrate(void)
{
	imu_data_t filtered_unscaled_data;
	imu_data_t calibrate_unscaled_data_max;
	imu_data_t calibrate_unscaled_data_min;

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

static void rc_calibrate_low_pass_filter(float new_data, float *filtered_data)
{
	float offset_read_alpha = 0.001f;

	*filtered_data = new_data * offset_read_alpha + *filtered_data * (1.0f - offset_read_alpha);
}

static void rc_calibrate(void)
{
	int print_delay = 0;

	char buffer;

	float rc_channel1_max, rc_channel1_neutrul, rc_channel1_min;
	float rc_channel2_max, rc_channel2_neutrul, rc_channel2_min;
	float rc_channel3_max, rc_channel3_min; //Throttle
	float rc_channel4_max, rc_channel4_neutrul, rc_channel4_min;
	float rc_channel5_max, rc_channel5_neutrul, rc_channel5_min; //Safey button
	float rc_channel6_max, rc_channel6_neutrul, rc_channel6_min, rc_channel6_temp; //Auto-pilot mode

	rc_channel1_max = rc_channel1_neutrul = rc_channel1_min = RC_CHANNEL_1_INPUT_CAPTURE;
	rc_channel2_max= rc_channel2_neutrul= rc_channel2_min = RC_CHANNEL_2_INPUT_CAPTURE;
	rc_channel3_max= rc_channel3_min = RC_CHANNEL_3_INPUT_CAPTURE;
	rc_channel4_max= rc_channel4_neutrul= rc_channel4_min = RC_CHANNEL_4_INPUT_CAPTURE;
	rc_channel5_max= rc_channel5_neutrul= rc_channel5_min = RC_CHANNEL_5_INPUT_CAPTURE;
	rc_channel6_max= rc_channel6_neutrul= rc_channel6_min = RC_CHANNEL_6_INPUT_CAPTURE;

	int i;
	for(i = 0; i < 4; i++) {
		serial1.printf("\x1b[H\x1b[2J");

		switch(i) {
		    case 0:
			serial1.printf("\x1b[H\x1b[2J");
			serial1.printf("Step1: Calibrate joystick's neutrul value\n\r");
			serial1.printf("[Please reset radio controller's postion]\n\r");
			break;
		    case 1:
			serial1.printf("Step2: Calibrate joystick's maximum value and minimum value\n\r");
			serial1.printf("[Please move the joystick to find out the extreme value]\n\r");
			break;
		    case 2:
			serial1.printf("Step3: Calibrate safety button's maximum value and minimum value\n\r");
			serial1.printf("[Please switch the button while start calibrating]\n\r");
			break;
		    case 3:
			serial1.printf("Step4: Calibrate mode button's maximum value, neutrul value and minimum value\n\r");
			serial1.printf("[Pleas switch the button to the change mode while start calibrating]\n\r");
			break;
		}

		serial1.printf("Press any key to start the calibration\n\r");
		serial1.getch();

		while(1) {
			if(i == 0) {
				/* Step1. Detect neutrul value of the joystick */
				rc_calibrate_low_pass_filter(RC_CHANNEL_1_INPUT_CAPTURE, &rc_channel1_neutrul);
				rc_calibrate_low_pass_filter(RC_CHANNEL_2_INPUT_CAPTURE, &rc_channel2_neutrul);
				rc_calibrate_low_pass_filter(RC_CHANNEL_4_INPUT_CAPTURE, &rc_channel4_neutrul);
			} else if(i == 1) {
				/* Step2. Detect maximum value and minimum value of the joystick */
				if(RC_CHANNEL_1_INPUT_CAPTURE > rc_channel1_max)
					rc_channel1_max = RC_CHANNEL_1_INPUT_CAPTURE;
				else if(RC_CHANNEL_1_INPUT_CAPTURE < rc_channel1_min)
					rc_channel1_min = RC_CHANNEL_1_INPUT_CAPTURE;

				if(RC_CHANNEL_2_INPUT_CAPTURE > rc_channel2_max)
					rc_channel2_max = RC_CHANNEL_2_INPUT_CAPTURE;
				else if(RC_CHANNEL_2_INPUT_CAPTURE < rc_channel2_min)
					rc_channel2_min = RC_CHANNEL_2_INPUT_CAPTURE;

				if(RC_CHANNEL_3_INPUT_CAPTURE > rc_channel3_max)
					rc_channel3_max = RC_CHANNEL_3_INPUT_CAPTURE;
				else if(RC_CHANNEL_3_INPUT_CAPTURE < rc_channel3_min)
					rc_channel3_min = RC_CHANNEL_3_INPUT_CAPTURE;

				if(RC_CHANNEL_4_INPUT_CAPTURE > rc_channel4_max)
					rc_channel4_max = RC_CHANNEL_4_INPUT_CAPTURE;
				else if(RC_CHANNEL_4_INPUT_CAPTURE < rc_channel4_min)
					rc_channel4_min = RC_CHANNEL_4_INPUT_CAPTURE;
			} else if(i == 2) {
				/* Step3. Detect maximum value and minimum value of the safety button */
				if(RC_CHANNEL_5_INPUT_CAPTURE > rc_channel5_max)
					rc_channel5_max = RC_CHANNEL_5_INPUT_CAPTURE;
				else if(RC_CHANNEL_5_INPUT_CAPTURE < rc_channel5_min)
					rc_channel5_min = RC_CHANNEL_5_INPUT_CAPTURE;
			} else if (i == 3) {
				float whole_scale, test_scale;

				/* Step4. Detect maximum value, neutrul value and minimum value of the mode button */
				if(RC_CHANNEL_6_INPUT_CAPTURE > rc_channel6_max) {
					/* Just a tricky math...
					 * 1 scale = maximum value - minimum value
					 * 0.5 scale = neutrul value - minimum value
					 * So, If the condition (2/3 scale > 1/2 scale > 1/3 scale) is satisfied...
					 * Then we found the neutrul value */
					whole_scale = RC_CHANNEL_6_INPUT_CAPTURE - rc_channel6_min;
					test_scale = rc_channel6_max - rc_channel6_min;
					if((test_scale < whole_scale * 0.666) && (test_scale > whole_scale * 0.333))
						rc_channel6_neutrul = rc_channel6_max;

					//Find the maximum value!
					rc_channel6_max = RC_CHANNEL_6_INPUT_CAPTURE;
				} else if(RC_CHANNEL_6_INPUT_CAPTURE < rc_channel6_min) {
					/* Just a tricky math...
					 * 1 scale = maximum value - minimum value
					 * 0.5 scale = neutrul value - minimum value
					 * So, If the condition (2/3 scale > 1/2 scale > 1/3 scale) is satisfied...
					 * Then we found the neutrul valu */
					whole_scale = rc_channel6_max - RC_CHANNEL_6_INPUT_CAPTURE;
					test_scale = rc_channel6_min - RC_CHANNEL_6_INPUT_CAPTURE;
					if((test_scale < whole_scale * 0.666) && (test_scale > whole_scale * 0.333))
						rc_channel6_neutrul = rc_channel6_min;

					//Find the minumum value!
					rc_channel6_min = RC_CHANNEL_6_INPUT_CAPTURE;
				}
			}

			buffer = serial1.receive();
			if(buffer == 'n' || buffer == 'N') {
				/* Confirm to save calibration results */
				char confirm_result = shell_confirm("Are you sure you want to save these calibration results? (y/n):");

				if(confirm_result == 'y' || confirm_result == 'Y') break;
			}

			print_delay++;

			if(print_delay == 20000) {
				serial1.printf("\x1b[H\x1b[2J");

				if(i == 0) {
					serial1.printf("[channel1 neutrul]%f\n\r", rc_channel1_neutrul);
					serial1.printf("[channel2 neutrul]%f\n\r", rc_channel2_neutrul);
					serial1.printf("[channel4 neutrul]%f\n\r", rc_channel4_neutrul);
				} else if(i == 1) {
					serial1.printf("[channel1 max]%f\t[channel1 min]%f\n\r", rc_channel1_max, rc_channel1_min);
					serial1.printf("[channel2 max]%f\t[channel2 min]%f\n\r", rc_channel2_max, rc_channel2_min);
					serial1.printf("[channel3 max]%f\t[channel3 min]%f\n\r", rc_channel3_max, rc_channel3_min);
					serial1.printf("[channel4 max]%f\t[channel4 min]%f\n\r", rc_channel4_max, rc_channel4_min);
				} else if(i == 2) {
					serial1.printf("[safety button max]%f\t[safety button min]%f\n\r",
						rc_channel5_max, rc_channel5_min);
				} else if(i == 3) {
					serial1.printf("[mode button max]%f\t[mode button neutrul]%f\t[mode button min]%f\n\r",
						rc_channel6_max, rc_channel6_neutrul, rc_channel6_min);
				}

				serial1.printf("Please press \'n\' if you are satisfy with these calibration results\n\r");

				print_delay = 0;
			}
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
		rc_calibrate();
		break;
	}
}
