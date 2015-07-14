#include <string.h>

#include "usart.h"

#include "parser.h"
#include "calibrate.h"

enum {ACCEL_CALIBRATE, MAG_CALIBRATE, RC_CALIBRATE};

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
		break;
	    case MAG_CALIBRATE:
		break;
	    case RC_CALIBRATE:
		break;
	}
}

