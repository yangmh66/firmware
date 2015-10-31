#if (AIRFRAME_SELECT == AIRFRAME_QUADROTOR)

#ifndef FILE_CONTROLLER_H
#define FILE_CONTROLLER_H

#include "pwm.h"
#include "radio_control.h"
#include "multirotor/attitude_stabilizer.h"
#include "vertical_stabilizer.h"
#include "led.h"
#include "global.h"

#define CONTROLLER_DISABLE 0
#define CONTROLLER_ENABLE 1

#include "navigation.h"

void PID_output(radio_controller_t* ,attitude_stablizer_pid_t* ,attitude_stablizer_pid_t* ,attitude_stablizer_pid_t* ,vertical_pid_t* );
void PID_rc_pass_command(attitude_t*, attitude_stablizer_pid_t*, attitude_stablizer_pid_t*, attitude_stablizer_pid_t*, attitude_stablizer_pid_t*, vertical_pid_t*, vertical_pid_t*, nav_pid_t*, radio_controller_t*);

#endif

#endif
