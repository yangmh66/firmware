#ifndef FILE_PID_PARAMETER_H
#define FILE_PID_PARAMETER_H

void PID_init(attitude_stablizer_pid_t* ,attitude_stablizer_pid_t* ,attitude_stablizer_pid_t* ,attitude_stablizer_pid_t* ,vertical_pid_t* ,vertical_pid_t* ,nav_pid_t*);
void PID_control_parameter_update(attitude_stablizer_pid_t* PID_roll,attitude_stablizer_pid_t* PID_pitch,attitude_stablizer_pid_t* PID_yaw_rate,attitude_stablizer_pid_t* PID_heading,vertical_pid_t* PID_Z,vertical_pid_t* PID_Zd,nav_pid_t* PID_nav);

#endif
