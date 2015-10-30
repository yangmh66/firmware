#include "multirotor/attitude_stabilizer.h"
#include "fixed_wing/attitude_stabilizer.h"
#include "vertical_stabilizer.h"
#include "navigation.h"
#include "pid_parameter.h"

void PID_init(attitude_stablizer_pid_t* PID_roll,attitude_stablizer_pid_t* PID_pitch,attitude_stablizer_pid_t* PID_yaw_rate,attitude_stablizer_pid_t* PID_heading,vertical_pid_t* PID_Z,vertical_pid_t* PID_Zd,nav_pid_t* PID_nav){

	/* at this point all gain data are being updated by  PID_control_parameter_update() function */

	PID_roll -> kp =0.20f;
	PID_roll -> kd =0.07f;
	PID_roll -> ki =0.0;
	PID_roll -> setpoint =0.0;

	PID_pitch -> kp =0.20f;
	PID_pitch -> kd =0.07f;
	PID_pitch -> ki =0.0;
	PID_pitch -> setpoint =0.0;

	PID_yaw_rate -> kp =0.65f;
	PID_yaw_rate -> kd =0.0f;
	PID_yaw_rate -> ki =0.0;
	PID_yaw_rate -> setpoint =0.0;
	PID_yaw_rate -> out_max = 30.0f;
	PID_yaw_rate -> out_min = -30.0f;

	PID_heading -> kp = 2.5f;
	PID_heading -> kd = 0.0f;
	PID_heading -> ki = 0.0;
	PID_heading -> out_max = 50.0f;
	PID_heading -> out_min = -50.0f;
	PID_heading -> setpoint = 0.0;

	PID_Zd -> kp =0.3f;
	PID_Zd -> kd =0.0;
	PID_Zd -> ki =0.03;
	PID_Zd -> out_max = +20.0f;
	PID_Zd -> out_min = -20.0f;
	PID_Zd -> setpoint =0.0;

	PID_Z -> kp =1.4f;//1.8f;
	PID_Z -> kd =0.0;
	PID_Z -> ki =0.0;
	PID_Z -> out_max = +50.0f;
	PID_Z -> out_min = -50.0f;
	PID_Z -> setpoint =0.0;


	PID_nav -> kp =0.045f;//0.045f;
	PID_nav -> kd =0.06f;//0.06;
	PID_nav -> ki =0.0001f;
	PID_nav -> out_max = +22.0f;
	PID_nav -> out_min = -22.0f;

}

void PID_control_parameter_update(attitude_stablizer_pid_t* PID_roll,attitude_stablizer_pid_t* PID_pitch,attitude_stablizer_pid_t* PID_yaw_rate,attitude_stablizer_pid_t* PID_heading,vertical_pid_t* PID_Z,vertical_pid_t* PID_Zd,nav_pid_t* PID_nav){



	/* Roll axis PID parameter */
	if(check_global_data_update_flag(ROLL_KP) == true){
		read_global_data_value(ROLL_KP, DATA_POINTER_CAST(&(PID_roll -> kp)));
	
		reset_global_data_update_flag(ROLL_KP);

	}

	if(check_global_data_update_flag(ROLL_KI) == true){
		read_global_data_value(ROLL_KI, DATA_POINTER_CAST(&(PID_roll -> ki)));
	
		reset_global_data_update_flag(ROLL_KI);

	}

	if(check_global_data_update_flag(ROLL_KD) == true){
		read_global_data_value(ROLL_KD, DATA_POINTER_CAST(&(PID_roll -> kd)));
	
		reset_global_data_update_flag(ROLL_KD);

	}


	/* Pitch axis PID parameter */
	if(check_global_data_update_flag(PITCH_KP) == true){
		read_global_data_value(PITCH_KP, DATA_POINTER_CAST(&(PID_pitch -> kp)));
	
		reset_global_data_update_flag(PITCH_KP);

	}

	if(check_global_data_update_flag(PITCH_KI) == true){
		read_global_data_value(PITCH_KI, DATA_POINTER_CAST(&(PID_pitch -> ki)));
	
		reset_global_data_update_flag(PITCH_KI);

	}

	if(check_global_data_update_flag(PITCH_KD) == true){
		read_global_data_value(PITCH_KD, DATA_POINTER_CAST(&(PID_pitch -> kd)));
	
		reset_global_data_update_flag(PITCH_KD);

	}

	/* Yaw rate PID parameter */
	if(check_global_data_update_flag(YAW_KP) == true){
		read_global_data_value(YAW_KP, DATA_POINTER_CAST(&(PID_yaw_rate -> kp)));
	
		reset_global_data_update_flag(YAW_KP);

	}

	if(check_global_data_update_flag(YAW_KI) == true){
		read_global_data_value(YAW_KI, DATA_POINTER_CAST(&(PID_yaw_rate -> ki)));
	
		reset_global_data_update_flag(YAW_KI);

	}

	if(check_global_data_update_flag(YAW_KD) == true){
		read_global_data_value(YAW_KD, DATA_POINTER_CAST(&(PID_yaw_rate -> kd)));
	
		reset_global_data_update_flag(YAW_KD);

	}

	/* Heading PID parameter */
	if(check_global_data_update_flag(HEADING_KP) == true){
		read_global_data_value(HEADING_KP, DATA_POINTER_CAST(&(PID_heading -> kp)));
	
		reset_global_data_update_flag(HEADING_KP);

	}

	if(check_global_data_update_flag(HEADING_KI) == true){
		read_global_data_value(HEADING_KI, DATA_POINTER_CAST(&(PID_heading -> ki)));
	
		reset_global_data_update_flag(HEADING_KI);

	}

	if(check_global_data_update_flag(HEADING_KD) == true){
		read_global_data_value(HEADING_KD, DATA_POINTER_CAST(&(PID_heading -> kd)));
	
		reset_global_data_update_flag(HEADING_KD);

	}

	/* Z PID parameter */
	if(check_global_data_update_flag(Z_KP) == true){
		read_global_data_value(Z_KP, DATA_POINTER_CAST(&(PID_Z -> kp)));
	
		reset_global_data_update_flag(Z_KP);

	}

	if(check_global_data_update_flag(Z_KI) == true){
		read_global_data_value(Z_KI, DATA_POINTER_CAST(&(PID_Z -> ki)));
	
		reset_global_data_update_flag(Z_KI);

	}

	if(check_global_data_update_flag(Z_KD) == true){
		read_global_data_value(Z_KD, DATA_POINTER_CAST(&(PID_Z -> kd)));
	
		reset_global_data_update_flag(Z_KD);

	}


	/* Zd PID parameter */
	if(check_global_data_update_flag(ZD_KP) == true){
		read_global_data_value(ZD_KP, DATA_POINTER_CAST(&(PID_Zd -> kp)));
	
		reset_global_data_update_flag(ZD_KP);

	}

	if(check_global_data_update_flag(ZD_KI) == true){
		read_global_data_value(ZD_KI, DATA_POINTER_CAST(&(PID_Zd -> ki)));
	
		reset_global_data_update_flag(ZD_KI);

	}

	if(check_global_data_update_flag(ZD_KD) == true){
		read_global_data_value(ZD_KD, DATA_POINTER_CAST(&(PID_Zd -> kd)));
	
		reset_global_data_update_flag(ZD_KD);

	}

	/* Navigation PID parameter */
	if(check_global_data_update_flag(NAV_KP) == true){
		read_global_data_value(NAV_KP, DATA_POINTER_CAST(&(PID_nav -> kp)));
	
		reset_global_data_update_flag(NAV_KP);

	}

	if(check_global_data_update_flag(NAV_KI) == true){
		read_global_data_value(NAV_KI, DATA_POINTER_CAST(&(PID_nav -> ki)));
	
		reset_global_data_update_flag(NAV_KI);

	}

	if(check_global_data_update_flag(NAV_KD) == true){
		read_global_data_value(NAV_KD, DATA_POINTER_CAST(&(PID_nav -> kd)));
	
		reset_global_data_update_flag(NAV_KD);

	}
}

