#if (AIRFRAME_SELECT == AIRFRAME_QUADROTOR)

#include "multirotor/controller.h"

void PID_rc_pass_command(attitude_t* attitude,attitude_stablizer_pid_t* PID_roll,attitude_stablizer_pid_t* PID_pitch,attitude_stablizer_pid_t* PID_heading,vertical_pid_t* PID_Z,vertical_pid_t* PID_Zd,nav_pid_t* PID_nav,radio_controller_t* rc_command){

	/* Attitude Controller command */
	PID_roll -> setpoint = (rc_command -> roll_control_input) + (PID_nav -> output_roll);
	PID_pitch -> setpoint = (rc_command -> pitch_control_input) + (PID_nav -> output_pitch);

		if( rc_command -> safety == ENGINE_ON) {
			/* If throttle is smaller than 10%, stop integrator */

			if( rc_command -> throttle_control_input < 10.0f){
				PID_roll -> integral = 0.0f;
				PID_pitch -> integral = 0.0f;
			}
		}else{
			
			/* Always stop integrator when safety is on */
			PID_roll -> integral = 0.0f;
			PID_pitch -> integral = 0.0f;

		}





	/* Heading Controller command */
	if(CONTROLLER_YAW_MODE  == YAW_MODE_MAGNETO){

		if( rc_command -> safety == ENGINE_ON) {

			PID_heading -> setpoint = (PID_heading -> setpoint)+ (rc_command -> yaw_rate_control_input)*CONTROL_DT;

			if((PID_heading -> setpoint ) > 360.0f){

				PID_heading -> setpoint = PID_heading -> setpoint - 360.0f;
			}else if((PID_heading -> setpoint ) <0.0f){

				PID_heading -> setpoint = PID_heading -> setpoint + 360.0f;

			}

		}else{

			PID_heading -> setpoint = attitude -> yaw;
		}

	}else if(CONTROLLER_YAW_MODE == YAW_MODE_GYRO){
		
			PID_heading -> setpoint = rc_command -> yaw_rate_control_input;

	}


	if((rc_command -> mode) == MODE_3){

		PID_Z -> controller_status = CONTROLLER_ENABLE ;
		PID_Zd -> controller_status = CONTROLLER_ENABLE ;
		PID_nav -> controller_status = CONTROLLER_ENABLE;

	}else if((rc_command -> mode) == MODE_2){

		PID_Z -> controller_status = CONTROLLER_ENABLE ;
		PID_Zd -> controller_status = CONTROLLER_ENABLE ;
		PID_nav -> controller_status = CONTROLLER_DISABLE;

	}else{ // MODE_1

		PID_Z -> controller_status = CONTROLLER_DISABLE ;
		PID_Zd -> controller_status = CONTROLLER_DISABLE ;
		PID_nav -> controller_status = CONTROLLER_DISABLE;

	}

}

void PID_output(radio_controller_t* rc_command,attitude_stablizer_pid_t* PID_roll,attitude_stablizer_pid_t* PID_pitch,attitude_stablizer_pid_t* PID_yaw_rate,vertical_pid_t* PID_Zd){

motor_output_t motor;

	motor. m1 =0.0;
	motor. m2 =0.0;
	motor. m3 =0.0;
	motor. m4 =0.0;
	motor. m5 =0.0;
	motor. m6 =0.0;
	motor. m7 =0.0;
	motor. m8 =0.0;
	motor. m9 =0.0;
	motor. m10 =0.0;
	motor. m11 =0.0;
	motor. m12 =0.0;
	if( rc_command -> safety == ENGINE_ON) {

	motor . m1 = -10.0f + (rc_command->throttle_control_input) - (PID_roll->output) + (PID_pitch -> output) - (PID_yaw_rate -> output) + (PID_Zd -> output);
	motor . m2 = -10.0f + (rc_command->throttle_control_input) + (PID_roll->output) + (PID_pitch -> output) + (PID_yaw_rate -> output) + (PID_Zd -> output);
	motor . m3 = -10.0f + (rc_command->throttle_control_input) + (PID_roll->output) - (PID_pitch -> output) - (PID_yaw_rate -> output) + (PID_Zd -> output);
	motor . m4 = -10.0f + (rc_command->throttle_control_input) - (PID_roll->output) - (PID_pitch -> output) + (PID_yaw_rate -> output) + (PID_Zd -> output);
	set_pwm_motor(&motor);

	LED_ON(LED3);

	}else{

	motor. m1 =0.0;
	motor. m2 =0.0;
	motor. m3 =0.0;
	motor. m4 =0.0;
	set_pwm_motor(&motor);

	LED_OFF(LED3);
	}
}

//__attribute__((unused))

#endif
