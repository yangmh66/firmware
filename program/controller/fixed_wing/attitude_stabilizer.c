#if (AIRFRAME_SELECT == AIRFRAME_CONVENTIONAL_FIXED_WING)

#include "fixed_wing/attitude_stabilizer.h"
#include "fixed_wing/controller.h"

#define LOOP_DT  (0.00025f)

void PID_attitude_roll(attitude_stablizer_pid_t* PID_control,imu_data_t* imu_filtered_data,attitude_t* attitude){

	if(PID_control -> controller_status == CONTROLLER_ENABLE) {

		(PID_control -> error) = (PID_control -> setpoint) - (attitude -> roll);

		float P = (PID_control -> error) * (PID_control -> kp);

		float D = -(imu_filtered_data -> gyro[0]) * (PID_control -> kd);


		PID_control -> integral += ((PID_control -> error) * (PID_control -> ki)) * LOOP_DT;

		PID_control -> integral = bound_float(PID_control -> integral,-10.0f,+10.0f);

		(PID_control -> output) = P+D+ (PID_control -> integral);

	}else{
	
		PID_control -> integral = 0.0f;
		PID_control -> output =0.0f;

	}
}


void PID_attitude_pitch(attitude_stablizer_pid_t* PID_control,imu_data_t* imu_filtered_data,attitude_t* attitude){

	if(PID_control -> controller_status == CONTROLLER_ENABLE) {

		(PID_control -> error) = (PID_control -> setpoint) - (attitude -> pitch);

		float P = (PID_control -> error) * (PID_control -> kp);

		float D = -(imu_filtered_data -> gyro[1]) * (PID_control -> kd);


		PID_control -> integral += ((PID_control -> error) * (PID_control -> ki)) * LOOP_DT;

		PID_control -> integral = bound_float(PID_control -> integral,-10.0f,+10.0f);


		(PID_control -> output) = P+D+ (PID_control -> integral);

	}else{

		PID_control -> integral = 0.0f;
		PID_control -> output =0.0f;

	}
}

void PID_attitude_yaw_rate(attitude_stablizer_pid_t* PID_control,imu_data_t* imu_filtered_data){

	if(PID_control -> controller_status == CONTROLLER_ENABLE) {

		// float (PID_control -> error) = (PID_control -> setpoint) - (attitude -> yaw);

		// float P = (PID_control -> error) * (PID_control -> kp);

		float P = -(PID_control -> setpoint - imu_filtered_data -> gyro[2]) * (PID_control -> kp);

		(PID_control -> output) = P;
 
		(PID_control -> output) = bound_float(PID_control -> output,PID_control -> out_min,PID_control -> out_max);

	}else{

		PID_control -> output =0.0f;

	}

}

void PID_attitude_heading(attitude_stablizer_pid_t* PID_control,attitude_t* attitude){

	if(PID_control -> controller_status == CONTROLLER_DISABLE) {

		/* Pass the value directly to yaw controller */
		(PID_control -> output) = (PID_control -> setpoint);

		return;
	}

	//(PID_control -> error) = (PID_control -> setpoint) - (attitude -> yaw);

	if(CONTROLLER_YAW_MODE  == YAW_MODE_MAGNETO){

		if((PID_control -> setpoint) > (attitude -> yaw)){
			if(((PID_control -> setpoint) - (attitude -> yaw))>=180.0f){
				(PID_control -> error) = (PID_control -> setpoint)-((attitude -> yaw)+360.0f);
			}else{
				(PID_control -> error) = (PID_control -> setpoint)-(attitude -> yaw);
			}
		}else{
			if(((attitude -> yaw) - (PID_control -> setpoint))>=180.0f){
				(PID_control -> error) = ((PID_control -> setpoint)+360.0f)-(attitude -> yaw);
			}else{
				(PID_control -> error) = (PID_control -> setpoint)-(attitude -> yaw);
			}

		}

		float P = (PID_control -> error) * (PID_control -> kp);

		(PID_control -> output) = P;

		(PID_control -> output) = bound_float(PID_control -> output,PID_control -> out_min,PID_control -> out_max);

	}else if(CONTROLLER_YAW_MODE == YAW_MODE_GYRO){

		/* Pass the value directly to yaw controller */
		(PID_control -> output) = (PID_control -> setpoint);

	}

}

#endif
