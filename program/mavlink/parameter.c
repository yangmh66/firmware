#include "global.h"
#include "communication.h"
#include "parameter.h"

extern mavlink_message_t received_msg;

mavlink_message_t msg;

void parameter_read_value(void)
{
	AccessRight data_access_right;
	Type data_type;
	int data_int;
	float data_float;
	char *data_name;

	int i, modifiable_data_cnt = 0;
	for(i = 0; i < get_global_data_count(); i++) {

		/* If the access right equal to READ_WRITE, send the data to the ground station */
		get_global_data_access_right(i, &data_access_right);
		if(data_access_right == READ_WRITE) {

			/* Prepare the data */
			get_global_data_type(i, &data_type);
			if(data_type == INTEGER)
				read_global_data_int(i, &data_int);
			else
				read_global_data_float(i, &data_float);

			/* Get the data name*/
			read_global_data_name(i, &data_name);

			/* Send out the data */
			mavlink_msg_param_value_pack(
				1, 0, &msg,
				data_name,   		              /* Data name */ 
				data_type ? data_float : data_int,    /* Data value */
				data_type ? MAV_PARAM_TYPE_REAL32 : MAV_PARAM_TYPE_INT16, /* Data type */
				(uint16_t)get_modifiable_data_count(), /* Data count */
				modifiable_data_cnt		       /* Index */
			);
			send_package(&msg);

			modifiable_data_cnt++;
		}
	}
}

void parameter_read_single_value(void)
{
	mavlink_param_request_read_t mprr;
	mavlink_msg_param_request_read_decode(&received_msg, &mprr);

	AccessRight data_access_right;
	Type data_type;
	int data_int;
	float data_float;
	char *data_name;

	int i;
	for(i = 0; i < get_global_data_count(); i++) {
		/* If the access right equal to READ_WRITE, send the data to the ground station */
		get_global_data_access_right(i, &data_access_right);
		if(data_access_right == READ_ONLY)
			continue;

		/* Get the data name */
		read_global_data_name(i, &data_name);

		if(strcmp(data_name, mprr.param_id) == 0) {
			/* Prepare the data */
			get_global_data_type(i, &data_type);
			if(data_type == INTEGER)
				read_global_data_int(i, &data_int);
			else
				read_global_data_float(i, &data_float);
			
			/* Send out the data */
			mavlink_msg_param_value_pack(
				1, 0, &msg,
				data_name,  		 	       /* Data name */ 
				data_type ? data_float : data_int,     /* Data value */
				data_type ? MAV_PARAM_TYPE_REAL32 : MAV_PARAM_TYPE_INT16, /* Data type */
				(uint16_t)get_modifiable_data_count(), /* Data count */
				mprr.param_index 		       /* Index */
			);
			send_package(&msg);

			break;
		}
	}
}

void parameter_write_value(void)
{
	mavlink_param_set_t mps;	

	Type data_type;
	int data_int;
	float data_float;
	char *data_name;

	int i;
	for(i = 0; i < get_global_data_count(); i++) {
		/* Get the data name */
		read_global_data_name(i, &data_name);

		/* Compare the global data with the parameter id */
		if(strcmp(data_name, mps.param_id) == 0) {
			get_global_data_type(i, &data_type);
			
			/* Update the new value */
			if(data_type == INTEGER) {
				set_global_data_int(i, mps.param_value);
				data_int = mps.param_value;
			} else {
				set_global_data_float(i, mps.param_value);
				data_float = mps.param_value;
			}

			/* Get the data name */
			read_global_data_name(i, &data_name);

			/* Ack message */
			mavlink_msg_param_value_pack(
				1, 0, &msg,
				data_name,   			       /* Data name */ 
				data_type ? data_float : data_int,     /* Data value */
				data_type ? MAV_PARAM_TYPE_REAL32 : MAV_PARAM_TYPE_INT16, /* Data type */
				get_modifiable_data_count(), 	       /* Data count */
				i		 		       /* Index */
			);
			send_package(&msg);

			break;
		}
	}
}