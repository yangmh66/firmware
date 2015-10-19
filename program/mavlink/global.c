#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "AT24C04C.h"
#include "global.h"
#include "attitude_stabilizer.h"
#include "checksum.h"

#define QUADCOPTER 0

void eeprom_debug_print(void);

bool eeprom_is_written;

/* EEPROM Should not be written or read while flying, trigger this
 * flag so next time when the safety button is on (Disarm), save 
 * the data back into the EEPROM */
bool eeprom_pending_flag;

#define QUADCOPTER 0
int modifiable_data_cnt = 0;
global_data_t global_mav_data_list[GLOBAL_DATA_CNT] = {
	/* global data information */
	[VEHICLE_TYPE] = {.name = "vehicle_type", .type = UINT8,
		.data.uint8_value = QUADCOPTER},

	/* IMU information */
	[TRUE_ROLL] = {.name = "imu.roll"},
	[TRUE_PITCH] = {.name = "imu.pitch"},
	[TRUE_YAW] = {.name = "imu.yaw"},

	/* GPS Location */
	[GPS_LAT] = {.name = "gps.latitude", .type = INT32},
	[GPS_LON] = {.name = "gps.longitude", .type = INT32},
	[GPS_ALT] = {.name = "gps.altitude", .type = INT32},

	/* GPS Speed */
	[GPS_VX] = {.name = "gps.vx", .type = INT16},
	[GPS_VY] = {.name = "gps.vy", .type = INT16},
	[GPS_VZ] = {.name = "gps.vz", .type = INT16},

	/* Remote controller */
	[SAFTY_BUTTON] = {.name = "safty-button", .type = UINT8},
	[MODE_BUTTON] = {.name = "flight-mode-button", .type = UINT8},

	/* Attitude PID gain */
	[ROLL_KP] = {.name = "roll.kp", .type = FLOAT, .parameter_config = true},
	[ROLL_KI] = {.name = "roll.ki", .type = FLOAT, .parameter_config = true},
	[ROLL_KD] = {.name = "roll.kd", .type = FLOAT, .parameter_config = true},
	[PITCH_KP] = {.name = "pitch.kp", .type = FLOAT, .parameter_config = true},
	[PITCH_KI] = {.name = "pitch.ki", .type = FLOAT, .parameter_config = true},
	[PITCH_KD] = {.name = "pitch.kd", .type = FLOAT, .parameter_config = true},
	[YAW_KP] = {.name = "yaw.kp", .type = FLOAT, .parameter_config = true},
	[YAW_KI] = {.name = "yaw.ki", .type = FLOAT, .parameter_config = true},
	[YAW_KD] = {.name = "yaw.kd", .type = FLOAT, .parameter_config = true},
	[HEADING_KP] = {.name = "heading.kp", .type = FLOAT, .parameter_config = true},
	[HEADING_KI] = {.name = "heading.ki", .type = FLOAT, .parameter_config = true},
	[HEADING_KD] = {.name = "heading.kd", .type = FLOAT, .parameter_config = true},

	/* Height PID gain */
	[Z_KP] = {.name = "z.kp", .type = FLOAT, .parameter_config = true},
	[Z_KI] = {.name = "z.ki", .type = FLOAT, .parameter_config = true},
	[Z_KD] = {.name = "z.kd", .type = FLOAT, .parameter_config = true},
	[ZD_KP] = {.name = "zd.kp", .type = FLOAT, .parameter_config = true},
	[ZD_KI] = {.name = "zd.ki", .type = FLOAT, .parameter_config = true},
	[ZD_KD] = {.name = "zd.kd", .type = FLOAT, .parameter_config = true},

	/* Navigation PID gain */
	[NAV_KP] = {.name = "navigation.kp", .type = FLOAT, .parameter_config = true},
	[NAV_KI] = {.name = "navigation.ki", .type = FLOAT, .parameter_config = true},
	[NAV_KD] = {.name = "navigation.kd", .type = FLOAT, .parameter_config = true},

	/* Sensor calibration */
	[ACCEL_X_MAX] = {.name = "accel.max-x", .type = FLOAT, .parameter_config = true,
		.data.float_value = 4096},
	[ACCEL_X_MIN] = {.name = "accel.min-x", .type = FLOAT, .parameter_config = true,
		.data.float_value = -4096},
	[ACCEL_Y_MAX] = {.name = "accel.max-y", .type = FLOAT, .parameter_config = true,
		.data.float_value = 4096},
	[ACCEL_Y_MIN] = {.name = "accel.min-y", .type = FLOAT, .parameter_config = true,
		.data.float_value = -4096},
	[ACCEL_Z_MAX] = {.name = "accel.max-z", .type = FLOAT, .parameter_config = true,
		.data.float_value = 4096},
	[ACCEL_Z_MIN] = {.name = "accel.min-z", .type = FLOAT, .parameter_config = true,
		.data.float_value = -4096},
	[MAG_X_MAX] = {.name = "mag.max-x", .type = FLOAT, .parameter_config = true,
		.data.float_value = 4096},
	[MAG_X_MIN] = {.name = "mag.min-x", .type = FLOAT, .parameter_config = true,
		.data.float_value = -4096},
	[MAG_Y_MAX] = {.name = "mag.max-y", .type = FLOAT, .parameter_config = true,
		.data.float_value = 4096},
	[MAG_Y_MIN] = {.name = "mag.min-y", .type = FLOAT, .parameter_config = true,
		.data.float_value = -4096},
	[MAG_Z_MAX] = {.name = "mag.max-z", .type = FLOAT, .parameter_config = true,
		.data.float_value = 4096},
	[MAG_Z_MIN] = {.name = "mag.min-z", .type = FLOAT, .parameter_config = true,
		.data.float_value = -4096}
};

void init_global_data(void)
{
	/* Calculate the data count on the ground station parameter
	   configuration panel */
	int i;
	for(i = 0; i < get_global_data_count(); i++) {
		bool parameter_config;

		get_global_data_parameter_config_status(i, &parameter_config);
		if(parameter_config == true)
			modifiable_data_cnt++;
	}
} 

void init_global_data_with_eeprom(void)
{
	load_global_data_from_eeprom();

	if(eeprom_is_written == false)
		eeprom.clear();

	eeprom_debug_print();
}

/**
  * @brief  get the count of global data
  * @param  None
  * @retval global data count (int)
  */
int get_global_data_count(void)
{
	return GLOBAL_DATA_CNT;
}

/**
  * @brief  get the count of modifiable global data
  * @param  None
  * @retval modifiable global data count (int)
  */
int get_modifiable_data_count(void)
{
	return modifiable_data_cnt;
}

/**
  * @brief  set global data's value
  * @param  index (int), Type type, value (Data)
  * @retval Operated result (0 - succeeded, 1 - error)
  */
int set_global_data_value(int index, Type type, Data value)
{
	/* Index is in the range or not */
	if((index < 0) || (index >= GLOBAL_DATA_CNT))
		return GLOBAL_ERROR_INDEX_OUT_RANGE;

	/* Set the variable type and value */
	global_mav_data_list[index].type = type;

	switch(type) {
	    case UINT8:
		global_mav_data_list[index].data.uint8_value = 
			value.uint8_value;
		break;
	    case INT8:
		global_mav_data_list[index].data.int8_value = 
			value.int8_value;
		break;
	    case UINT16:
		global_mav_data_list[index].data.uint16_value = 
			value.uint16_value;
		break;
	    case INT16:
		global_mav_data_list[index].data.int16_value = 
			value.int16_value;
		break;
	    case UINT32:
		global_mav_data_list[index].data.uint32_value = 
			value.uint32_value;
		break;
	    case INT32:
		global_mav_data_list[index].data.int32_value = 
			value.int32_value;
		break;
	    case FLOAT:
		global_mav_data_list[index].data.float_value = 
			value.float_value;
		break;
	}

	global_mav_data_list[index].updated_flag = true;

	return GLOBAL_SUCCESS;
}

/**
  * @brief  get the variable type of global data
  * @param  index (int), variable type (Type* to get the type)
  * @retval Operated result (0 - succeeded, 1 - error)
  */
int get_global_data_type(int index, Type *type)
{
	/* Index is in the range or not */
	if((index < 0) || (index >= GLOBAL_DATA_CNT))
		return GLOBAL_ERROR_INDEX_OUT_RANGE;

	*type = global_mav_data_list[index].type;

	return GLOBAL_SUCCESS;
}

/**
  * @brief  get the access right of global data
  * @param  index (int), access right (AccessRight* to get the type)
  * @retval Operated result (0 - succeeded, 1 - error)
  */
int get_global_data_parameter_config_status(int index, bool *parameter_config)
{
	/* Index is in the range or not */
	if((index < 0) || (index >= GLOBAL_DATA_CNT))
		return GLOBAL_ERROR_INDEX_OUT_RANGE;

	*parameter_config = global_mav_data_list[index].parameter_config;

	return GLOBAL_SUCCESS;
}

/**
  * @brief  get the name of global data
  * @param  index (int), name (char* to get the name)
  * @retval Operated result (0 - succeeded, 1 - error)
  */
int read_global_data_name(int index, char **name)
{
        /* Index is in the range or not */
	if((index < 0) || (index >= GLOBAL_DATA_CNT))
		return GLOBAL_ERROR_INDEX_OUT_RANGE;
	
	*name = global_mav_data_list[index].name;

	return GLOBAL_SUCCESS;
}

/**
  * @brief  get the value of global data
  * @param  index (int), value (Data* to get the result value)
  * @retval Operated result (0 - succeeded, 1 - error)
  */
int read_global_data_value(int index, Data *value)
{
        /* Index is in the range or not */
	if((index < 0) || (index >= GLOBAL_DATA_CNT))
		return GLOBAL_ERROR_INDEX_OUT_RANGE;

	switch(global_mav_data_list[index].type) {
	    case UINT8:
		value->uint8_value = 
			global_mav_data_list[index].data.uint8_value;
		break;
	    case INT8:
		value->int8_value =
			global_mav_data_list[index].data.int8_value;
		break;
	    case UINT16:
		value->uint16_value =
			global_mav_data_list[index].data.uint16_value;
		break;
	    case INT16:
		value->int16_value = 
			global_mav_data_list[index].data.int16_value;
		break;
	    case UINT32:
		value->uint32_value =
			global_mav_data_list[index].data.uint32_value;
		break;
	    case INT32:
		value->int32_value =
			global_mav_data_list[index].data.int32_value;
		break;
	    case FLOAT:
		value->float_value =
			global_mav_data_list[index].data.float_value;
		break;
	}

	return GLOBAL_SUCCESS;
}

int set_global_data_eeprom_address(int index, uint16_t eeprom_address)
{
        /* Index is in the range or not */
	if((index < 0) || (index >= GLOBAL_DATA_CNT))
		return GLOBAL_ERROR_INDEX_OUT_RANGE;

	/* Address is valid or not */
	if(eeprom_address > 1024)
		return GLOBAL_EEPROM_INVALID_ADDRESS;

	global_mav_data_list[index].eeprom_address = eeprom_address;

	return GLOBAL_SUCCESS;
}

int get_global_data_eeprom_address(int index, uint16_t *eeprom_address)
{
        /* Index is in the range or not */
	if((index < 0) || (index >= GLOBAL_DATA_CNT))
		return GLOBAL_ERROR_INDEX_OUT_RANGE;

	*eeprom_address = global_mav_data_list[index].eeprom_address;

	return GLOBAL_SUCCESS;
}

int save_global_data_into_eeprom(int index, int *eeprom_return_status)
{
        /* Index is in the range or not */
	if((index < 0) || (index >= GLOBAL_DATA_CNT))
		return GLOBAL_ERROR_INDEX_OUT_RANGE;

	/* No need to save! */
	if(global_mav_data_list[index].parameter_config == false) {
		return GLOBAL_SUCCESS;
	}

	uint8_t *buffer;
	uint8_t data_len;

	/* Identify the data type */
	switch(global_mav_data_list[index].type) {
	    case UINT8:
		buffer = (uint8_t *)&global_mav_data_list[index].data.uint8_value;
		data_len = sizeof(uint8_t);
		break;
	    case INT8:
		buffer = (uint8_t *)&global_mav_data_list[index].data.int8_value;
		data_len = sizeof(int8_t);
		break;
	    case UINT16:
		buffer = (uint8_t *)&global_mav_data_list[index].data.uint16_value;
		data_len = sizeof(uint16_t);
		break;
	    case INT16:
		buffer = (uint8_t *)&global_mav_data_list[index].data.int16_value;
		data_len = sizeof(uint16_t);
		break;
	    case UINT32:
		buffer = (uint8_t *)&global_mav_data_list[index].data.uint32_value;
		data_len = sizeof(uint32_t);
		break;
	    case INT32:
		buffer = (uint8_t *)&global_mav_data_list[index].data.int32_value;
		data_len = sizeof(int32_t);
		break;
	    case FLOAT:
		buffer = (uint8_t *)&global_mav_data_list[index].data.float_value;
		data_len = sizeof(float);
		break;
	}

	//Get the eeprom address
	uint16_t eeprom_address = global_mav_data_list[index].eeprom_address;

	//Generate checksum data
	uint8_t checksum = checksum_generate(buffer, data_len);

	/* Write the data (payload & checksum) */
	eeprom.write(buffer, eeprom_address, data_len); //Payload bytes
	eeprom.write(&checksum, eeprom_address + data_len, 1); //Checksum byte

	/* Read the written data (payload & checksum) */
	Data data_eeprom;
	uint8_t buffer_verify[5]; //FIXME:Hard code
	uint8_t checksum_verify;
	eeprom.read(buffer_verify, eeprom_address, data_len + 1); //Plus lenght by 1 for checksum
	memcpy(&checksum_verify, buffer_verify + data_len, 1); //Checksum part

	bool data_is_correct = true;

	/* Payload check */
	if(memcmp(buffer, buffer_verify, data_len) != 0) {
		data_is_correct = false;
	}

	/* Checksum test */
	if(checksum_verify != checksum) {
		data_is_correct = false;
	}

	/* Data check result (payload & checksum) */
	if(data_is_correct == false) {
		*eeprom_return_status = EEPROM_DATA_CHECK_FAILED;

		EEPROM_DEBUG_PRINT("[address: %d]Data check failure\n\r", eeprom_address);
	} else {
		*eeprom_return_status = EEPROM_SUCCESS;

		//XXX: float only ...
		EEPROM_DEBUG_PRINT("[Address: %d - Value: %f]", eeprom_address, (double)data_eeprom.float_value);
		EEPROM_DEBUG_PRINT("%d %d %d %d (%d)", buffer_verify[0], buffer_verify[1], buffer_verify[2], buffer_verify[3], checksum_verify);
	}

	/* Set up the first byte of eeprom (data = global data count) */
	if(eeprom_is_written == false) {
		eeprom_is_written = true;
	}

	return GLOBAL_SUCCESS;
}

void load_global_data_from_eeprom(void)
{
	/* Load the data from eeprom */
	uint8_t eeprom_data[5] = {0};
	eeprom.read(eeprom_data, 0, 1);

	/* If first byte's value of EEPROM is equal to the global data count, it means 
	   the EEPROM has been written */	
	eeprom_is_written = (eeprom_data[0] == get_global_data_count() ? true : false);

	bool parameter_config;
	/* Start from second byte, 
	 * First byte: check the eeprom has been use or not */
	uint16_t eeprom_address = 1;
	uint8_t checksum;
	Type type;
	uint8_t type_size;
	Data data;

	int i;
	for(i = 0; i < get_global_data_count(); i++) {
		get_global_data_parameter_config_status(i, &parameter_config);

		if(parameter_config == true) {
			//Set the eeprom address into the global data
			set_global_data_eeprom_address(i, eeprom_address);

			get_global_data_type(i, &type);

			/* Get the size of the current global data's data type */
			switch(type) {
			    case UINT8:
				type_size = sizeof(uint8_t);
				break;
			    case INT8:
				type_size = sizeof(int8_t);
				break;
			    case UINT16:
				type_size = sizeof(uint16_t);
				break;
			    case INT16:
				type_size = sizeof(int16_t);
				break;
			    case UINT32:
				type_size = sizeof(uint32_t);
				break;
			    case INT32:
				type_size = sizeof(int32_t);
				break;
			    case FLOAT:
				type_size = sizeof(float);
				break;
			}

			if(eeprom_is_written == true) {
				/* Read the data from the eeprom */
				eeprom.read(eeprom_data, eeprom_address, type_size + 1);
				memcpy(&data, eeprom_data, type_size);
				memcpy(&checksum, eeprom_data + type_size, 1);
	
				if(checksum_test(eeprom_data, type_size, checksum) == 0) {
					set_global_data_value(i, type, DATA_CAST(data));
				} else {
					EEPROM_DEBUG_PRINT("EEPROM checksum test is failed!\n"); //TODO:Data is not correct, handle this situation!
				}
			}

			//One more byte for checksum
			eeprom_address += type_size + 1;
		}
	}
}

void set_eeprom_pending_flag(void)
{
	eeprom_pending_flag = true;
}

void clear_eeprom_pending_flag(void)
{
	eeprom_pending_flag = false;
}

bool is_eeprom_pending_to_save(void)
{
	return (eeprom_pending_flag == true) ? true : false;
}

void eeprom_debug_print(void)
{
	bool parameter_config;
	uint16_t eeprom_address;

	Data data;
	uint8_t eeprom_data[5] = {0};
	uint8_t checksum = 0;

	int i, j;
	for(i = 0; i < get_global_data_count(); i++) {
		get_global_data_parameter_config_status(i, &parameter_config);

		if(parameter_config == true) {
			get_global_data_eeprom_address(i , &eeprom_address);

			eeprom.read(eeprom_data, eeprom_address, sizeof(float) + 1);
			memcpy(&data, eeprom_data, sizeof(float));
			memcpy(&checksum, eeprom_data + sizeof(float), 1);
			
			EEPROM_DEBUG_PRINT("[address : %d] ", eeprom_address);

			for(j = 0; j < 5; j++) {
				if(j != 4) 
					EEPROM_DEBUG_PRINT("%d ", eeprom_data[j]);
				else
					EEPROM_DEBUG_PRINT("(%d) ", eeprom_data[j]);
			}

			EEPROM_DEBUG_PRINT("\n\r");
			//EEPROM_DEBUG_PRINT("-> value : %f (%d)\n\r", data.float_value, eeprom_data[4]);
		}
	}

	EEPROM_DEBUG_PRINT("\n\r");
}

/**
  * @brief  Set updated_flag
  * @param  index (int),
  * @retval None
  */
void set_global_data_update_flag(int index){
	global_mav_data_list[index].updated_flag = true;
}


/**
  * @brief  Unset updated_flag
  * @param  index (int),
  * @retval None
  */
void reset_global_data_update_flag(int index){
	global_mav_data_list[index].updated_flag = false;
}

/**
  * @brief  Check global data updated_flag
  * @param  index (int),
  * @retval Updated_flag status
  */
bool check_global_data_update_flag(int index){
	return global_mav_data_list[index].updated_flag;
}
