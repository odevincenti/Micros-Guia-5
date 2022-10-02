/***************************************************************************//**
  @file     orientation.c
  @brief    Orientation retrieval and calculation
  @author   Olivia De Vincenti
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdio.h>
#include "timer.h"
#include "i2cm.h"
#include "board.h"
#include "gpio.h"
#include "FXOS8700CQ.h"
#include "math.h"
#include "orientation.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define ORIENTATION_DEVELOPMENT_MODE
#define MAGNET

#define BYTE_SIZE	8
#define ACC_SHIFT	2

#define I2C_ID				I2C0_ID
#define FXOS8700CQ_ADD		0x1D
#define FXOS8700CQ_ID		0xC7
#define TIMER_MS			16

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
	uint8_t reg_add;
	uint8_t reg_data;
} orientation_reg_t;

typedef struct {
	orientation_reg_t	who_am_i;
	orientation_reg_t	C1;
	orientation_reg_t 	M_C1;
	orientation_reg_t 	M_C2;
	orientation_reg_t 	xyz_cfg;
	orientation_reg_t	M_C3;
	orientation_reg_t	clear;
} orientation_config_t;

// typedef struct {
// 	// Accelerometer axis
// 	int16_t	x_acc_axis;
// 	int16_t	y_acc_axis;
// 	int16_t	z_acc_axis;
// 	// Magnetometer axis
// 	int16_t	x_mag_axis;
// 	int16_t	y_mag_axis;
// 	int16_t	z_mag_axis;
// } axis_t;

typedef struct {
// Accelerometer axis
	int16_t	x_acc_axis;
	int16_t	y_acc_axis;
	int16_t	z_acc_axis;
// Magnetometer axis
	int16_t	x_mag_axis;
	int16_t	y_mag_axis;
	int16_t	z_mag_axis;
} axis_t;

typedef struct {
	uint8_t status;
	uint8_t	x_acc_MSB;
	uint8_t	x_acc_LSB;
	uint8_t	y_acc_MSB;
	uint8_t	y_acc_LSB;
	uint8_t	z_acc_MSB;
	uint8_t	z_acc_LSB;
	uint8_t	x_mag_MSB;
	uint8_t	x_mag_LSB;
	uint8_t	y_mag_MSB;
	uint8_t	y_mag_LSB;
	uint8_t	z_mag_MSB;
	uint8_t	z_mad_LSB;
} raw_axis_t;

typedef struct {
	int16_t	roll;
	int16_t	pitch;
	int16_t	yaw;
} orientation_t;

typedef struct {
	bool	roll_state;
	bool	pitch_state;
	bool	yaw_state;
} orientation_state_t;

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
void get_accel_data();
void get_accel_magnet_data();
void update_orientation();
void orientation_ISR();
void read_reg(uint8_t* reg_add, uint8_t* read_data, uint8_t bytes_to_read);
void write_reg(uint8_t* reg_add, uint8_t* write_data, uint8_t bytes_to_write);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static bool orientation_init = false;
static tim_id_t orientation_timer;

raw_axis_t raw_axis_data;
axis_t axis_data;
orientation_t angle_data;
orientation_state_t angle_state;
uint8_t axis_reg = FXOS8700CQ_STATUS;
// uint8_t magnet_reg = FXOS8700CQ_OUT_X_MSB;
orientation_config_t orientation_config;
uint8_t acc_offset_reg = FXOS8700CQ_OFF_X;
uint8_t mag_offset_reg = FXOS8700CQ_OFF_X;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

bool orientation_Init(){

	bool r = false;

	if (!orientation_init){

		I2C_Init(I2C_ID);

		timerInit();

		orientation_timer = timerGetId();
		timerCreate(orientation_timer, TIMER_MS2TICKS(TIMER_MS), TIM_MODE_PERIODIC, &orientation_ISR);

		r = true;
		//orientation_init = true;
	}

	return r;
}

bool orientation_Config(){

	if (!orientation_init){

		orientation_config.who_am_i.reg_add = FXOS8700CQ_WHO_AM_I;
		orientation_config.who_am_i.reg_data = 0x0;
		read_reg(&orientation_config.who_am_i.reg_add, &orientation_config.who_am_i.reg_data, 1);
		//if (orientation_config.who_am_i.reg_data != FXOS8700CQ_ID) { return false; }
		
		orientation_config.clear.reg_data = 0x0;
		orientation_config.C1.reg_add = FXOS8700CQ_CTRL_REG1;
		write_reg(&orientation_config.C1.reg_add, &orientation_config.clear.reg_data, 1);
		
		orientation_config.M_C1.reg_data = 0x1F;
		orientation_config.M_C1.reg_add = FXOS8700CQ_M_CTRL_REG1;
		write_reg(&orientation_config.M_C1.reg_add, &orientation_config.M_C1.reg_data, 1);

		orientation_config.M_C2.reg_data = 0x20;
		orientation_config.M_C2.reg_add = FXOS8700CQ_M_CTRL_REG2;
		write_reg(&orientation_config.M_C2.reg_add, &orientation_config.M_C2.reg_data, 1);

		orientation_config.xyz_cfg.reg_data = 0x01;
		orientation_config.xyz_cfg.reg_add = FXOS8700CQ_XYZ_DATA_CFG;
		write_reg(&orientation_config.xyz_cfg.reg_add, &orientation_config.xyz_cfg.reg_data, 1);

		orientation_config.C1.reg_data = 0x0D;
		write_reg(&orientation_config.C1.reg_add, &orientation_config.C1.reg_data, 1);

		timerActivate(orientation_timer);
		orientation_init = true;

		return true;

	} else {
		return false;
	}
}

void orientation_Start(){
	if (!orientation_init){
		timerActivate(orientation_timer);
		get_accel_data();
		orientation_init = true;
	}
}

bool isOrientationReady(){
	if (I2C_IsBusFree(I2C_ID)){
		return true;
	} else {
		return false;
	}
}

void calibrateOrientation(offset_t* orientation_offset){

	// Set standby mode
	orientation_config.C1.reg_add = FXOS8700CQ_CTRL_REG1;
	orientation_config.clear.reg_data = 0x0;
	write_reg(&orientation_config.C1.reg_add, &orientation_config.clear.reg_data, 1);

#ifdef MAGNET
	// Save M_C3 config
	orientation_config.M_C3.reg_add = FXOS8700CQ_M_CTRL_REG3;
	orientation_config.M_C3.reg_data = 0x0;
	read_reg(&orientation_config.M_C3.reg_add, &orientation_config.M_C3.reg_data, 1);

	// Clear M_C3
	write_reg(&orientation_config.M_C3.reg_add, &orientation_config.clear.reg_data, 1);
#endif

	// Set accelerometer offset
	write_reg(&acc_offset_reg, (uint8_t*) orientation_offset, AXIS_N);

#ifdef MAGNET
	// Set magnetometer offset
	write_reg(&mag_offset_reg, ((uint8_t*) orientation_offset) + 6, 6);

	// Restore M_C3 config
	write_reg(&orientation_config.M_C3.reg_add, &orientation_config.M_C3.reg_data, 1);
#endif

	// Set active mode
	orientation_config.C1.reg_data = 0x0D;
	write_reg(&orientation_config.C1.reg_add, &orientation_config.C1.reg_data, 1);

}

uint16_t getRoll(){
	return angle_data.roll;
}

uint16_t getPitch(){
	return angle_data.pitch;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void orientation_ISR(){
	
	//static int h = 0;

		// Retrieve and refactor data
		refactor_data();

		// uint16_t roll = atan2(axis_data.y_acc_axis, axis_data.z_acc_axis);
		// if (roll - angle_data.roll > ANGLE_THRESHOLD){
		// 	angle_data.roll = roll;
		// 	angle_state.roll_state = true;
		// }

		// uint16_t pitch = atan2(-axis_data.x_acc_axis, axis_data.y_acc_axis*sin(angle_data.roll) + axis_data.z_acc_axis*cos(angle_data.roll));
		// if (pitch - angle_data.pitch > ANGLE_THRESHOLD){
		// 	angle_data.pitch = pitch;
		// 	angle_state.pitch_state = true;
		// }

	//if (h%2){

#ifdef MAGNET
		// Update data
		get_accel_magnet_data();
#else
		get_accel_data();
#endif
/*	} else {

		orientation_config.who_am_i.reg_add = FXOS8700CQ_WHO_AM_I;
		orientation_config.who_am_i.reg_data = 0x0;
		read_reg(&orientation_config.who_am_i.reg_add, &orientation_config.who_am_i.reg_data, 1);

	}

		h++;*/

}

void refactor_data(){
	
	// Retrieve and refactor data
	uint8_t i = 0;
	uint8_t* raw_ptr = &raw_axis_data.x_acc_MSB;
	int16_t* axis_ptr = &axis_data.x_acc_axis;

	for (i = 0; i < AXIS_N; i++, raw_ptr += 2, axis_ptr++){
		if ((int8_t) *raw_ptr < 0){
			*axis_ptr = (((*raw_ptr << BYTE_SIZE) | *(raw_ptr + 1)) >> ACC_SHIFT) | 0xC000;
		} else {
			*axis_ptr = ((*raw_ptr << BYTE_SIZE) | *(raw_ptr + 1)) >> ACC_SHIFT;
		}
	}
	
#ifdef MAGNET
	//for (i = 0; i < AXIS_N; raw_ptr += 2, axis_ptr++){
	//	*axis_ptr = (*raw_ptr << BYTE_SIZE) | *(raw_ptr + 1);
	//}
#endif

}

void get_accel_data(){
	i2c_transaction_t trans_w_rsta = { .mode = I2C_WRITE_MODE, .address = FXOS8700CQ_ADD, .ptr = &axis_reg, .count = 1, .next_rsta = true};
	I2C_NewTransaction(I2C0_ID, &trans_w_rsta);

	i2c_transaction_t trans_r_rsta = { .mode = I2C_READ_MODE, .address = FXOS8700CQ_ADD, .ptr = (uint8_t*)(&raw_axis_data), .count = 7, .next_rsta = false};
	I2C_NewTransaction(I2C0_ID, &trans_r_rsta);
}

void get_accel_magnet_data(){
	i2c_transaction_t trans_w_rsta = { .mode = I2C_WRITE_MODE, .address = FXOS8700CQ_ADD, .ptr = &axis_reg, .count = 1, .next_rsta = true};
	I2C_NewTransaction(I2C0_ID, &trans_w_rsta);

	i2c_transaction_t trans_r_rsta = { .mode = I2C_READ_MODE, .address = FXOS8700CQ_ADD, .ptr = (uint8_t*)(&raw_axis_data), .count = 13, .next_rsta = false};
	I2C_NewTransaction(I2C0_ID, &trans_r_rsta);
}

void read_reg(uint8_t* reg_add, uint8_t* read_data, uint8_t bytes_to_read){
	i2c_transaction_t trans_w_rsta = { .mode = I2C_WRITE_MODE, .address = FXOS8700CQ_ADD, .ptr = reg_add, .count = 1, .next_rsta = true};
	I2C_NewTransaction(I2C0_ID, &trans_w_rsta);

	i2c_transaction_t trans_r_rsta = { .mode = I2C_READ_MODE, .address = FXOS8700CQ_ADD, .ptr = read_data, .count = bytes_to_read, .next_rsta = false};
	I2C_NewTransaction(I2C0_ID, &trans_r_rsta);
}

void write_reg(uint8_t* reg_add, uint8_t* write_data, uint8_t bytes_to_write){
	i2c_transaction_t trans_w_rsta = { .mode = I2C_WRITE_MODE, .address = FXOS8700CQ_ADD, .ptr = reg_add, .count = 1, .next_rsta = true};
	I2C_NewTransaction(I2C0_ID, &trans_w_rsta);

	i2c_transaction_t trans_r_rsta = { .mode = I2C_WRITE_MODE, .address = FXOS8700CQ_ADD, .ptr = write_data, .count = bytes_to_write, .next_rsta = false};
	I2C_NewTransaction(I2C0_ID, &trans_r_rsta);
}

void print_axis_data(){
	printf("X: %d\n", axis_data.x_acc_axis);
	printf("Y: %d\n", axis_data.y_acc_axis);
	printf("Z: %d\n", axis_data.z_acc_axis);
}

/******************************************************************************/

