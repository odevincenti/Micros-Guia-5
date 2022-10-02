/***************************************************************************//**
  @file     orientation.h
  @brief    Orientation retrieval and calculation
  @author   Olivia De Vincenti
 ******************************************************************************/

#ifndef _ORIENTATION_H_
#define _ORIENTATION_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define ANGLE_THRESHOLD		5
#define AXIS_N				3
#define mG_TO_OFFSET(x)		((x) / 2)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
// Accelerometer axis
	int8_t	x_acc_axis;
	int8_t	y_acc_axis;
	int8_t	z_acc_axis;
// Magnetometer axis
	int8_t	x_mag_axis;
	int8_t	y_mag_axis;
	int8_t	z_mag_axis;
} offset_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
bool orientation_Init();

bool orientation_Config();

void calibrateOrientation(offset_t * orientation_offset);

void orientation_Start();

bool isOrientationReady();

uint16_t getRoll();

uint16_t getPitch();

/*******************************************************************************
 ******************************************************************************/

void get_accel_magnet_data();

void refactor_data();

void print_axis_data();

void print_regs();


#endif // _ORIENTATION_H_
