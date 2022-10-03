/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdio.h>
#include "orientation.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
static int i = 0;
static uint8_t reg_add = 0x0;
static offset_t off = { .acc_offset_reg = 0x0, .x_acc_axis = 0xAB, .y_acc_axis = 0x51, .z_acc_axis = 0x7F, .mag_offset_reg = 0x0, .x_mag_axis = 0x0, .y_mag_axis = 0x0, .z_mag_axis = 0x0};

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	orientation_Init();
	i = 0;
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	if(!(i)){

		orientation_Config();
		//calibrateOrientation(&off);

		i++;
	}

	if (isOrientationReady()){
		orientation_Start();
	}

//		isOrientationReady();
	/*if (orientation_Compute()){

		if (getRollState()){
			printf("Roll: %d\n", getRoll());
		}

		if (getPitchState()){
			printf("Pitch: %d\n", getPitch());
		}

		if (getYawState()){
			printf("Yaw: %d\n", getYaw());
		}
*/

	if(!(i%5000)){
		orientation_Compute();
		printf("Roll: %d\n", getRoll());
		printf("Pitch: %d\n", getPitch());
		printf("Yaw: %d\n", getYaw());

		//print_regs();
		//read_reg(&reg_add, &read[0], 60);
		// print_axis_data();
		// printf("Roll: %i\n", getRoll());
		// printf("Pitch: %i\n", getPitch());

	}

	i++;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/*******************************************************************************
 ******************************************************************************/
