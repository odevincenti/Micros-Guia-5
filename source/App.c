/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdio.h>
#include "board.h"
#include "gpio.h"
#include "i2cm.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define FXOS8700CQ_ADD			0x1D
#define FXOS8700CQ_WHO_AM_I		0x0D

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
typedef struct {
	uint16_t	x_axis;
	uint16_t	y_axis;
	uint16_t	z_axis;
} axis_t;

axis_t axis_data;
uint8_t reg_address = 0x0;

uint8_t who_am_i_reg = FXOS8700CQ_WHO_AM_I;
uint8_t who_am_i;
static int i;

static uint8_t write = 0x0;
static uint8_t read[3];
static uint8_t read_rsta[7];

void read_reg(uint8_t* reg_add, uint8_t* read_data, uint8_t bytes_to_read);

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	I2C_Init(I2C0_ID);
	i = 0;
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	if(!(i)){

		read_reg(&who_am_i_reg, &who_am_i, 1);

		//i2c_transaction_t trans_w = { .mode = I2C_WRITE_MODE, .address = 0x1D, .ptr = &write[0], .count = 4, .next_rsta = false};
		//I2C_NewTransaction(I2C0_ID, &trans_w);
		// start
		// 0011101 0 0

		//i2c_transaction_t trans_r = { .mode = I2C_READ_MODE, .address = 0x1D, .ptr = &read[0], .count = 3, .next_rsta = false};
		//I2C_NewTransaction(I2C0_ID, &trans_r);
		// start
		// 00111010 0

		// i2c_transaction_t trans_w_rsta = { .mode = I2C_WRITE_MODE, .address = 0x1D, .ptr = &write, .count = 1, .next_rsta = true};
		// I2C_NewTransaction(I2C0_ID, &trans_w_rsta);

		// i2c_transaction_t trans_r_rsta = { .mode = I2C_READ_MODE, .address = 0x1D, .ptr = &read_rsta[0], .count = 7, .next_rsta = false};
		// I2C_NewTransaction(I2C0_ID, &trans_r_rsta);

		/*i2c_transaction_t trans_w_rsta = { .mode = I2C_WRITE_MODE, .address = 0x1D, .ptr = &reg_address, .count = 1, .next_rsta = true};
		I2C_NewTransaction(I2C0_ID, &trans_w_rsta);

		i2c_transaction_t trans_r_rsta = { .mode = I2C_READ_MODE, .address = 0x1D, .ptr = (uint8_t*)(&axis_data), .count = 6, .next_rsta = false};
		I2C_NewTransaction(I2C0_ID, &trans_r_rsta);*/

		i++;
	}

	if(!(i % 5000)){

		int a = who_am_i;
		//printf("WHO AM I: %X\n", who_am_i);
		// printf("X: %u\n", axis_data.x_axis);
		// printf("Y: %u\n", axis_data.y_axis);
		// printf("Z: %u\n", axis_data.z_axis);
	}

	i++;

}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void read_reg(uint8_t* reg_add, uint8_t* read_data, uint8_t bytes_to_read){
	i2c_transaction_t trans_w_rsta = { .mode = I2C_WRITE_MODE, .address = FXOS8700CQ_ADD, .ptr = reg_add, .count = 1, .next_rsta = true};
	I2C_NewTransaction(I2C0_ID, &trans_w_rsta);

	i2c_transaction_t trans_r_rsta = { .mode = I2C_READ_MODE, .address = FXOS8700CQ_ADD, .ptr = read_data, .count = bytes_to_read, .next_rsta = false};
	I2C_NewTransaction(I2C0_ID, &trans_r_rsta);
}


/*******************************************************************************
 ******************************************************************************/
