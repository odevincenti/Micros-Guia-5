/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "board.h"
#include "gpio.h"
#include "i2cm.h"
#include "MK64F12.h"
#include "hardware.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
static uint8_t write[4] = "Hola";
static uint8_t read[3];
static uint8_t read_rsta[2];

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	I2C_Init(I2C0_ID);
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	static bool i = true;
	if(i){

		// i2c_transaction_t trans_w = { .mode = I2C_WRITE_MODE, .address = 0x1D, .ptr = &write[0], .count = 4, .next_rsta = false};
		// I2C_NewTransaction(I2C0_ID, &trans_w);
		// start
		// 00111010 0

		i2c_transaction_t trans_r = { .mode = I2C_READ_MODE, .address = 0x1D, .ptr = &read[0], .count = 3, .next_rsta = false};
		I2C_NewTransaction(I2C0_ID, &trans_r);
		// start
		// 00111010 0

		i2c_transaction_t trans_w_rsta = { .mode = I2C_WRITE_MODE, .address = 0x1D, .ptr = &write[0], .count = 4, .next_rsta = true};
		I2C_NewTransaction(I2C0_ID, &trans_w_rsta);

		i2c_transaction_t trans_r_rsta = { .mode = I2C_READ_MODE, .address = 0x1D, .ptr = &read_rsta[0], .count = 2, .next_rsta = false};
		I2C_NewTransaction(I2C0_ID, &trans_r_rsta);

		i = false;

	}

}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/



/*******************************************************************************
 ******************************************************************************/
