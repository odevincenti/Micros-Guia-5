/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

// #include "board.h"
// #include "gpio.h"
#include "i2cm.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	I2C_Init(I2C0_ID);
	uint8_t write[] = "Hola";
	i2c_transaction_t trans_w = { .mode = I2C_WRITE_MODE, .address = 0x05, .ptr = &write[0], .count = 4, .next_rsta = false};
	I2C_NewTransaction(I2C0_ID, &trans_w);

	uint8_t read[5];
	i2c_transaction_t trans_r = { .mode = I2C_READ_MODE, .address = 0x05, .ptr = &read[0], .count = 5, .next_rsta = false};
	I2C_NewTransaction(I2C0_ID, &trans_r);

	i2c_transaction_t trans_w_rsta = { .mode = I2C_WRITE_MODE, .address = 0x05, .ptr = &write[0], .count = 4, .next_rsta = true};
	I2C_NewTransaction(I2C0_ID, &trans_w_rsta);

	uint8_t read_rsta[9];
	i2c_transaction_t trans_r_rsta = { .mode = I2C_READ_MODE, .address = 0x05, .ptr = &read_rsta[0], .count = 9, .next_rsta = false};
	I2C_NewTransaction(I2C0_ID, &trans_r_rsta);
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{

}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/



/*******************************************************************************
 ******************************************************************************/
