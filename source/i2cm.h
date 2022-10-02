/***************************************************************************//**
  @file     I2Cm.h
  @brief    I2C master mode driver
  @author   Olivia De Vincenti
 ******************************************************************************/

#ifndef _I2CM_H_
#define _I2CM_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define I2C_N	3

#define I2C_WRITE_MODE	0
#define I2C_READ_MODE	1
#define ERR_ADDRESS		0x0

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
enum {I2C0_ID, I2C1_ID, I2C2_ID};
enum {I2C_START, I2C_RSTART, I2C_STOP};
enum {I2C_NO_ERR, I2C_ERR_NACK, I2C_ERR_FULL, I2C_ERR_EMPTY, I2C_ARB_LOST};

typedef struct {
	bool mode;				// true for read, false for write
	uint8_t address;		// slave address
	uint8_t* ptr;			// ptr to read to or to write from
	uint8_t count;			// amount of bytes
	bool next_rsta;				// true if repeated start
} i2c_transaction_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
static i2c_transaction_t i2c_empty_trans = {.mode = true, .address = ERR_ADDRESS, .ptr = 0, .count = 0, .next_rsta = false};

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
void I2C_Init(uint8_t id);

bool I2C_NewTransaction(uint8_t id, i2c_transaction_t* trans);

bool I2C_IsBusFree(uint8_t id);

bool I2C_IsIDTaken(uint8_t id);

void I2C_FreeID(uint8_t id);

uint8_t I2C_WasError(uint8_t id);

/*******************************************************************************
 ******************************************************************************/

#endif // _I2CM_H_
