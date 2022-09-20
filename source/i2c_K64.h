/***************************************************************************//**
  @file     i2c.c
  @brief    I2C MCAL layer
  @author   Olivia De Vincenti
 ******************************************************************************/

#ifndef _I2C_K64_H_
#define _I2C_K64_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdbool.h>
#include "MK64F12.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
/* I2C	*	Signal	*	MK64 Pin	*	MUX		*	Kinetis Pin		*/
// 0		SCL			PTE24			5			Si
// 0		SCL			PTB0			2			No
// 0		SCL			PTB2			2			Si
// 0		SCL			PTD2			7			Si
// 0		SDA			PTE25			5			Si
// 0		SDA			PTB1			2			No
// 0		SDA			PTB3			2			Si
// 0		SDA			PTD3			7			Si
// 1		SCL			PTE1			6			No
// 1		SCL			PTC10			2			Si
// 1		SDA			PTE0			6			No
// 1		SDA			PTC11			2			Si
// 2		SCL			PTA12			5			No
// 2		SCL			PTA14			5			No
// 2		SDA			PTA13			5			No

/**
 * @brief Get bit from register
 * 
 * @param x: Register to read bit from
 * @param n: Number of bit to read
 */
#define GET_BIT(x, n)			(((x) >> (n)) & 1 ) 

/**
 * @brief Set specified bit in register
 * 
 * @param x: Register to set bit in
 * @param n: Number of bit to set
 */
#define SET_BIT(x, n)   		((x) | ( 1 << (n)))

/**
 * @brief Clear specified bit in register
 * 
 * @param x: Register to clear bit in
 * @param n: Number of bit to clear
 */
#define CLEAR_BIT(x, n) 		((x) & ~( 1 << (n)))

/**
 * @brief Changes a bit in a register to the one specified
 * 
 * @param x: Register in which to set bit
 * @param b: Bit value to set
 * @param n: Number of bit to set
 */
#define CHANGE_BIT(x, b, n)		((b) ? SET_BIT(x, n) : CLEAR_BIT(x, n))

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
static I2C_Type* const I2C_ptrs[] = I2C_BASE_PTRS;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

void i2c_enable_clock_gating(uint8_t id);

void i2c_disable(I2C_Type* i2c_ptr);

void i2c_enable(I2C_Type* i2c_ptr);

void i2c_transmit_ack(I2C_Type* i2c_ptr);

void i2c_set_baud_rate(I2C_Type* i2c_ptr, uint32_t baud_rate);

void i2c_set_TX(I2C_Type* i2c_ptr);

void i2c_set_RX(I2C_Type* i2c_ptr);

bool i2c_send_start_signal(I2C_Type* i2c_ptr);

bool i2c_send_stop_signal(I2C_Type* i2c_ptr);

bool i2c_repeated_start(I2C_Type* i2c_ptr);

void i2c_write_to_data_register(I2C_Type* i2c_ptr, uint8_t data);

uint8_t i2c_read_data_register(I2C_Type* i2c_ptr);

bool i2c_is_transfer_complete(I2C_Type* i2c_ptr);

bool i2c_is_bus_busy(I2C_Type* i2c_ptr);

bool i2c_was_ack(I2C_Type* i2c_ptr);

bool i2c_was_arbitration_lost(I2C_Type* i2c_ptr);

bool i2c_interrupt_pending(I2C_Type* i2c_ptr);

void i2c_disable_interrupt_flag(I2C_Type* i2c_ptr);

// General call: used to address all slaves at the same time, only allows write operations, not read
void i2c_enable_general_call(I2C_Type* i2c_ptr);

void i2c_disable_general_call(I2C_Type* i2c_ptr);

void i2c_enable_start_stop_IRQ(I2C_Type* i2c_ptr);

bool i2c_did_bus_start(I2C_Type* i2c_ptr);

void i2c_did_bus_stop(I2C_Type* i2c_ptr);

void i2c_disable_start_stop_IRQ(I2C_Type* i2c_ptr);

void i2c_set_7_bit_address(I2C_Type* i2c_ptr);

void i2c_set_10_bit_address(I2C_Type* i2c_ptr);

// Only available for 10 bit address mode
void i2c_set_upper_3_address_bits(I2C_Type* i2c_ptr, uint8_t b7, uint8_t b8, uint8_t b9);

void i2c_set_normal_drive_mode(I2C_Type* i2c_ptr);

void i2c_set_high_drive_mode(I2C_Type* i2c_ptr);

void i2c_set_filter_factor(I2C_Type* i2c_ptr, uint8_t factor);

/*******************************************************************************
 ******************************************************************************/

#endif // _I2C_K64_H_
