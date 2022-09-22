/**
 * @file utils.h
 * @brief File to dump everything that is o may be useful thoughout the project but doesn't belog to one module
 * @author Olivia De Vincenti
 */

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
