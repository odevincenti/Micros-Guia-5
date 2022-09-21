/***************************************************************************//**
  @file     i2c.c
  @brief    I2C MCAL layer
  @author   Olivia De Vincenti
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "MK64F12.h"
#include "hardware.h"
#include "board.h"
#include "SysTick.h"
#include "i2c_K64.h"

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

/******************* PINS ********************/

// #define I2C0_SCL0_PIN		PTE24
// #define I2C0_SCL1_PIN		PTB2
// #define I2C0_SCL2_PIN		PTD2
// #define I2C0_SCL3_PIN		PTB2		// NON-FRDM
// #define I2C0_SDA0_PIN		PTE25
// #define I2C0_SDA1_PIN		PTB3
// #define I2C0_SDA2_PIN		PTD3
// #define I2C0_SDA3_PIN		PTB1		// NON-FRDM
// #define I2C1_SCL0_PIN		PTC10
// #define I2C1_SCL1_PIN		PTE1		// NON-FRDM
// #define I2C1_SDA0_PIN		PTC11
// #define I2C1_SDA1_PIN		PTE0		// NON-FRDM
// #define I2C2_SCL0_PIN		PTA12		// NON-FRDM
// #define I2C2_SCL1_PIN		PTA14		// NON-FRDM
// #define I2C2_SDA0_PIN		PTA13		// NON-FRDM

// I2C0
#define I2C0_SCL		I2C0_SCL1_PIN
#define I2C0_SCL_MUX	2
#define I2C0_SDA		I2C0_SDA1_PIN
#define I2C0_SDA_MUX	2

// I2C1
#define I2C1_SCL		I2C1_SCL0_PIN
#define I2C1_SCL_MUX	2
#define I2C1_SDA		I2C1_SDA0_PIN
#define I2C1_SDA_MUX	2

// I2C2
#define I2C2_SCL		I2C2_SCL0_PIN
#define I2C2_SCL_MUX	5
#define I2C2_SDA		I2C2_SDA0_PIN
#define I2C2_SDA_MUX	5

#define MUL 0x1		// mul = 2
#define ICR 0x2B	// SCL divider = 512 (I2C divider and hold values table - Reference Manual 51.4.1.10)

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
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

const pin_t I2C_SCL[I2C_N] = {I2C0_SCL, I2C1_SCL, I2C2_SCL};
const pin_t I2C_SDA[I2C_N] = {I2C0_SDA, I2C1_SDA, I2C2_SDA};
const uint8_t I2C_SCL_MUX[I2C_N] = {I2C0_SCL_MUX, I2C1_SCL_MUX, I2C2_SCL_MUX};
const uint8_t I2C_SDA_MUX[I2C_N] = {I2C0_SDA_MUX, I2C1_SDA_MUX, I2C2_SDA_MUX};
const PORT_Type * PORT_ptr[] = {PORTA_BASE, PORTB_BASE, PORTC_BASE, PORTD_BASE, PORTE_BASE};
const uint8_t PORT_SCG[] = {SIM_SCGC5_PORTA_MASK, SIM_SCGC5_PORTB_MASK, SIM_SCGC5_PORTC_MASK, SIM_SCGC5_PORTD_MASK, SIM_SCGC5_PORTE_MASK};
const uint8_t I2C_NVIC[] = {I2C0_IRQn, I2C1_IRQn, I2C2_IRQn};

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void i2c_enable_pins(uint8_t id){
	pin_t scl = I2C_SCL[id];
	pin_t sda = I2C_SDA[id];
	PORT_Type* scl_port_ptr = PORT_ptr[PIN2PORT(scl)];
	PORT_Type* sda_port_ptr = PORT_ptr[PIN2PORT(scl)];

	// Enable clock gating
	SIM->SCGC5 |= PORT_SCG[PIN2PORT(scl)];
	SIM->SCGC5 |= PORT_SCG[PIN2PORT(sda)];

	// Enable pin
	scl_port_ptr->PCR[PIN2NUM(scl)] = PORT_PCR_ODE(1) | PORT_PCR_MUX(I2C_SCL_MUX[id]);		// Select mux & open drain mode
	sda_port_ptr->PCR[PIN2NUM(sda)] = PORT_PCR_ODE(1) | PORT_PCR_MUX(I2C_SDA_MUX[id]);

}

// void i2c_enable_pin_IRQ(uint8_t id){
// 	pin_t scl = I2C_SCL[id];
// 	pin_t sda = I2C_SDA[id];
// 	PORT_Type* scl_port_ptr = PORT_ptr[PIN2PORT(scl)];
// 	PORT_Type* sda_port_ptr = PORT_ptr[PIN2PORT(scl)];

// 	scl_port_ptr->PCR[PIN2NUM(scl)] |= PORT_PCR_ISF_MASK;	
// 	sda_port_ptr->PCR[PIN2NUM(sda)] = PORT_PCR_ODE(1) | PORT_PCR_MUX(I2C_SDA_MUX[id]);

// 	PORT_PCR_IRQC(0) | PORT_PCR_ISF_MASK

// }


void i2c_enable_clock_gating(uint8_t id){
	switch(id)
	{
		case 0:
			SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
			break;
		
		case 1:
			SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
			break;

		case 2:
			SIM->SCGC1 |= SIM_SCGC1_I2C2_MASK;
			break;

		default:
			break;
	}
}

void i2c_disable(I2C_Type* i2c_ptr){
	i2c_ptr->C1 &= ~I2C_C1_IICEN_MASK;
}

void i2c_enable(I2C_Type* i2c_ptr){
	i2c_ptr->S |= I2C_S_TCF_MASK;
	i2c_ptr->C1 |= I2C_C1_IICEN_MASK;
}

void i2c_enable_IRQ(uint8_t id, I2C_Type* i2c_ptr){
	i2c_ptr->C1 |= I2C_C1_IICIE_MASK;
	i2c_enable_pin_IRQ(id);
	i2c_ptr->S |= I2C_S_IICIF_MASK;
	i2c_enable_start_stop_IRQ(i2c_ptr);
	NVIC_EnableIRQ(I2C_NVIC[id]);
}

void i2c_transmit_ack(I2C_Type* i2c_ptr){
	i2c_ptr->C1 &= ~I2C_C1_TXAK_MASK;
}

void i2c_set_baud_rate(I2C_Type* i2c_ptr){//}, uint32_t baud_rate){
	i2c_ptr->F = I2C_F_ICR(ICR) | I2C_F_MULT(MUL);
}

void i2c_set_TX(I2C_Type* i2c_ptr){
	i2c_ptr->C1 |= I2C_C1_TX_MASK;
}

void i2c_set_RX(I2C_Type* i2c_ptr){
	i2c_ptr->C1 &= ~I2C_C1_TX_MASK;
}

bool i2c_send_start_signal(I2C_Type* i2c_ptr){
#ifdef I2C_SAFE_MODE
	if ( (i2c_ptr->C1 & I2C_C1_MST_MASK) != I2C_C1_MST_MASK ){		// If MST is clear
#endif
		i2c_ptr->C1 |= I2C_C1_MST_MASK;			// Send start bit
		return true;
#ifdef I2C_SAFE_MODE
	} else {				// If MST is set
		return false;		// Start bit can't be sent, transmission in progress
	}
#endif
}

bool i2c_send_stop_signal(I2C_Type* i2c_ptr){
#ifdef I2C_SAFE_MODE
	if ( (i2c_ptr->C1 & I2C_C1_MST_MASK) == I2C_C1_MST_MASK ){		// If MST is set
#endif
		i2c_ptr->C1 &= ~I2C_C1_MST_MASK;		// Send stop bit
		return true;
#ifdef I2C_SAFE_MODE
	} else {				// If MST is clear
		return false;		// Stop bit can't be sent, no transmission in progress
	}
#endif
}

bool i2c_repeated_start(I2C_Type* i2c_ptr){
#ifdef I2C_SAFE_MODE
	if ( (i2c_ptr->C1 & I2C_C1_MST_MASK) == I2C_C1_MST_MASK ){		// If MST is set, start bit was sent
#endif
		i2c_ptr->C1 |= I2C_C1_RSTA_MASK;		// Set repeated start
		return true;
#ifdef I2C_SAFE_MODE
	} else {				// If MST is clear, no start bit was sent
		return false;		// Can't do a repeated start
	}
#endif
}

void i2c_write_to_data_register(I2C_Type* i2c_ptr, uint8_t data){
	i2c_ptr->D = data;
}

uint8_t i2c_read_data_register(I2C_Type* i2c_ptr){
	return i2c_ptr->D;
}

bool i2c_is_transfer_complete(I2C_Type* i2c_ptr){
	if ( (i2c_ptr->S & I2C_S_TCF_MASK) == I2C_S_TCF_MASK ){		// Extract TCF bit
		return true;			// If bit set, transfer complete
	} else {
		return false;			// If bit clear, transfer in progress
	}
}

// bool i2c_set_no_transfer(I2C_Type* i2c_ptr){
// 	i2c_ptr->S |= I2C_S_TCF_MASK;
// }

bool i2c_is_bus_busy(I2C_Type* i2c_ptr){
	if ( (i2c_ptr->S & I2C_S_BUSY_MASK) == I2C_S_BUSY_MASK ){		// Extract BUSY bit
		return true;			// If bit set, bus is busy
	} else {
		return false;			// If bit clear, bus is idle
	}
}

bool i2c_was_ack(I2C_Type* i2c_ptr){
	if ( (i2c_ptr->S & I2C_S_RXAK_MASK) == I2C_S_RXAK_MASK ){
		return false;			// If bit set, no acknowledge signal detected
	} else {
		return true;			// If bit clear, acknowledge signal was received
	}
}

bool i2c_was_arbitration_lost(I2C_Type* i2c_ptr){
	if ( (i2c_ptr->S & I2C_S_ARBL_MASK) == I2C_S_ARBL_MASK ){		// Extract ARBL bit
		i2c_ptr->S |= I2C_S_ARBL_MASK;			// Clear bit (51.3.4: "The ARBL bit must be cleared by software, by writing 1 to it")
		return true;			// If bit set, arbitration procedure is lost
	} else {
		return false;			// If bit clear, standard bus operation 
	}
}

bool i2c_interrupt_pending(I2C_Type* i2c_ptr){
	if ( (i2c_ptr->S & I2C_S_IICIF_MASK) == I2C_S_IICIF_MASK ){
		return true;			// If bit set, interrupt pending
	} else {
		return false;			// If bit clear, no interrupt pending
	}
}

void i2c_disable_interrupt_flag(I2C_Type* i2c_ptr){
	if ( (i2c_ptr->S & I2C_S_IICIF_MASK) == I2C_S_IICIF_MASK ){
		i2c_ptr->S |= I2C_S_IICIF_MASK;
	}
}

// General call: used to address all slaves at the same time, only allows write operations, not read
void i2c_enable_general_call(I2C_Type* i2c_ptr){		
	i2c_ptr->C2 |= I2C_C2_GCAEN_MASK;
}

void i2c_disable_general_call(I2C_Type* i2c_ptr){
	i2c_ptr->C2 &= ~I2C_C2_GCAEN_MASK;
}

void i2c_enable_start_stop_IRQ(I2C_Type* i2c_ptr){
	i2c_ptr->FLT |= I2C_FLT_SSIE_MASK;
}

bool i2c_did_bus_start(I2C_Type* i2c_ptr){
	if ( (i2c_ptr->FLT & I2C_FLT_STARTF_MASK) == I2C_FLT_STARTF_MASK ){
		i2c_ptr->FLT |= I2C_FLT_STARTF_MASK;			// Clear STARTF bit by writing 1 into it
		return true;
	} else {
		return false;
	}
}

void i2c_did_bus_stop(I2C_Type* i2c_ptr){
	if ( (i2c_ptr->FLT & I2C_FLT_STOPF_MASK) == I2C_FLT_STOPF_MASK ){
		i2c_ptr->FLT |= I2C_FLT_STOPF_MASK;				// Clear STOPF bit by writing 1 into it
		return true;
	} else {
		return false;
	}
}

void i2c_disable_start_stop_IRQ(I2C_Type* i2c_ptr){
	i2c_did_bus_stop(i2c_ptr);
	i2c_did_bus_start(i2c_ptr);
	i2c_ptr->FLT &= ~I2C_FLT_SSIE_MASK;
}

void i2c_set_7_bit_address(I2C_Type* i2c_ptr){
	i2c_ptr->C2 &= ~I2C_C2_ADEXT_MASK;
}

void i2c_set_10_bit_address(I2C_Type* i2c_ptr){
	i2c_ptr->C2 |= I2C_C2_ADEXT_MASK;
}

// Only available for 10 bit address mode
void i2c_set_upper_3_address_bits(I2C_Type* i2c_ptr, uint8_t b7, uint8_t b8, uint8_t b9){
#ifdef I2C_SAFE_MODE
	if( (i2c_ptr->C2 & I2C_C2_ADEXT_MASK) == I2C_C2_ADEXT_MASK ){
#endif
		i2c_ptr->C2 = CHANGE_BIT(i2c_ptr->C2, b9, 2);		// Set bit 9
		i2c_ptr->C2 = CHANGE_BIT(i2c_ptr->C2, b8, 1);		// Set bit 8
		i2c_ptr->C2 = CHANGE_BIT(i2c_ptr->C2, b7, 0);		// Set bit 7
#ifdef I2C_SAFE_MODE
	}
#endif
}

void i2c_set_normal_drive_mode(I2C_Type* i2c_ptr){
	i2c_ptr->C2 &= ~I2C_C2_HDRS_MASK;
}

void i2c_set_high_drive_mode(I2C_Type* i2c_ptr){
	i2c_ptr->C2 |= I2C_C2_HDRS_MASK;
}


void i2c_set_filter_factor(I2C_Type* i2c_ptr, uint8_t factor){
#ifdef I2C_SAFE_MODE
	if (factor <= I2C_FLT_FLT_MASK){
#endif
		i2c_ptr->FLT = CHANGE_BIT(i2c_ptr->FLT, GET_BIT(factor, 3), 3);
		i2c_ptr->FLT = CHANGE_BIT(i2c_ptr->FLT, GET_BIT(factor, 2), 2);
		i2c_ptr->FLT = CHANGE_BIT(i2c_ptr->FLT, GET_BIT(factor, 1), 1);
		i2c_ptr->FLT = CHANGE_BIT(i2c_ptr->FLT, GET_BIT(factor, 0), 0);
#ifdef I2C_SAFE_MODE
	}
#endif
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/
/******************************************************************************/