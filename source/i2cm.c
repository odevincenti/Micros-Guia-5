/***************************************************************************//**
  @file     i2c.c
  @brief    I2C driver
  @author   Olivia De Vincenti
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "fifo.h"
#include "i2cm.h"
#include "hardware.h"
#include "board.h"
#include "MK64F12.h"
#include "gpio.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/******************* PINS ********************/

#define I2C_DEVELOPMENT_MODE
#define I2C_SET_FAKE_READ
// TODO: Tener 3 modos según el futuro uso:
//		- Sin fake read
//		- Con fake read
//		- Ambos: Chequea si ese periférico necesita fake read o no

// I2C0
#define I2C0_SCL		PTE24
#define I2C0_SCL_MUX	5
#define I2C0_SDA		PTE25
#define I2C0_SDA_MUX	5

// I2C1
#define I2C1_SCL		PTC10
#define I2C1_SCL_MUX	2
#define I2C1_SDA		PTC11
#define I2C1_SDA_MUX	2

// I2C2
#define I2C2_SCL		PORTNUM2PIN(PA, 12)
#define I2C2_SCL_MUX	5
#define I2C2_SDA		PORTNUM2PIN(PA, 13)
#define I2C2_SDA_MUX	5

#define I2C0_TEST_PIN	PTD1
#define I2C1_TEST_PIN	PTB11
#define I2C2_TEST_PIN	PTB20

#define I2C_IN_ISR		1
#define I2C_OUT_ISR		0

#define MUL 0x02			// mul = 4
#define ICR_100K 0x17		// SCL divider = 26 (I2C divider and hold values table - Reference Manual 51.4.1.10)

#define I2C_MAX_TRANS_BUFFER	16

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
enum {I2C_WRITE, I2C_READ, I2C_FAKE_READ, I2C_IDLE};

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
void I2C_ISR(uint8_t id);
void I2C_fsm(uint8_t id);
void I2C_start_transaction(uint8_t id);
bool I2C_push_transaction(uint8_t id, i2c_transaction_t* trans);
i2c_transaction_t* I2C_pull_transaction(uint8_t id);
void I2C_reset(uint8_t id);

void i2c_enable_clock_gating(uint8_t id);
void i2c_enable_pins(uint8_t id);
void i2c_enable(I2C_Type* i2c_ptr);
void i2c_disable(I2C_Type* i2c_ptr);
void i2c_enable_IRQ(uint8_t id, I2C_Type* i2c_ptr);
void i2c_set_baud_rate(I2C_Type* i2c_ptr);
void i2c_enable_start_stop_IRQ(I2C_Type* i2c_ptr);
void i2c_disable_start_stop_IRQ(I2C_Type* i2c_ptr);

__ISR__ I2C0_IRQHandler(void);
__ISR__ I2C1_IRQHandler(void);
__ISR__ I2C2_IRQHandler(void);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static pin_t const I2C_SCL_pins[I2C_N] = {I2C0_SCL, I2C1_SCL, I2C2_SCL};
static pin_t const I2C_SDA_pins[I2C_N] = {I2C0_SDA, I2C1_SDA, I2C2_SDA};
static uint8_t const I2C_SCL_MUX[I2C_N] = {I2C0_SCL_MUX, I2C1_SCL_MUX, I2C2_SCL_MUX};
static uint8_t const I2C_SDA_MUX[I2C_N] = {I2C0_SDA_MUX, I2C1_SDA_MUX, I2C2_SDA_MUX};
static uint8_t const I2C_TEST_PIN[I2C_N] = {I2C0_TEST_PIN, I2C1_TEST_PIN, I2C2_TEST_PIN};

static PORT_Type* const PORT_PTRS[] = PORT_BASE_PTRS;
static I2C_Type* const I2C_PTRS[] = I2C_BASE_PTRS;

static uint16_t const PORT_SCG[] = {SIM_SCGC5_PORTA_MASK, SIM_SCGC5_PORTB_MASK, SIM_SCGC5_PORTC_MASK, SIM_SCGC5_PORTD_MASK, SIM_SCGC5_PORTE_MASK};
static uint8_t const I2C_NVIC[] = {I2C0_IRQn, I2C1_IRQn, I2C2_IRQn};

static uint8_t I2C_state[] = {I2C_IDLE, I2C_IDLE, I2C_IDLE};

static bool I2C_init[] = {false, false, false};
// static bool i2c_loading[] = {false, false, false};
static uint8_t I2C_error_reg[] = {I2C_NO_ERR, I2C_NO_ERR, I2C_NO_ERR};

static fifo_id_t i2c_fifo[I2C_N] = {FIFO_INVALID_ID, FIFO_INVALID_ID, FIFO_INVALID_ID};
static i2c_transaction_t* i2c_curr_trans[I2C_N] = {&i2c_empty_trans, &i2c_empty_trans, &i2c_empty_trans};

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void I2C_Init(uint8_t id){

	if(!I2C_init[id] && id < I2C_N){

		// Request FIFO
		i2c_fifo[id] = FIFO_GetId();

		// Init Test Pin
		gpioMode(I2C_TEST_PIN[id], OUTPUT);

		// Init I2C
		I2C_Type* i2c_ptr = I2C_PTRS[id];

		// Enable clock gating
		switch(id){
			case 0:
				SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK; break;
			case 1:
				SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK; break;
			case 2:
				SIM->SCGC1 |= SIM_SCGC1_I2C2_MASK; break;
		}		

		// Enable port clock gating
		SIM->SCGC5 |= PORT_SCG[PIN2PORT(I2C_SCL_pins[id])];
		SIM->SCGC5 |= PORT_SCG[PIN2PORT(I2C_SDA_pins[id])];

		// Enable pin					  										Open Drain			Mux							    Pull Enable		   Pullup
		PORT_PTRS[PIN2PORT(I2C_SCL_pins[id])]->PCR[PIN2NUM(I2C_SCL_pins[id])] = PORT_PCR_ODE(1) | PORT_PCR_MUX(I2C_SCL_MUX[id]) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		PORT_PTRS[PIN2PORT(I2C_SDA_pins[id])]->PCR[PIN2NUM(I2C_SDA_pins[id])] = PORT_PCR_ODE(1) | PORT_PCR_MUX(I2C_SDA_MUX[id]) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
		
		// Set baud rate
		i2c_ptr->F = I2C_F_ICR(ICR_100K) | I2C_F_MULT(MUL);

		// Reset Control Register
		i2c_ptr->C1 = 0x0;			

		// Enable I2C
		i2c_ptr->C1 |= I2C_C1_IICEN_MASK;

		// Enable interrupt
		i2c_ptr->C1 |= I2C_C1_IICIE_MASK;
		//i2c_ptr->FLT |= I2C_FLT_SSIE_MASK;
		NVIC_EnableIRQ(I2C_NVIC[id]);	

		I2C_init[id] = true;
	}
}

bool I2C_NewTransaction(uint8_t id, i2c_transaction_t* trans){
	//if (trans->mode == I2C_READ_MODE){ trans->count++; }
	bool b = FIFO_PushToBuffer(i2c_fifo[id], trans);
	if (!b && FIFO_GetBufferLength(i2c_fifo[id]) == 1 && I2C_state[id] == I2C_IDLE){
		I2C_start_transaction(id);
	}
	return b;
}

bool I2C_IsIDTaken(uint8_t id){
	return I2C_init[id];
}

void I2C_FreeID(uint8_t id){
	I2C_reset(id);
	I2C_init[id] = false;
}

uint8_t I2C_WasError(uint8_t id){
	return I2C_error_reg[id];
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void I2C_ISR(uint8_t id){
	I2C_Type* i2c_ptr = I2C_PTRS[id];
	// if ((i2c_ptr->S & I2C_S_IICIF_MASK) == I2C_S_IICIF_MASK){		// If interrupt flag on:
		
		if ((i2c_ptr->S & I2C_S_TCF_MASK) == I2C_S_TCF_MASK){
			I2C_fsm(id);		// Go to FSM
			i2c_ptr->S |= I2C_S_IICIF_MASK;		// Disable interrupt flag
		
		} else if ((i2c_ptr->S & I2C_S_ARBL_MASK) == I2C_S_ARBL_MASK){
			i2c_ptr->S |= I2C_S_ARBL_MASK;		// Disable interrupt flag
			i2c_ptr->S |= I2C_S_IICIF_MASK;		// Disable interrupt flag
			i2c_ptr->C1 &= ~I2C_C1_MST_MASK;	// Stop

			I2C_state[id] = I2C_IDLE;			// Set next state
			I2C_error_reg[id] = I2C_ARB_LOST;	// Error, arbitration lost

		} else {
			i2c_ptr->S |= I2C_S_IICIF_MASK;		// Disable interrupt flag
		}
	// }
}

void I2C_fsm(uint8_t id){

	I2C_Type* i2c_ptr = I2C_PTRS[id];

	switch(I2C_state[id]){

	case I2C_WRITE:
		if ((i2c_ptr->S & I2C_S_RXAK_MASK) != I2C_S_RXAK_MASK){		// If ACK:
			i2c_ptr->C1 |= I2C_C1_TX_MASK;							// Set TX mode
			i2c_ptr->D = *i2c_curr_trans[id]->ptr++;				// Write to data
			if(!(--i2c_curr_trans[id]->count)){					// If count is 0 -> Done writing:
				I2C_state[id] = I2C_IDLE;
			}
		} else {											// If NACK:
			i2c_ptr->C1 &= ~I2C_C1_MST_MASK;				// Stop
			// I2C_error_reg[id] = I2C_ERR_NACK;			// Reg error
			I2C_state[id] = I2C_IDLE;
		}
		break;
	
	case I2C_READ:
		i2c_ptr->C1 &= ~I2C_C1_TX_MASK;					// Set RX mode
		if (--i2c_curr_trans[id]->count == 1){
			i2c_ptr->C1 |= I2C_C1_TXAK_MASK;			// Send NACK
			*i2c_curr_trans[id]->ptr++ = i2c_ptr->D;		// Read data
		} else if (i2c_curr_trans[id]->count){			// If this is not the last byte:
			i2c_ptr->C1 &= ~I2C_C1_TXAK_MASK;			// Send ACK
			*i2c_curr_trans[id]->ptr++ = i2c_ptr->D;		// Read data
		} else {									// If count is 0 -> Done reading:
			i2c_ptr->C1 &= ~I2C_C1_MST_MASK;			// Stop
			*i2c_curr_trans[id]->ptr++ = i2c_ptr->D;		// Read data
			i2c_ptr->C1 |= I2C_C1_TX_MASK;				// Clear RX mode
			I2C_state[id] = I2C_IDLE;
			if (!FIFO_IsBufferEmpty(i2c_fifo[id])){
				I2C_start_transaction(id);
			}
		}
		break;

#ifdef I2C_SET_FAKE_READ
	case I2C_FAKE_READ:
		i2c_ptr->C1 &= ~I2C_C1_TX_MASK;			// Set RX mode
		if (i2c_curr_trans[id]->count == 1){
			i2c_ptr->C1 |= I2C_C1_TXAK_MASK;		// Send NACK
		} else {
			i2c_ptr->C1 &= ~I2C_C1_TXAK_MASK;		// Send ACK
		}
		i2c_ptr->D;		// First read
		I2C_state[id] = I2C_READ;				// Next state: READ
		break;
#endif

	case I2C_IDLE:
		if (!i2c_curr_trans[id]->next_rsta && (i2c_ptr->C1 & I2C_C1_MST_MASK) == I2C_C1_MST_MASK){		// If not repeated start
			i2c_ptr->C1 &= ~I2C_C1_MST_MASK;		// Stop
		}
		if (!FIFO_IsBufferEmpty(i2c_fifo[id])){
			I2C_start_transaction(id);
		}
		break;

	default:
		break;
	}
}

void I2C_start_transaction(uint8_t id){
	I2C_Type* i2c_ptr = I2C_PTRS[id];
	i2c_ptr->C1 |= I2C_C1_TX_MASK;				// TX mode
	if (!i2c_curr_trans[id]->next_rsta){	// If new transaction:
		FIFO_PullFromBuffer(i2c_fifo[id], i2c_curr_trans[id]);		// Get next transaction
		// There's a minimum free time that must go by betweet STOP and START (See Section 3.8.8: https://www.nxp.com/docs/en/data-sheet/K64P144M120SF5.pdf)
		while ((i2c_ptr->S & I2C_S_BUSY_MASK) == I2C_S_BUSY_MASK);
		i2c_ptr->C1 |= I2C_C1_MST_MASK;			// Start
	} else{									// If a repeated start follows:
		FIFO_PullFromBuffer(i2c_fifo[id], i2c_curr_trans[id]);		// Get next transaction
		i2c_ptr->C1 |= I2C_C1_RSTA_MASK;		// Repeated start
	}
	i2c_ptr->D = (i2c_curr_trans[id]->address << 1) | i2c_curr_trans[id]->mode;				// Write address
#ifdef I2C_SET_FAKE_READ
	I2C_state[id] = (i2c_curr_trans[id]->mode == I2C_READ) ? I2C_FAKE_READ : I2C_WRITE;		// Set next state
#else
	I2C_state[id] = (i2c_curr_trans[id]->mode == I2C_READ) ? I2C_READ : I2C_WRITE;		// Set next state
#endif

}

void I2C_reset(uint8_t id){
	I2C_Type* i2c_ptr = I2C_PTRS[id];
	i2c_disable(i2c_ptr);
	i2c_disable_start_stop_IRQ(i2c_ptr);
	i2c_ptr->S |= I2C_S_IICIF_MASK;
}

/******************************************************************************/

__ISR__ I2C0_IRQHandler(void) {
#ifdef I2C_DEVELOPMENT_MODE
	gpioWrite(I2C0_TEST_PIN, I2C_IN_ISR);
#endif
    I2C_ISR(0);
#ifdef I2C_DEVELOPMENT_MODE
	gpioWrite(I2C0_TEST_PIN, I2C_OUT_ISR);
#endif
}

__ISR__ I2C1_IRQHandler(void) {
#ifdef I2C_DEVELOPMENT_MODE
	gpioWrite(I2C1_TEST_PIN, I2C_IN_ISR);
#endif
    I2C_ISR(1);
#ifdef I2C_DEVELOPMENT_MODE
	gpioWrite(I2C1_TEST_PIN, I2C_OUT_ISR);
#endif
}

__ISR__ I2C2_IRQHandler(void) {
#ifdef I2C_DEVELOPMENT_MODE
	gpioWrite(I2C2_TEST_PIN, I2C_IN_ISR);
#endif
    I2C_ISR(2);
#ifdef I2C_DEVELOPMENT_MODE
	gpioWrite(I2C2_TEST_PIN, I2C_OUT_ISR);
#endif
}


// #define I2C0_SCL0_PIN		PORTNUM2PIN(PE, 24)
// #define I2C0_SCL1_PIN		PORTNUM2PIN(PB, 2)
// #define I2C0_SCL2_PIN		PORTNUM2PIN(PD, 2)
// #define I2C0_SCL3_PIN		PORTNUM2PIN(PB, 2)		// NON-FRDM
// #define I2C0_SDA0_PIN		PORTNUM2PIN(PE, 25)
// #define I2C0_SDA1_PIN		PORTNUM2PIN(PB, 3)
// #define I2C0_SDA2_PIN		PORTNUM2PIN(PD, 3)
// #define I2C0_SDA3_PIN		PORTNUM2PIN(PB, 1)		// NON-FRDM
// #define I2C1_SCL0_PIN		PORTNUM2PIN(PC, 10)
// #define I2C1_SCL1_PIN		PORTNUM2PIN(PE, 1)		// NON-FRDM
// #define I2C1_SDA0_PIN		PORTNUM2PIN(PC, 11)
// #define I2C1_SDA1_PIN		PORTNUM2PIN(PE, 0)		// NON-FRDM
// #define I2C2_SCL0_PIN		PORTNUM2PIN(PA, 12)		// NON-FRDM
// #define I2C2_SCL1_PIN		PORTNUM2PIN(PA, 14)		// NON-FRDM
// #define I2C2_SDA0_PIN		PORTNUM2PIN(PA, 13)		// NON-FRDM
