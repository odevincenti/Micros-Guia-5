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

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/******************* PINS ********************/

// I2C0
#define I2C0_SCL		I2C0_SCL0_PIN
#define I2C0_SCL_MUX	5
#define I2C0_SDA		I2C0_SDA0_PIN
#define I2C0_SDA_MUX	5

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

#define MUL 0x0		// mul = 1
#define ICR 0x1A	// SCL divider = 26 (I2C divider and hold values table - Reference Manual 51.4.1.10)

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

static PORT_Type* const PORT_PTRS[] = PORT_BASE_PTRS;
static I2C_Type* const I2C_PTRS[] = I2C_BASE_PTRS;

static uint16_t const PORT_SCG[] = {SIM_SCGC5_PORTA_MASK, SIM_SCGC5_PORTB_MASK, SIM_SCGC5_PORTC_MASK, SIM_SCGC5_PORTD_MASK, SIM_SCGC5_PORTE_MASK};
static uint8_t const I2C_NVIC[] = {I2C0_IRQn, I2C1_IRQn, I2C2_IRQn};

static uint8_t I2C_state[] = {I2C_IDLE, I2C_IDLE, I2C_IDLE};

static bool I2C_init[] = {false, false, false};
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
		i2c_ptr->F = I2C_F_ICR(ICR) | I2C_F_MULT(MUL);

		// Reset Control Register
		i2c_ptr->C1 = 0x0;			

		// Enable I2C
		i2c_ptr->C1 |= I2C_C1_IICEN_MASK;

		// Enable interrupt
		i2c_ptr->C1 |= I2C_C1_IICIE_MASK;
		i2c_ptr->FLT |= I2C_FLT_SSIE_MASK;
		NVIC_EnableIRQ(I2C_NVIC[id]);	

		I2C_init[id] = true;
	}
}

bool I2C_NewTransaction(uint8_t id, i2c_transaction_t* trans){
	bool b = FIFO_PushToBuffer(i2c_fifo[id], trans);
	if (!b && I2C_state[id] == I2C_IDLE && FIFO_GetBufferLength(i2c_fifo[id]) == 1){
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
	if ((i2c_ptr->S & I2C_S_IICIF_MASK) == I2C_S_IICIF_MASK){		// If interrupt flag on:
		if ((i2c_ptr->FLT & I2C_FLT_STOPF_MASK) == I2C_FLT_STOPF_MASK){		// If a STOP was sent:
			i2c_ptr->FLT |= I2C_FLT_STOPF_MASK;			// Disable flag
			i2c_ptr->S |= I2C_S_IICIF_MASK;		// Disable interrupt flag
			if (!FIFO_IsBufferEmpty(i2c_fifo[id])){
				I2C_start_transaction(id);
			} else {
				// i2c_ptr->C1 &= ~(I2C_C1_IICEN_MASK | I2C_C1_IICIE_MASK);
			}

		} else if ((i2c_ptr->FLT & I2C_FLT_STARTF_MASK) == I2C_FLT_STARTF_MASK){	// If a START was sent:
			i2c_ptr->FLT |= I2C_FLT_STARTF_MASK;		// Disable flag
			i2c_ptr->S |= I2C_S_IICIF_MASK;		// Disable interrupt flag
			if ((i2c_ptr->S & I2C_S_TCF_MASK) == I2C_S_TCF_MASK){	// If transfer complete:
				I2C_fsm(id);		// Go to FSM
			}
		
		} else if ((i2c_ptr->S & I2C_S_ARBL_MASK) == I2C_S_ARBL_MASK){
			i2c_ptr->S |= I2C_S_ARBL_MASK;		// Disable interrupt flag
			i2c_ptr->S |= I2C_S_IICIF_MASK;		// Disable interrupt flag
			i2c_ptr->C1 &= ~I2C_C1_MST_MASK;			// Stop

			I2C_state[id] = I2C_IDLE;			// Set next state
			I2C_error_reg[id] = I2C_ARB_LOST;	// Error, arbitration lost

		} else if ((i2c_ptr->S & I2C_S_TCF_MASK) == I2C_S_TCF_MASK){
			i2c_ptr->S |= I2C_S_IICIF_MASK;		// Disable interrupt flag
			if ((i2c_ptr->S & I2C_S_TCF_MASK) == I2C_S_TCF_MASK){	// If transfer complete:
				I2C_fsm(id);		// Go to FSM
			}
			//I2C_state[id] = I2C_IDLE;			// Set next state
			//I2C_error_reg[id] = I2C_ARB_LOST;	// Error, arbitration lost
		} else {
			i2c_ptr->S |= I2C_S_IICIF_MASK;		// Disable interrupt flag
		}
	}
}

void I2C_fsm(uint8_t id){

	I2C_Type* i2c_ptr = I2C_PTRS[id];
	uint8_t data;

	switch(I2C_state[id]){

	case I2C_WRITE:
		if ((i2c_ptr->S & I2C_S_RXAK_MASK) != I2C_S_RXAK_MASK){		// If ACK:
			i2c_ptr->C1 |= I2C_C1_TX_MASK;							// Set TX mode
			i2c_ptr->D = *i2c_curr_trans[id]->ptr++;				// Write to data
			if(!(--i2c_curr_trans[id]->count)){					// If count is 0 -> Done writing:
				if (!i2c_curr_trans[id]->next_rsta){			// If not repeated start
					i2c_ptr->C1 &= ~I2C_C1_MST_MASK;			// Stop
				}												// FIXME: Si era rsta, vuelve a entrar a la ISR?
				I2C_state[id] = I2C_IDLE;
			}
		} else {											// If NACK:
			i2c_ptr->C1 &= ~I2C_C1_MST_MASK;				// Stop
			I2C_error_reg[id] = I2C_ERR_NACK;				// Reg error
			I2C_state[id] = I2C_IDLE;
		}
		break;
	
	case I2C_READ:
		i2c_ptr->C1 &= ~I2C_C1_TX_MASK;					// Set RX mode
		*i2c_curr_trans[id]->ptr++ = i2c_ptr->D;		// Read data
		if (--i2c_curr_trans[id]->count ){			// If this is not the last byte:
			i2c_ptr->C1 &= ~I2C_C1_TXAK_MASK;			// Send ACK
		} else {									// If count is 0 -> Done reading:
			i2c_ptr->C1 |= I2C_C1_TX_MASK;				// Clear RX mode
			if (!i2c_curr_trans[id]->next_rsta){		// If not repeated start
				i2c_ptr->C1 &= ~I2C_C1_MST_MASK;		// Stop
			}												// FIXME: Si era rsta, vuelve a entrar a la ISR?
			I2C_state[id] = I2C_IDLE;
		}
		break;

	case I2C_FAKE_READ:
		i2c_ptr->C1 &= ~I2C_C1_TX_MASK;			// Set RX mode
		i2c_ptr->C1 &= ~I2C_C1_TXAK_MASK;		// Send ACK
		i2c_ptr->D;								// Discard read
		I2C_state[id] = I2C_READ;				// Next state: READ
		break;

	case I2C_IDLE:
		if (!FIFO_IsBufferEmpty(i2c_fifo[id])){
			//I2C_start_transaction(id);
		}
		break;

	default:
		break;
	}
}


void I2C_start_transaction(uint8_t id){
	I2C_Type* i2c_ptr = I2C_PTRS[id];
	if (I2C_state[id] == I2C_IDLE && !FIFO_IsBufferEmpty(i2c_fifo[id])){				// Si el bus está libre: hay que mandar un start
		bool b = FIFO_PullFromBuffer(i2c_fifo[id], i2c_curr_trans[id]);
		i2c_ptr->C1 |= I2C_C1_TX_MASK;			// TX mode
		i2c_ptr->C1 |= I2C_C1_MST_MASK;			// Start
		i2c_ptr->D = (i2c_curr_trans[id]->address << 1) | i2c_curr_trans[id]->mode;		// Write address
		I2C_state[id] = (i2c_curr_trans[id]->mode == I2C_READ) ? I2C_FAKE_READ : I2C_WRITE;		// Set next state
	} else if (i2c_curr_trans[id]->next_rsta){		// If a repeated start follows
		bool b = FIFO_PullFromBuffer(i2c_fifo[id], i2c_curr_trans[id]);
		// todo: Add write mode
		i2c_ptr->C1 |= I2C_C1_TX_MASK;			// TX mode
		i2c_ptr->C1 |= I2C_C1_RSTA_MASK;		// Repeated start
		i2c_ptr->D = (i2c_curr_trans[id]->address << 1) | i2c_curr_trans[id]->mode;		// Write address
		I2C_state[id] = I2C_FAKE_READ;			// Set next state
	}
}

void I2C_reset(uint8_t id){
	I2C_Type* i2c_ptr = I2C_PTRS[id];
	i2c_disable(i2c_ptr);
	i2c_disable_start_stop_IRQ(i2c_ptr);
	i2c_ptr->S |= I2C_S_IICIF_MASK;
	// i2c_disable_pin_IRQ(id);
}

/******************************************************************************/

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

void i2c_enable_pins(uint8_t id){
	pin_t scl = I2C_SCL_pins[id];
	pin_t sda = I2C_SDA_pins[id];
	PORT_Type* scl_port_ptr = PORT_PTRS[PIN2PORT(scl)];
	PORT_Type* sda_port_ptr = PORT_PTRS[PIN2PORT(sda)];

	// Enable port clock gating
	SIM->SCGC5 |= PORT_SCG[PIN2PORT(scl)];
	SIM->SCGC5 |= PORT_SCG[PIN2PORT(sda)];

	// Enable pin					  Open Drain		Mux							    Pull Enable		   Pullup
	scl_port_ptr->PCR[PIN2NUM(scl)] = PORT_PCR_ODE(1) | PORT_PCR_MUX(I2C_SCL_MUX[id]) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	sda_port_ptr->PCR[PIN2NUM(sda)] = PORT_PCR_ODE(1) | PORT_PCR_MUX(I2C_SDA_MUX[id]) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

}

void i2c_enable(I2C_Type* i2c_ptr){
	// i2c_ptr->S |= I2C_S_TCF_MASK;	TCF is read-only
	i2c_ptr->C1 |= I2C_C1_IICEN_MASK;
}

void i2c_disable(I2C_Type* i2c_ptr){
	i2c_ptr->C1 &= ~I2C_C1_IICEN_MASK;
}

void i2c_enable_IRQ(uint8_t id, I2C_Type* i2c_ptr){
	i2c_ptr->C1 |= I2C_C1_IICIE_MASK;
	//i2c_enable_pin_IRQ(id);
	//i2c_ptr->S |= I2C_S_IICIF_MASK;			// No es necesario, es Read Only
	i2c_enable_start_stop_IRQ(i2c_ptr);		
	NVIC_EnableIRQ(I2C_NVIC[id]);
}

void i2c_set_baud_rate(I2C_Type* i2c_ptr){//}, uint32_t baud_rate){
	i2c_ptr->F = I2C_F_ICR(ICR) | I2C_F_MULT(MUL);
}

void i2c_enable_start_stop_IRQ(I2C_Type* i2c_ptr){
	i2c_ptr->FLT |= I2C_FLT_SSIE_MASK;
}

void i2c_disable_start_stop_IRQ(I2C_Type* i2c_ptr){
	if ((i2c_ptr->FLT & I2C_FLT_STOPF_MASK) == I2C_FLT_STOPF_MASK){		// If a STOP was sent:
		i2c_ptr->FLT |= I2C_FLT_STOPF_MASK;				// Clear STOPF bit by writing 1 into it
	} else if ((i2c_ptr->FLT & I2C_FLT_STARTF_MASK) == I2C_FLT_STARTF_MASK){
		i2c_ptr->FLT |= I2C_FLT_STARTF_MASK;			// Clear STARTF bit by writing 1 into it
	}
	i2c_ptr->FLT &= ~I2C_FLT_SSIE_MASK;
}

__ISR__ I2C0_IRQHandler(void) {
    I2C_ISR(0);
}

__ISR__ I2C1_IRQHandler(void) {
    I2C_ISR(1);
}

__ISR__ I2C2_IRQHandler(void) {
    I2C_ISR(2);
}
