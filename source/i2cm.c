/***************************************************************************//**
  @file     i2c.c
  @brief    I2C driver
  @author   Olivia De Vincenti
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "fifo.h"
#include "i2c_K64.h"
#include "i2cm.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define I2C_MAX_TRANS_BUFFER	16
#define ERR_ADDRESS		0x0

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
enum {I2C_WRITE, I2C_READ, I2C_FAKE_READ, I2C_START, I2C_REPEATED_START, I2C_IDLE};

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/
const i2c_transaction_t i2c_empty_trans = {.mode = true, .address = ERR_ADDRESS, .ptr = 0, .count = 0, .next_rsta = false};

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/
static uint8_t I2C_state[] = {I2C_IDLE, I2C_IDLE, I2C_IDLE};
static bool I2C_init[] = {false, false, false};
static uint8_t I2C_write_fifo[] = {FIFO_INVALID_ID, FIFO_INVALID_ID, FIFO_INVALID_ID};
static uint8_t I2C_read_fifo[] = {FIFO_INVALID_ID, FIFO_INVALID_ID, FIFO_INVALID_ID};
static uint8_t I2C_start_count[] = {0, 0, 0};

static i2c_transaction_t i2c_trans[I2C_N][I2C_MAX_TRANS_BUFFER];
static i2c_transaction_t* i2c_trans_head[I2C_N];
static i2c_transaction_t* i2c_trans_tail[I2C_N];

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void I2C_Init(uint8_t id){
	
	if(!I2C_init[id] && id < I2C_N){

		// Request FIFO
		I2C_write_fifo[id] = FIFO_GetId();
		I2C_read_fifo[id] = FIFO_GetId();

		// Init I2C
		I2C_Type* i2c_ptr = I2C_ptrs[id];
		i2c_enable_clock_gating(id);
		i2c_enable_pins(id);
		i2c_disable(i2c_ptr);
		i2c_set_baud_rate(i2c_ptr);
		i2c_set_7_bit_address(i2c_ptr);

		// Habilito interrupciones
		i2c_enable_pin_IRQ(id);
		i2c_enable_IRQ(id, i2c_ptr);

		i2c_enable(i2c_ptr);
		I2C_init[id] = true;
	}
}

bool I2C_push_transaction(uint8_t id, i2c_transaction_t trans){
	if ( i2c_trans_head[id] + 1 != i2c_trans_tail[id]){			// If buffer not empty
		*(i2c_trans_head[id]) = trans;							// Push 
		if ( i2c_trans_head[id] + 1 != &i2c_trans[id][0] + I2C_MAX_TRANS_BUFFER){	// If not at the end:
			i2c_trans_head[id]++;							// Inc head
		} else {											// If at the end:
			i2c_trans_head[id] = &i2c_trans[id][0];			// Reset head
		}
		return true;		// All ok
	} else {
		return false;		// ERR: Transaction buffer full
	}
}

i2c_transaction_t* I2C_pull_transaction(uint8_t id){
	i2c_transaction_t* r;
	if ( i2c_trans_head[id] != i2c_trans_tail[id] + 1){
		if ( i2c_trans_tail[id] + 1 != &i2c_trans[id][0] + I2C_MAX_TRANS_BUFFER){	// If not at the end:
			i2c_trans_tail[id]++;							// Inc tail
		} else {											// If at the end:
			i2c_trans_tail[id] = &i2c_trans[id][0];	// Reset tail
		}
		r = i2c_trans_tail[id];
	} else {
		r = &i2c_empty_trans;
	}
	return r;
}

void I2C_StartTransaction(uint8_t id){
	if (!i2c_is_bus_busy(I2C_ptrs[id])){				// Si el bus está libre
		i2c_transaction_t* trans = I2C_pull_transaction(id);
		if (trans->address != 0x0){					// If there are transactions to make
			if (trans->mode == I2C_WRITE){		
				if (FIFO_MAX_N - 1 - FIFO_GetBufferLength(I2C_write_fifo) < trans->count){			// If write buffer full. Error:
					// Retreat trans buffer
					i2c_trans_tail[id] = (i2c_trans_tail[id] - 1 != &i2c_trans[id][0]) ? i2c_trans_tail[id]-- : &i2c_trans[id][I2C_MAX_TRANS_BUFFER - 1];
					I2C_state[id] = I2C_IDLE;		// Set next state
					return;
				} else {
					FIFO_WriteToBuffer(I2C_write_fifo[id], trans->ptr, trans->count);		//Save data to transmit.
				}
			}
			i2c_send_start_signal(I2C_ptrs[id]);		// Start
			i2c_set_TX(I2C_ptrs[id]);					// TX mode
			i2c_write_to_data_register(I2C_ptrs[id], (trans->address << 1) | trans->mode);	// Write address
			I2C_state[id] = (trans->mode == I2C_READ) ? I2C_FAKE_READ : I2C_WRITE;		// Set next state
		}
	} else{
		if (i2c_trans_tail[id]->next_rsta){
			i2c_transaction_t* trans = I2C_pull_transaction(id);
			if (trans->address != 0x0){					// If there are transactions to make
				// TODO: Add W mode
				i2c_repeated_start(I2C_ptrs[id]);		// Repeated start
				i2c_set_TX(I2C_ptrs[id]);				// TX mode
				i2c_write_to_data_register(I2C_ptrs[id], (trans->address << 1) | I2C_READ);	// Write address
				I2C_state[id] = I2C_FAKE_READ;			// Set next state
			} else {
				// TODO: Error, decía que iba a haber rsta y nada
			}
		}
	}
}

void I2C_IRQ(uint8_t id){
	if (i2c_interrupt_pending(I2C_ptrs[id])){
		if(i2c_did_bus_stop(I2C_ptrs[id])){
		} else if (i2c_did_bus_start(I2C_ptrs[id])){}
		i2c_disable_interrupt_flag(I2C_ptrs[id]);
		I2C_FSM(id);
	}
}

void I2C_FSM(uint8_t id){

	uint8_t data;

	switch(I2C_state[id]){

	case I2C_WRITE:
		if (i2c_is_transfer_complete(I2C_ptrs[id])){
			if (i2c_was_ack(I2C_ptrs[id])){					// If ACK:
				i2c_set_TX(I2C_ptrs[id]);
				if(!FIFO_PullFromBuffer(I2C_write_fifo[id], &data)){	// Pull from buffer, if pull ok:
					i2c_write_to_data_register(I2C_ptrs[id], data);		// Write to data
					if(!(--i2c_trans_tail[id]->count)){					// If count is 0 -> Done writing:
						if (!i2c_trans_tail[id]->next_rsta){			// If not repeated start
							i2c_send_stop_signal(I2C_write_fifo[id]);		// Stop
						}
						I2C_state[id] = I2C_IDLE;
					}
				}											
			} else {											// If NACK:
				i2c_send_stop_signal(I2C_write_fifo[id]);		// Stop
				// TODO: Set error
			}
		} else {
			// TODO: lost arbitration (multimaster)
		}
		break;
	case I2C_READ:
		if (i2c_is_transfer_complete(I2C_ptrs[id])){
			// TODO:chequear si hubo ack de slave (seria error)
			i2c_set_RX(I2C_ptrs[id]);
			i2c_send_ack_signal(I2C_ptrs[id]);
			if(FIFO_PushToBuffer(I2C_read_fifo[id], i2c_read_data_register(I2C_ptrs[id]))){
				if(!(--i2c_trans_tail[id]->count)){					// If count is 0 -> Done reading:
					if (!i2c_trans_tail[id]->next_rsta){			// If not repeated start
						i2c_send_stop_signal(I2C_write_fifo[id]);		// Stop
					}
					I2C_state[id] = I2C_IDLE;
				}
			} // else : error

		} else {
			// lost arbitration (multimaster)
		}
		break;

	case I2C_FAKE_READ:
		if (i2c_is_transfer_complete(I2C_ptrs[id])){
			i2c_set_RX(I2C_ptrs[id]);
			i2c_send_ack_signal(I2C_ptrs[id]);
			i2c_read_data_register(I2C_ptrs[id]);
			I2C_state[id] = I2C_READ;
		}
		break;

	case I2C_IDLE:
		I2C_StartTransaction(id);
		break;

	default:
		break;

	}


}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/******************************************************************************/