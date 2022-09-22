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

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/
static uint8_t I2C_state[] = {I2C_IDLE, I2C_IDLE, I2C_IDLE};
static bool I2C_init[] = {false, false, false};
static uint8_t I2C_write_fifo[] = {FIFO_INVALID_ID, FIFO_INVALID_ID, FIFO_INVALID_ID};
static uint8_t I2C_read_fifo[] = {FIFO_INVALID_ID, FIFO_INVALID_ID, FIFO_INVALID_ID};
static bool I2C_rsta[] = {false, false, false};
static uint8_t I2C_start_count[] = {0, 0, 0};
static uint8_t I2C_rsta_address[] = {0, 0, 0};

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

void I2C_StartTransaction(uint8_t id, uint8_t address, bool mode, bool rsta){
	if (i2c_is_bus_busy(I2C_ptrs[id])){				// Si el bus está libre (SÓLO MULTIMASTER)
		i2c_send_start_signal(I2C_ptrs[id]);		// Start
		i2c_set_TX(I2C_ptrs[id]);					// TX mode
		i2c_write_to_data_register(I2C_ptrs[id], (address << 1) | mode);	// Write address
		I2C_state[id] = (mode == I2C_READ) ? I2C_FAKE_READ : I2C_WRITE;		// Set next state
		if (rsta){
			//TODO: rsta
		}
	}

}

bool I2C_LoadBuffer(uint8_t id, bool mode){
	// TODO: Load write or read buffer
	// Instruction buffer?
}

void I2C_IRQ(uint8_t id){
	if (i2c_interrupt_pending(I2C_ptrs[id])){
		if(i2c_did_bus_stop(I2C_ptrs[id])){
			I2C_start_count[id] = 0;
		} else if (i2c_did_bus_start(I2C_ptrs[id])){
			I2C_start_count[id]++;
			if (I2C_start_count[id] > 1){

			}
		}
		i2c_disable_interrupt_flag(I2C_ptrs[id]);
		I2C_FSM(id);
	}
	// leo ack
	// si nack -> termino y manda stop bit
	// si ack -> sigo
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
					if(FIFO_IsBufferEmpty(I2C_write_fifo[id])){			// If buffer empty -> Done writing:
						i2c_send_stop_signal(I2C_write_fifo[id]);		// Stop
						// TODO: STATE IDLE
					}
					// stop?
					// next state?
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
			FIFO_PushToBuffer(I2C_read_fifo[id], i2c_read_data_register(I2C_ptrs[id]));
			// stop?
			// next_state?
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

	case I2C_START:

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