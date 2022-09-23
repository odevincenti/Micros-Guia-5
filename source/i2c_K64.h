// /***************************************************************************//**
//   @file     i2c.h
//   @brief    I2C MCAL layer
//   @author   Olivia De Vincenti
//  ******************************************************************************/

// #ifndef _I2C_K64_H_
// #define _I2C_K64_H_

// /*******************************************************************************
//  * INCLUDE HEADER FILES
//  ******************************************************************************/

// #include <stdbool.h>
// #include "MK64F12.h"

// #define I2C_N	3

// /*******************************************************************************
//  * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
//  ******************************************************************************/
// typedef void (*I2C_callback_t)(uint8_t id);

// /*******************************************************************************
//  * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
//  ******************************************************************************/
// static I2C_Type* const I2C_ptrs[] = I2C_BASE_PTRS;

// /*******************************************************************************
//  * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
//  ******************************************************************************/

// void i2c_enable_pins(uint8_t id);

// void i2c_enable_pin_IRQ(uint8_t id);

// void i2c_disable_pin_IRQ(uint8_t id);

// void i2c_enable_clock_gating(uint8_t id);

// void i2c_disable_clock_gating(uint8_t id);

// void i2c_enable(I2C_Type* i2c_ptr);

// void i2c_disable(I2C_Type* i2c_ptr);

// void i2c_enable_IRQ(uint8_t id, I2C_Type* i2c_ptr);

// void i2c_disable_IRQ(uint8_t id, I2C_Type* i2c_ptr);

// void i2c_set_baud_rate(I2C_Type* i2c_ptr);//, uint32_t baud_rate);

// void i2c_set_TX(I2C_Type* i2c_ptr);

// void i2c_set_RX(I2C_Type* i2c_ptr);

// bool i2c_repeated_start(I2C_Type* i2c_ptr);

// bool i2c_send_start_signal(I2C_Type* i2c_ptr);

// bool i2c_send_stop_signal(I2C_Type* i2c_ptr);

// void i2c_send_ack_signal(I2C_Type* i2c_ptr);

// bool i2c_was_ack(I2C_Type* i2c_ptr);

// void i2c_write_to_data_register(I2C_Type* i2c_ptr, uint8_t data);

// uint8_t i2c_read_data_register(I2C_Type* i2c_ptr);

// bool i2c_is_transfer_complete(I2C_Type* i2c_ptr);

// // bool i2c_set_no_transfer(I2C_Type* i2c_ptr);

// bool i2c_is_bus_busy(I2C_Type* i2c_ptr);

// bool i2c_was_arbitration_lost(I2C_Type* i2c_ptr);

// bool i2c_interrupt_pending(I2C_Type* i2c_ptr);

// void i2c_disable_interrupt_flag(I2C_Type* i2c_ptr);

// // General call: used to address all slaves at the same time, only allows write operations, not read
// void i2c_enable_general_call(I2C_Type* i2c_ptr);

// void i2c_disable_general_call(I2C_Type* i2c_ptr);

// void i2c_enable_start_stop_IRQ(I2C_Type* i2c_ptr);

// void i2c_disable_start_stop_IRQ(I2C_Type* i2c_ptr);

// bool i2c_did_bus_start(I2C_Type* i2c_ptr);

// bool i2c_did_bus_stop(I2C_Type* i2c_ptr);

// void i2c_set_7_bit_address(I2C_Type* i2c_ptr);

// void i2c_set_10_bit_address(I2C_Type* i2c_ptr);

// // Only available for 10 bit address mode
// void i2c_set_upper_3_address_bits(I2C_Type* i2c_ptr, uint8_t add);

// void i2c_set_normal_drive_mode(I2C_Type* i2c_ptr);

// void i2c_set_high_drive_mode(I2C_Type* i2c_ptr);

// void i2c_set_filter_factor(I2C_Type* i2c_ptr, uint8_t factor);

// /*******************************************************************************
//  ******************************************************************************/

// #endif // _I2C_K64_H_
