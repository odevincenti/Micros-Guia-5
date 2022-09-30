/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _BOARD_H_
#define _BOARD_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "gpio.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/***** BOARD defines **********************************************************/

// On Board User LEDs
#define PIN_LED_RED     PORTNUM2PIN(PB, 22)    // PTB22
#define PIN_LED_GREEN   PORTNUM2PIN(PE, 26)    // PTE26
#define PIN_LED_BLUE    PORTNUM2PIN(PB, 21)    // PTB21

#define LED_ACTIVE      LOW
#define LED_DESACTIVE   HIGH



// On Board User Switches
#define PIN_SW2         PORTNUM2PIN(PC,6)     // PTC6
#define PIN_SW3         PORTNUM2PIN(PA,4)     // PTA4
#define MY_SW         	PORTNUM2PIN(PC,0)     // PTC0

#define SW_ACTIVE       HIGH
#define SW_INPUT_TYPE   // ???



// PINES DIGITALES
// FRDM

#define	PTA0		PORTNUM2PIN(PA, 0)
#define	PTA1		PORTNUM2PIN(PA, 1)
#define	PTA2		PORTNUM2PIN(PA, 2)

#define	PTB2		PORTNUM2PIN(PB, 2)
#define	PTB3		PORTNUM2PIN(PB, 3)
#define	PTB9		PORTNUM2PIN(PB, 9)
#define	PTB10		PORTNUM2PIN(PB, 10)
#define	PTB11		PORTNUM2PIN(PB, 11)
#define	PTB18		PORTNUM2PIN(PB, 18)
#define	PTB19		PORTNUM2PIN(PB, 19)
#define	PTB20		PORTNUM2PIN(PB, 20)
#define	PTB23		PORTNUM2PIN(PB, 23)

#define	PTC0		PORTNUM2PIN(PC, 0)
#define	PTC1		PORTNUM2PIN(PC, 1)
#define	PTC2		PORTNUM2PIN(PC, 2)
#define	PTC3		PORTNUM2PIN(PC, 3)
#define	PTC4		PORTNUM2PIN(PC, 4)
#define	PTC5		PORTNUM2PIN(PC, 5)
#define	PTC7		PORTNUM2PIN(PC, 7)
#define	PTC8		PORTNUM2PIN(PC, 8)
#define	PTC9		PORTNUM2PIN(PC, 9)
#define	PTC10		PORTNUM2PIN(PC, 10)
#define	PTC11		PORTNUM2PIN(PC, 11)
#define	PTC16		PORTNUM2PIN(PC, 16)
#define	PTC17		PORTNUM2PIN(PC, 17)

#define	PTD0		PORTNUM2PIN(PD, 0)
#define	PTD1		PORTNUM2PIN(PD, 1)
#define	PTD2		PORTNUM2PIN(PD, 2)
#define	PTD3		PORTNUM2PIN(PD, 3)

#define	PTE24		PORTNUM2PIN(PE, 24)
#define	PTE25		PORTNUM2PIN(PE, 25)
#define	PTE26		PORTNUM2PIN(PE, 26)


#define DIO_1           PORTNUM2PIN(PC, 3)	    // PTC3
#define DIO_2           PORTNUM2PIN(PC, 2)	    // PTC2
#define DIO_3           PORTNUM2PIN(PA, 2)	    // PTA2
#define DIO_4           PORTNUM2PIN(PB, 23)	    // PTB23
#define DIO_5           PORTNUM2PIN(PB, 18)	    // PTB18
#define DIO_6           PORTNUM2PIN(PB, 9)	    // PTB9
#define DIO_7           PORTNUM2PIN(PC, 17)	    // PTC17
#define DIO_8           PORTNUM2PIN(PC, 16)	    // PTC16
#define DIO_9           PORTNUM2PIN(PC, 7)	    // PTC7
#define DIO_10          PORTNUM2PIN(PC, 5)	    // PTC5
#define DIO_11          PORTNUM2PIN(PC, 0)	    // PTC0
#define DIO_12          PORTNUM2PIN(PC, 9)	    // PTC9
#define DIO_13          PORTNUM2PIN(PC, 8)	    // PTC8
#define DIO_14          PORTNUM2PIN(PC, 1)	    // PTC1
#define DIO_15          PORTNUM2PIN(PB, 19)	    // PTB19


#define DIO_16          PORTNUM2PIN(PB, 11)	    // PTB11
#define DIO_17          PORTNUM2PIN(PC, 11)	    // PTC11
#define DIO_18          PORTNUM2PIN(PC, 10)	    // PTC10

// TEST PIN
#define DIO_19          PORTNUM2PIN(PB, 10)	    // PTC10


// UARTS
// TX
#define DIO_20			PORTNUM2PIN(PA, 2)
#define DIO_21			PORTNUM2PIN(PC, 4)
#define DIO_22			PORTNUM2PIN(PD, 3)
#define DIO_23			PORTNUM2PIN(PC, 17)
#define DIO_24			PORTNUM2PIN(PE, 24)

// RX
#define DIO_25			PORTNUM2PIN(PA, 3)
#define DIO_26			PORTNUM2PIN(PC, 3)
#define DIO_27			PORTNUM2PIN(PD, 2)
#define DIO_28			PORTNUM2PIN(PC, 16)
#define DIO_29			PORTNUM2PIN(PE, 25)

/*******************************************************************************
 ******************************************************************************/

#endif // _BOARD_H_
