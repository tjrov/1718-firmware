//Pin Definitions for ROV firmware

/*****************************************************
To set a pin as output (default is input) do:

	DEBUG_LED_DDR |= (1 << DEBUG_LED);
	
	You basically setting one of the bits in the Data DiRection Register (DDR) to 1

To set as input, no code is needed. Input is the default.

To set a pin high, write a one to one of the bits in the correct Port register

	DEBUG_LED_PORT |= (1 << DEBUG_LED);

To set a pin low, write a zero to one of the bits in the correct Port register

	DEBUG_LED_PORT &= ~(1 << DEBUG_LED);

******************************************************/

#include <avr/io.h>

//DEBUG LED
#define DEBUG_LED PB5
#define DEBUG_LED_PORT PORTB
#define DEBUG_LED_DDR DDRB

//MOTOR 1
#define MOT1_DIR1 PD3
#define MOT1_DIR1_PORT PORTD
#define MOT1_DIR1_DDR DDRD

#define MOT1_DIR2 PC1
#define MOT1_DIR2_PORT PORTC
#define MOT1_DIR2_DDR DDRC

//MOTOR 2
#define MOT2_DIR1 PD5
#define MOT2_DIR1_PORT PORTD
#define MOT2_DIR1_DDR DDRD

#define MOT2_DIR2 PC2
#define MOT2_DIR2_PORT PORTC
#define MOT2_DIR2_DDR DDRC

//MOTOR 3
#define MOT3_DIR1 PD7
#define MOT3_DIR1_PORT PORTD
#define MOT3_DIR1_DDR DDRD

#define MOT3_DIR2 PB0
#define MOT3_DIR2_PORT PORTB
#define MOT3_DIR2_DDR DDRB

//MOTOR 4
#define MOT4_DIR1 PB1
#define MOT4_DIR1_PORT PORTB
#define MOT4_DIR1_DDR DDRB

#define MOT4_DIR2 PD4
#define MOT4_DIR2_PORT PORTD
#define MOT4_DIR2_DDR DDRD

//RELAY 1
#define RLY1_CTRL PC0
#define RLY1_CTRL_PORT PORTC
#define RLY1_CTRL_DDR DDRC

//RELAY 2
#define RLY2_CTRL PD7
#define RLY2_CTRL_PORT PORTD
#define RLY2_CTRL_DDR DDRD

//LED HEADLIGHTS
#define LED_CTRL PC3
#define LED_CTRL_PORT PORTC
#define LED_CTRL_DDR DDRC

//SPEAKER
#define SPK PB2
#define SPK_PORT PORTB
#define SPK_DDR DDR

//RS-485 TRANSMIT ENABLE
#define TX_EN PD2
#define TX_EN_PORT PORTD
#define TX_EN_DDR DDRD
