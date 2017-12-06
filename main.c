/*
 * eANTS_main.c
 *
 * Created: 06/12/2017 20:19:21
 * Author : lewis
 */ 

 //Clock speed of 16MHz
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

//Setup USART communication parameter
#define BAUD 4800
#define MYUBRR F_CPU/16/BAUD-1

#include <avr/io.h>
#include <avr/delay.h>

//IO pins
//unsigned const int LED_PIN = PORTB5;
unsigned const int CARRY_PIN = PORTB1;
//unsigned const int RX_ENABLE_PIN = PORTD3;

const int RXPIN =		PORTD0;
const int TXPIN =		PORTD1;
const int REDPIN =		PORTD3;
const int GREENPIN =	PORTD4;
const int BLUEPIN =		PORTD5;
const int BATTERYPIN =	PORTD6;
const int CARRYPIN =	PORTB1;

//Colours for the RGB LED
const int LIGHTBLUE =	0b111;
const int LIGHTGREEN =	0b011;
const int DARKGREEN =	0b010;
const int PURPLE =		0b101;
const int TURQUOISE =	0b110;
const int RED =			0b001;
const int DARKBLUE =	0b100;
const int OFFCOLOUR =	0b000;

//Communication
const unsigned char BEGIN_BYTE = 0b11001001;
const unsigned char END_BYTE = 0b11001010;
//Addresses
const unsigned char ALL_ROBOTS_BYTE = 11001011;
const unsigned char ADDRESS_GROUP1_BYTE = 0b11001100;
const unsigned char ADDRESS_GROUP2_BYTE = 0b11001101;
const unsigned char ADDRESS_GROUP3_BYTE = 0b11001110;
//Programming
const unsigned char BOOTLOADER_BYTE = 0b11001111;
//Control
const unsigned char HIBERNATE_BYTE = 0b11010000;
const unsigned char CHANGE_ID_BYTE = 0b11010001;

const unsigned char ENABLE_MANUAL_BYTE = 0b1010010;  
const unsigned char DISABLE_MANUAL_BYTE = 0b11010011;
const unsigned char MOVE_FORWARDS_BYTE = 0b11010100;
const unsigned char MOVE_BACKWARDS_BYTE = 0b11010101;
const unsigned char STOP_BYTE = 0b11010110;
const unsigned char ROTATE_RIGHT_BYTE = 0b11010111;
const unsigned char ROTATE_LEFT_BYTE = 0b11011000;

const unsigned char COMMAND1_BYTE = 0b11011001;
const unsigned char COMMAND2_BYTE = 0b11011010;
const unsigned char COMMAND3_BYTE = 0b11011011;

const unsigned char SENSE1_BYTE = 0b11011100;
const unsigned char SENSE2_BYTE = 0b11011101;
const unsigned char SENSE3_BYTE = 0b11011110;
//Requests
const unsigned char STATUS1_BYTE = 0b11011111;
const unsigned char STATUS2_BYTE = 0b11100000;
const unsigned char STATUS3_BYTE = 0b11100001;
const unsigned char CHARGE_BYTE = 0b11100010;
const unsigned char SPEED_BYTE = 0b11100011;
const unsigned char ORIENTATION_BYTE = 0b11100100;
const unsigned char REQUEST_DATA = 0b11100101;


//ERROR
const unsigned char MESSAGE_ERROR_BYTE = 0b11111111;

//This is robot #1
const unsigned char ADDRESS = 0b00000001; 

unsigned char charge = 255;
unsigned char speed = 0;
unsigned char orientation = 128;
unsigned char status1 = 120;
unsigned char status2 = 220;
unsigned char status3 = 35;

//Address group memberships
char address_group1 = 0;
char address_group2 = 0;
char address_group3 = 0;
char address_group4 = 0;
char address_group5 = 0;

//Maximum number of requests/commands at once
unsigned const MAX_REQUESTS = 6;
unsigned const MAX_COMMANDS = 5;

//The values set for the sensing
/*Sense 1 is for sensing the nearest. Its data is encoding according to the following pattern
*1. Robot address
*2. Robot status 1
*3. Robot status 2
*4. Robot status 3
*/
unsigned int sense1[4]; //Space for closest robot
unsigned int sense2 = 0;
unsigned int sense3 = 0;
//Flags telling if they have been recently set
int sense1Set = 0;
int sense2Set = 0;
int sense3Set = 0;

//Other flags for control requests
short manualModeSet = 0;
int moveForwardsEnabled = 0;
int moveBackwardsEnabled = 0;
int rotateSet = 0;
int command1Set = 0;
int command2Set = 0;
int command3Set = 0;
//The values for the controlling
int moveSpeed = 0;
int rotateAngle = 0;


//Timeout time for receiving requests/etc.
unsigned const long TIMEOUT = F_CPU/500; // 1/500 second 

//boolean values defining if a specific value has been requested
int request1 = 0;
int request2 = 0;
int request3 = 0;
int request4 = 0;
int request5 = 0;
int request6 = 0;

//unsigned long transmitTimeout = 0; //Time to wait after transmitting before receiving data

/************************************************************************/
/* Creates a PWM carrier signal on PORTB1/ Arduino pin ~9
/************************************************************************/
void INIT_CARRIER(){

	//Carry signal pin
	DDRB |= (1 << CARRY_PIN);

	//Frequency and duty cycle
	ICR1 = 26;
	OCR1A = 13;

	TCCR1A =
	// Compare output mode:
	(1 << COM1A1) | (0 << COM1A0) |
	// lower 2 WGM bits:
	(0 << WGM11) | (0 << WGM10);

	TCCR1B =
	// upper 2 WGM bits:
	(1 << WGM13) | (0 << WGM12) |

	// Prescaler/start timer:
	(0 << CS12) | (1 << CS11 ) | (0 << CS10);

}

/************************************************************************/
/* Initialize serial communication                                      */
/************************************************************************/
void USART_Init( unsigned int ubrr){
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}/************************************************************************/
/* Transmit characters over the serial connection                       */
/************************************************************************/void USART_Transmit( unsigned char data ){
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
	}/************************************************************************/
/* Receive characters from the serial connection                         */
/************************************************************************/unsigned char USART_Receive( void ){
	uint32_t i = 500000;  //timeout of approx. F_CPU/5e6 = 1/32 sec = 30 ms
	do {
		if(UCSR0A & (1<<RXC0)){
			return UDR0;
		}
	} while (--i);
	return MESSAGE_ERROR_BYTE;
	
}/************************************************************************/
/* Deals with incoming messages/************************************************************************/void HANDLE_MESSAGE( unsigned char address){		if (address == ADDRESS||(address_group1 == 1 && address == ADDRESS_GROUP1_BYTE) ||
	(address_group2 == 1 && address == ADDRESS_GROUP2_BYTE) ||
	(address_group3 == 1 && address == ADDRESS_GROUP3_BYTE) ||
	address == ALL_ROBOTS_BYTE){
		
		SET_COLOUR(LIGHTGREEN);

		//PORTB |= (1 << LED_PIN);
		//Receive next message
		unsigned char message = USART_Receive();
		
		//Loop until all robot have been addresses
		while(message!= BEGIN_BYTE && message != MESSAGE_ERROR_BYTE && message != END_BYTE){
			message = USART_Receive();
		}
		if (message == BEGIN_BYTE){
			message = USART_Receive();
		}
		//This code runs, once the robot is being talked to or ends if it isn't

		int request1 = 0;
		int request2 = 0;
		int request3 = 0;
		int request4 = 0;
		int request5 = 0;
		int request6 = 0;

		while(message != MESSAGE_ERROR_BYTE && message != END_BYTE){
			
			SET_COLOUR(LIGHTGREEN);
			//////////////////////////////////////////////////////////////////////////
			//Control requests

			if (message == SENSE1_BYTE){
				sense1[0] = message;
				int tempSenseIndex = 1;
				for(int i = 1;i < 4;i++){
					message = USART_Receive();
					sense1[i] = message;
					if(message = MESSAGE_ERROR_BYTE)break;
				}
				if(message != MESSAGE_ERROR_BYTE)sense1Set = 1;
				}else if (message == SENSE2_BYTE){
				message = USART_Receive();
				if (message != MESSAGE_ERROR_BYTE){
					sense2 = message;
					sense2Set = 1;
				}
				}else if (message == SENSE3_BYTE){
				message = USART_Receive();
				if (message != MESSAGE_ERROR_BYTE){
					sense3 = message;
					sense3Set = 1;
				}
				}else if (message == ENABLE_MANUAL_BYTE){
				manualModeSet = 1;
				}else if (message == DISABLE_MANUAL_BYTE){
				manualModeSet = 0;
				}else if(message == MOVE_FORWARDS_BYTE){
				message = USART_Receive();
				if (message != MESSAGE_ERROR_BYTE){
					moveSpeed = message;
					moveForwardsEnabled = 1;
					moveBackwardsEnabled = 0;
					rotateSet = 0;
				}
				}else if(message == MOVE_BACKWARDS_BYTE){
				message = USART_Receive();
				if (message != MESSAGE_ERROR_BYTE){
					moveSpeed = message;
					moveForwardsEnabled = 1;
					moveBackwardsEnabled = 0;
					rotateSet = 0;
				}
				}else if (message == ROTATE_RIGHT_BYTE || message ==  ROTATE_LEFT_BYTE){
				int direction = message;
				message = USART_Receive();
				if (message != MESSAGE_ERROR_BYTE){
					if (direction = ROTATE_RIGHT_BYTE){
						rotateAngle = message;
						}else{
						rotateAngle = message*-1;
					}
					rotateSet = 1;
					moveForwardsEnabled = 0;
					moveBackwardsEnabled = 0;
				}
				}else if(message == COMMAND1_BYTE){
				command1Set = 1;
				}else if(message == COMMAND2_BYTE){
				command2Set = 1;
				}else if(message == COMMAND3_BYTE){
				command3Set = 1;
			}
			//////////////////////////////////////////////////////////////////////////
			//End Control requests

			//////////////////////////////////////////////////////////////////////////
			//Special requests

			if(message == BOOTLOADER_BYTE){
				//enter boot loader
				}else if (message == HIBERNATE_BYTE){
				//Hibernate
				}else if(message == CHANGE_ID_BYTE){
				//Change id
				unsigned char newId = USART_Receive();
			}
			//////////////////////////////////////////////////////////////////////////
			//End Special requests

			//////////////////////////////////////////////////////////////////////////
			//Information requests
			//step 1: register all the sent requests
			
			if(message == STATUS1_BYTE){
				request1 = 1;
				}else if (message == STATUS2_BYTE){
				request2 = 1;
				}else if (message == STATUS3_BYTE){
				request3 = 1;
				}else if (message == CHARGE_BYTE){
				request4 = 1;
				}else if (message == SPEED_BYTE){
				request5 = 1;
				}else if (message == ORIENTATION_BYTE){
				request6 = 1;
			}
			if (message == REQUEST_DATA){
				//step 2:send all the requested fields, stored in array
				_delay_ms(10);
				USART_Transmit(address);
				USART_Transmit(BEGIN_BYTE);
				if (request1 == 1){
					USART_Transmit(STATUS1_BYTE);
					USART_Transmit(status1);
					//PORTB = 0xFF;
				}
				if (request2 == 1){
					USART_Transmit(STATUS2_BYTE);
					USART_Transmit(status2);
				}
				if (request3 == 1){
					USART_Transmit(STATUS3_BYTE);
					USART_Transmit(status3);
				}
				if (request4 == 1){
					USART_Transmit(CHARGE_BYTE);
					USART_Transmit(charge);
				}
				if (request5 == 1){
					USART_Transmit(SPEED_BYTE);
					USART_Transmit(speed);
				}
				if (request6 == 1){
					USART_Transmit(ORIENTATION_BYTE);
					USART_Transmit(orientation);
				}
				USART_Transmit(END_BYTE);

				
			}
			//////////////////////////////////////////////////////////////////////////
			//End information requests

			//If the communication is ended, fulfill all the requests and updates
			if (message == END_BYTE || message == MESSAGE_ERROR_BYTE){
				message = MESSAGE_ERROR_BYTE;
				if (sense1Set == 1){
					sense1Set = 0;
				}
				if (sense2Set == 1){
					sense2Set = 0;
				}
				if (sense3Set == 1){
					sense3Set = 0;
				}
				if (manualModeSet == 1){
					if (rotateSet){
						rotateSet = 0;
						}else if (moveForwardsEnabled){
						moveForwardsEnabled = 0;
						}else if (moveBackwardsEnabled){
						moveBackwardsEnabled = 0;
					}
				}
				if (command1Set == 1){
					command1Set = 0;
				}
				if (command2Set == 1){
					command2Set = 0;
				}
				if (command3Set == 1){
					command3Set = 0;
				}
			}else{
				//read the next message:
				message = USART_Receive();
			}
			
		}

	}

}//End main while loop

/*
void enableRX(){
	DDRD |= (1 << RX_ENABLE_PIN);
}

void disableRX(){
	DDRD &= ~(1 << RX_ENABLE_PIN);
	PORTD |= (1 << RX_ENABLE_PIN);
}
*//************************************************************************/
/*Sets up the IO on the micro controller             
/************************************************************************/void SETUP_IO(){

	//RGB LED
	DDRD |= (1 << REDPIN);
	DDRD |= (1 << GREENPIN);
	DDRD |= (1 << BLUEPIN);
	
	/*
	//STEPPER B
	DDRC |= (1 << B1PIN);
	DDRC |= (1 << B2PIN);
	DDRC |= (1 << B3PIN);
	DDRC |= (1 << B4PIN);
	PORTC &= ~(1 << B1PIN);
	PORTC &= ~(1 << B2PIN);
	PORTC &= ~(1 << B3PIN);
	PORTC &= ~(1 << B4PIN);
	*/
	
}/************************************************************************/
/* Sets the RGB LED to the desired colour
/************************************************************************///red, green, blue
void SET_COLOUR(int colour){
	if ((1 << 0) & colour){
		PORTD |= (1 << REDPIN);
		}else{
		PORTD &= ~(1 << REDPIN);
	}
	if ((1 << 1) & colour){
		PORTD |= (1 << GREENPIN);
		}else{
		PORTD &= ~(1 << GREENPIN);
	}
	if ((1 << 2) & colour){
		PORTD |= (1 << BLUEPIN);
		}else{
		PORTD &= ~(1 << BLUEPIN);
	}
}

/************************************************************************/
/* The main void responsible for receiving data and responding accordingly
/************************************************************************/

void main( void ){

	//Set port B as output
	//DDRB = 0xFF;
	//DDRB |= (1 << LED_PIN);
	//Set led to off
	//PORTB &= ~(1 << LED_PIN);
	//RX Enable pin
	//DDRD &= ~(1 << RX_ENABLE_PIN);
	//PORTD &= ~(1 << RX_ENABLE_PIN);

	//Initialize USART
	USART_Init(MYUBRR);
	INIT_CARRIER();

	SETUP_IO();
	SET_COLOUR(RED);

	//disableRX();

	while(1){

		unsigned char address = USART_Receive();
		//Check if robot is being addressed
		HANDLE_MESSAGE(address);

		
	}
}//End main function

