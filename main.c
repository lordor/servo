/*
 * main.c
 *
 *  Created on: Nov 22, 2013
 *      Author: mzun
 */


#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL	// Clock Speed
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

volatile unsigned char data = 0x00;
volatile unsigned char firstByte = 0x00;
volatile unsigned char secondByte = 0x00;
volatile unsigned int counter = 0; // indicates which servo's PWM on each port to set next
volatile unsigned int timerCompareOrder = 0;
volatile unsigned int mode = 0;
volatile unsigned int mask[] = {11, 10, 9, 8, 15, 14, 13, 12, 3, 2, 1, 0, 7, 6, 5, 4, 20, 21, 22, 23, 16, 17, 18, 19, 24, 25, 26, 27, 28, 29, 30, 31};

const int topServoLimit = 574;
const int bottomServoLimit = 124;

volatile unsigned int currentPositions[24]; // stores current servos positions
volatile unsigned int minPositions[24]; // stores min servos positions, initialized during program start-up
volatile unsigned int maxPositions[24]; // stores max servos positions, initialized during program start-up


void InitUSART(void)
{
// Set baud rate
	UBRR0H = (unsigned char)(BAUD_PRESCALE>>8);
	UBRR0L = (unsigned char)BAUD_PRESCALE;

// Enable receiver
	UCSR0C = (1<<URSEL0)|(1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B = (1<<RXEN0)|(1<<RXCIE0)|(1<<TXEN0);
}

void USART_send(uint8_t data){
  
  while(!(UCSR0A & (1<<UDRE0))){
  }
  
  UDR0 = data;
}

void InitTimers(void)
{
	// timer 2
   TCCR2 = _BV(WGM21) | _BV(CS22) | _BV(CS21);
   OCR2 = 0x9B;

	// timer 1
   TCCR1B = _BV(WGM12);
   OCR1A = 0x00;
   OCR1B = 0x00;

	// timer 3
   TCCR3B = _BV(WGM32);
   OCR3A = 0x00;
}

void InitPositionRanges(void)
{
	int i = 0;
	for(;i < 24; i++)
	{
		minPositions[i] = bottomServoLimit;
		maxPositions[i] = topServoLimit;
		currentPositions[i] = 300;
	}
}

void InitInterruptSettings(void)
{
	TIMSK = _BV(OCIE1A) | _BV(OCIE1B) | _BV(OCIE2);
	ETIMSK = _BV(OCIE3A);
	TIFR = _BV(OCF2);
	sei();
}

void SetPinValue(char port, int portNumber)
{
	unsigned char volatile setPort[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; // index indicates which port value you want to set

	switch(port)
	{
		case 'A':
			PORTA = setPort[portNumber];
		break;
		case 'B':
			PORTB = setPort[portNumber];
		break;
		case 'C':
			PORTC = setPort[portNumber];
		break;
		case 'D':
			PORTD = setPort[portNumber];
		break;
	}
}

void SetOCRs(int counter)
{
	if(timerCompareOrder == 0)
	{
		OCR1A = currentPositions[counter];
		OCR1B = currentPositions[counter + 8];
	}
	else
	{
		OCR1B = currentPositions[counter];
		OCR1A = currentPositions[counter + 8];
	}
	OCR3A = currentPositions[counter + 16];
}

void ProcessBytes(void)
{
	short controlBits = firstByte >> 5;
	short servoNumber = firstByte - (controlBits << 5);
	servoNumber = mask[servoNumber];
	int position = 0;
	switch(servoNumber)
	{
		case 31:
			mode = 1;
			break;
		case 30:
			mode = 0;
			break;
		default:
			switch(controlBits)
			{
				case 5:
					if(mode)
					{
						minPositions[servoNumber] = currentPositions[servoNumber];
					}
					else
					{
						minPositions[servoNumber] = bottomServoLimit + secondByte*3.54;
					}
					break;
				case 6:
					if(mode)
					{
						maxPositions[servoNumber] = currentPositions[servoNumber];
					}
					else
					{
						maxPositions[servoNumber] = bottomServoLimit + secondByte*3.54;
					}
					break;
				case 7:
					position = bottomServoLimit + secondByte*3.15;
					currentPositions[servoNumber] = position;
					break;
				default:
					position = minPositions[servoNumber] + secondByte*(maxPositions[servoNumber] - minPositions[servoNumber])/127;
					currentPositions[servoNumber] = position;
			}
	}
	firstByte = 0x00;
	secondByte = 0x00;
}

// timer 2 interrupts
ISR(TIMER2_COMP_vect)
{
	if(counter == 8)
	{
		counter = 0;
	}

	if(currentPositions[counter] > currentPositions[counter + 8])
	{
		timerCompareOrder = 0;
	}
	else
	{
		timerCompareOrder = 1;
	}

	SetOCRs(counter);

	TCNT1 = 0;
	TCNT3 = 0;
	if(mode)
	{
		SetPinValue('A', counter);
		SetPinValue('B', counter);
		TCCR1B=(1<<CS11)|(1<<CS10);
		SetPinValue('C', counter);
		TCCR3B=(1<<CS31)|(1<<CS30);
	}

	counter++;
}

// timer 1 interrupts
ISR(TIMER1_COMPA_vect)
{
	if(timerCompareOrder == 0)
	{
		PORTA = 0x00;
	}
	else
	{
		PORTB = 0x00;
	}
	TCCR1B=(0<<CS11)|(0<<CS10);
}

ISR(TIMER1_COMPB_vect)
{
	if(timerCompareOrder == 1)
	{
		PORTA = 0x00;
	}
	else
	{
		PORTB = 0x00;
	}
}

// timer 3 interrupts
ISR(TIMER3_COMPA_vect)
{
	TCCR3B=(0<<CS31)|(0<<CS30);
	PORTC = 0x00;
}


ISR(USART0_RXC_vect)
{
	while ( !(UCSR0A & (1<<RXC0)) ){
	}
//	if(UPE0)
//	{
		data=UDR0;
		if(data < 128)
		{
			secondByte = data;
			if(firstByte > 0)
			{
				ProcessBytes();
			}
		}
		else
		{
			firstByte = data;
		}
		USART_send(data);
//	}
	
}

void main( void )
{
	InitPositionRanges();
	InitTimers();
	InitInterruptSettings();
	InitUSART();

	DDRA = 0xFF;
	DDRB = 0xFF;
	DDRC = 0xFF;

	while(1)
	{
		while((_SFR_BYTE(UCSR0A) & _BV(RXC0))==0);
	}
}

