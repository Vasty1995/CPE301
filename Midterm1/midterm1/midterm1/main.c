/*
 * midterm1.c
 *
 * Created: 4/11/2018 11:09:31 AM
 * Author : YKengne
 */ 

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#define F_CPU 8000000UL

#define FOSC 16000000			//Frequency
#define BAUD 9600				//Baud Rate
#define MYUBRR FOSC/16/BAUD-1	//Automatic BAUD rate calculation

volatile uint8_t adcValue;

volatile uint8_t fifteenPlus = 120;	//


/*Function Declarations*/
void USART_Init();
void ADC_Init();
void outputChr(unsigned char c);
void outputStr(char *c);
void readTemp();

void AT_Tx(char *t);

ISR(ADC_vect)
{
	ADCSRA	|= (1 << ADIF);	//Reset flag
	adcValue = ADCH;		//MSB 8-bits of ADC form left shift of ADLAR
}


ISR(TIMER1_OVF_vect)
{
	TIFR1 |= (1 << TOV1);	//Clr Flag
	fifteenPlus++;

	
	
}

int main(void)
{
	TCCR1B	|= (1 << CS12) | (1 << CS10);	//Set prescale 1024
	TIMSK1 |= (1 << TOIE1);					//enable OVF interrupt
	
	unsigned int oneFive = 0;
	
	ADC_Init();			//initialize ADC
	USART_Init();		//UART initialization
	
	
	while (1)
	{
		if(fifteenPlus>=120)
		{
			readTemp();
			fifteenPlus = 0;
		}
	}
}


void USART_Init()
{
	
	
	UBRR0H = (MYUBRR>>8);	//Shift MSB "top" of UBRR0H	0100 0100 >> 8 -> UBRR0H 0000 0000
	UBRR0L = MYUBRR;		//UBRR0L 0100 0100
	
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);	 //Enable Rec and Trans
	UCSR0B |= (1 << RXCIE0);				 //Enable Rec INT
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); //Set frame 8-bit, 1 STP
}
void ADC_Init()
{
	DDRC	= 0;			//Set PORTC as input for adc
	DIDR0	= 0x01;			//Disable Digi input on ADC0
	
	ADMUX = 0;				//Sets Mux selection bits to 0 ADC0 used
	ADMUX |= (1 << REFS0);	//Use Vcc Ref voltage selectin 01
	ADMUX |= (1 << ADLAR);	//Left adjust ADC Reg, ADCH 8-bit Resolution
	
	
	ADCSRA |= (1 << ADEN);	//Enable ADC
	ADCSRA |= (1 << ADATE); //Set ADC Auto Trig
	ADCSRA |= (1 << ADIE);	//Enable Interrupts
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0); //8MHz, Pre-Scale 64 = 125KHz
	ADCSRA |= (1 << ADSC);	//Start Conversion
	
	ADCSRB = 0;			//Free running mode
	sei();				//Enable interrupts
	
}

void outputChr(unsigned char c)
{
	UDR0 = c;	//Display Char on Serial
	_delay_ms(800);
}

void outputStr(char *c)
{
	unsigned int i = 0;	//loop control
	while(c[i] != 0)
	outputChr(c[i++]);
}

void readTemp()
{
	char seeTemp[8];
	float	lm34_0;		//For ASCII Temp output
	float	lm34_1;		//For showing valued of ADC
	
	
	while((ADCSRA & (1 << ADIF)) == 0);			//Wait for conversion to finish
	
	/*Conversion to °F*/
	lm34_0 = (adcValue * 5.0 / 0x100) * 100.0;	//(ADC * 5 = 200 /256) * 100
	
	dtostrf(lm34_0, 5, 2, seeTemp);				//Float to char conversion
	
	
	AT_Tx(seeTemp);
	
	
}

void AT_Tx(char *t)
{
	/*Build Strings for AT+ commands*/
	unsigned char CIPStart[]	= "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n";
	unsigned char CIPSend[]		= "AT+CIPSEND=51\r\n";
	unsigned char Data[]		= "GET /update?api_key=X5GNOJ6AFIEO0XRP&field1=";
	unsigned char temp0			= "t";
	unsigned char temp1			= "\r\n";
	unsigned char CIPClose[]	= "AT+CIPCLOSE\r\n";
	
	
	_delay_ms(2000);
	outputStr(CIPStart);	//Send Start String
	
	_delay_ms(2000);
	outputStr(CIPSend);		//Number of Char being sent
	
	_delay_ms(2000);
	outputStr(Data);		//Get command sent
	outputStr(t);			//Temperature string added to end of Get command
	outputStr("\n\r");		//Enter key
	
