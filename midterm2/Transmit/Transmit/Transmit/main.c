/*
 * Transmit.c
 *
 * Created: 5/2/2018 9:20:31 AM
 * Author : YKengne
 */ 


#define F_CPU 16000000UL //16 Mhz.
#define BAUD 9600	//Set Baud rate 9600.
#define MYUBRR F_CPU/16/BAUD-1 //Configuration for MYUBRR.

#include <avr/io.h>		//include necessary libraries/files
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include "nrf24l01.h"
#include "nrf24l01-mnemonics.h"
#include <stdio.h>

void read_adc(void);	//function to read ADC value
void adc_init(void);	//function to initialize the ADC
void USART_init(unsigned int ubrr);	//Function to initialize USART
void USART_tx_string(char *data);	//Function to print the ADC value

void setup_timer(void);		//function to set up a timer interrupt every 1 second
nRF24L01 *setup_rf(void);	//function to set up nRF24L01 chip

volatile bool rf_interrupt = false;
volatile unsigned int adc_temp;		//variable to hold ADC value read from LM34
char outs[20];				//variable to hold ADC value
void USART_init(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<TXEN0);	//Enable receiver, transmitter & RX interrupt.
	UCSR0C = (3<<UCSZ00);  //Asynchronous 8 N 1
}

void USART_Transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0=data;				//set UDR0 = data
}

int main(void)
{
	adc_init();		//initialize ADC
	setup_timer();	//set up a timer1 overflow. Interrupts every 1 second
	USART_init(MYUBRR);	//initialize USART
	USART_tx_string("Connected!\r\n");  //lets us know if we are connected to the visualizer
	_delay_ms(125);		//wait for 125 us
	
	uint8_t to_address[5] = { 0x21, 0x26, 0x32, 0xFF, 0x3E }; //Can pick any combination of hex values so long as it is the same in the receive address.
	bool on = false;
	nRF24L01 *rf = setup_rf();
	
	sei();		//set enable interrupt
	
	while (true) {
		if (rf_interrupt) {
			rf_interrupt = false;
			int success = nRF24L01_transmit_success(rf);
			if (success != 0)
			nRF24L01_flush_transmit_message(rf);
		}
		if (send_message) {
			send_message = false;
			on = !on;
			nRF24L01Message msg;
			if (on) memcpy(msg.data, "ON", 3);
			else memcpy(msg.data, "OFF", 4);
			msg.length = strlen((char *)msg.data) + 1;
			nRF24L01_transmit(rf, to_address, &msg);
		}
	}
	return 0;
}
nRF24L01 *setup_rf(void)
{
	nRF24L01 *rf = nRF24L01_init();

	rf->ss.port = &PORTB;
	rf->ss.pin = PB2		//set ss to PB2
	rf->ce.port = &PORTB;
	rf->ce.pin = PB1;		//set ce to PB1
	rf->sck.port = &PORTB;
	rf->sck.pin = PB5;		//set sck to PB5
	rf->mosi.port = &PORTB;
	rf->mosi.pin = PB3;		//set MOSI to PB3
	rf->miso.port = &PORTB;
	rf->miso.pin = PB4;		//set MISO to PB4
	// interrupt on falling edge of INT0 (PD2)
	EICRA |= _BV(ISC01);
	EIMSK |= _BV(INT0);
	nRF24L01_begin(rf);
	return rf;
}
// setup timer to trigger interrupt every second when at 1MHz
void setup_timer(void)
{
	TIMSK1 |= (1<<TOIE1);			//enable Timer1 overflow interrupt
	TCCR1B |= (1<<CS12) | (1<<CS10);	//set timer prescale to 1024
	TCNT1 = 49911;		//set TCNT value. Calculated using 65535 - (16MHz/1024-1)
}
// each one second interrupt
ISR(TIMER1_COMPA_vect)
{
	send_message = true;		//set send_message = true
}
// nRF24L01 interrupt
ISR(INT0_vect)
{
	rf_interrupt = true;		//set rf_interrupt = true
}

void read_adc(void)		//this function is responsible for reading the ADC pins
{
	
	unsigned char i=4;	//declare how many times we will measure temperature per average
	adc_temp=0;			//initialize adc_temp to 0
	while(i--)			//while i != 0, keep looping
	{
		
		ADCSRA |= (1<<ADSC);		//start the first conversion
		while(ADCSRA & (1<<ADSC));	//while ADCSRA and 1 is written to ADSC, keep looping
		adc_temp+=ADC;			//sum the ADC values
		_delay_ms(50);			//wait for 50 ms
	}
	adc_temp = adc_temp /8;		//take the average of the measured temperatures
}

void adc_init(void)		//this function is responsible for setting up and enabling the ADC
{
	ADMUX = (0 << REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX1) | (1<<MUX0);
	ADCSRA = (1<<ADEN) | (0<<ADSC) | (0 << ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2)| (0<<ADPS1) | (1<<ADPS0);
}

ISR(TIMER1_OVF_vect)//
{
	read_adc();
	snprintf(outs,sizeof(outs),"%3d\r\n",adc_temp);	//Change to "%3d F\r\n" for degree output.
	USART_tx_string(outs);
	_delay_ms(125);
	TCNT1 = 49911;	//Reset TCNT1.
}

void USART_tx_string(char *data)
{
	while(*data != '\0')
	{
		while(!(UCSR0A & (1<<UDRE0)));	//print out the variable data
		UDR0 = *data;
		data++;
	}
}
