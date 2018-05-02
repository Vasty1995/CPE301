/*
 * Receiver.c
 *
 * Created: 5/2/2018 9:35:28 AM
 * Author : YKengne
 */ 



#define F_CPU 16000000UL	//16 MHz
#define BAUD 9600	//Set Baud rate 9600.
#define MYUBRR F_CPU/16/BAUD-1 //Configuration for MYUBRR.

#include <avr/io.h>		//include necessary libraries/files
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include <stdio.h>
#include "nrf24l01.h"
#include  "nrf24l01-mnemonics.h"
nRF24L01 *setup_rf(void);
void process_message(char *message);
inline void prepare_led_pin(void);
inline void set_led_high(void);

inline void set_led_low(void);
volatile bool rf_interrupt = false;
void USART_init(unsigned int ubrr);	//Function to initialize USART
void USART_Transmit(unsigned char data); //Function to print the value in Visualizer

int main(void) {
	uint8_t address[5] = { 0x21, 0x26, 0x32, 0xFF, 0x3E };//Can pick any combination of hex values so long as it is the same in the receive address.
	prepare_led_pin();
	sei();		//set enable interrupt
	nRF24L01 *rf = setup_rf();
	nRF24L01_listen(rf, 0, address);
	uint8_t addr[5];
	nRF24L01_read_register(rf, CONFIG, addr, 1);
	USART_init(MYUBRR);	//initialize USART
	while (true) {
		if (rf_interrupt) {
			rf_interrupt = false;
			while (nRF24L01_data_received(rf)) {
				nRF24L01Message msg;
				USART_tx_string(msg);
				nRF24L01_read_received_data(rf, &msg);
				process_message((char *)msg.data);
				
			}
			nRF24L01_listen(rf, 0, address);
		}
	}
	return 0;
}
nRF24L01 *setup_rf(void) {
	nRF24L01 *rf = nRF24L01_init();
	rf->ss.port = &PORTB;
	rf->ss.pin = PB2;		//set ss to PB2
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
void process_message(char *message) {
	if (strcmp(message, "ON") == 0)
	set_led_high();
	else if (strcmp(message, "OFF") == 0)
	set_led_low();
}
inline void prepare_led_pin(void) {
	DDRB |= _BV(PB0);
	PORTB &= ~_BV(PB0);
}
inline void set_led_high(void) {
	PORTB |= _BV(PB0);
}
inline void set_led_low(void) {
	PORTB &= ~_BV(PB0);
}
// nRF24L01 interrupt
ISR(INT0_vect) {
	rf_interrupt = true;	//set rf_interrupt = true on INT0 interrupt
}

void USART_init(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<TXEN0);	//Enable receiver, transmitter & RX interrupt.
	UCSR0C = (3<<UCSZ00);  //Asynchronous 8 N 1
}

void USART_tx_string(char *str)
{
	unsigned int i=0;
	while(str[i]!='\0')
	{
		USART_Transmit(str[i]);		//print out variable str
		i++;
	}
}

void USART_Transmit(unsigned char data)
{
	//check if buffer is empty so that data can be written to transmit (check code example in datasheet)
	while(!(UCSR0A & (1<<UDRE0)));
	//copy data to be sent to UDR0
	UDR0 = data;
}
