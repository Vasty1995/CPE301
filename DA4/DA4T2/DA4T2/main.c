/*
 * DA4T2.c
 *
 * Created: 4/13/2018 12:37:56 PM
 * Author : YKengne
 */ 

#define F_CPU 8000000UL // XTAL = 8MHZ

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#define BAUDRATE	9600
#define ASYNCH_NORM_PRESCALER (F_CPU/16/BAUDRATE - 1)

void ADC0init();					// Initialize ADC0 input
unsigned short readADC();			// read ADC0 analog input and return it
void delay_ms(unsigned int);		// shell procedule to call _delay_ms on variable input
void step_clockwise(unsigned int, unsigned int);	// step stepper motor desired number of times with delay
unsigned char rotateLeft(unsigned char);	// rotate bits of input to the left.
int USART0_sendChar(char, FILE*);	// Write character to USART0
void usart0_init (void);			// Initialize USART0

// reset stream pointer
// http://www.gnu.org/savannah-checkouts/non-gnu/avr-libc/user-manual/group__avr__stdio.html
FILE USART0_stream = FDEV_SETUP_STREAM(USART0_sendChar, NULL, _FDEV_SETUP_WRITE);

// Current position signal of stepper motor
unsigned char positionSig = 0x33;

int main()
{
 	unsigned short adcVal;
	DDRB = 0xFF; // make portB output pins.

	stdout = &USART0_stream;	// change standard output to point to a USART stream
	
	usart0_init();				// Initialize USART0 for debugging and monitoring
	ADC0init();					// Initialize ADC0 input
	
	while (1)
	{
 		adcVal = readADC();			// read ADC0;
		step_clockwise(1, adcVal);	// Step stepper motor 1 step with an adcVal delay
		printf("ADC Value: %u | Position signal: 0x%X\n", adcVal, positionSig); // print monitoring message
	}
}

void usart0_init (void)
/*
 * Procedure to initialize USART0 asynchronous with enabled RX/TX, 8 bit data,
 * no parity, and 1 stop bit. 
*/
{
	UCSR0B = (1<<TXEN0)  | (1<<RXEN0);	// enable transmit/receive
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);	// asynchronous, 8N1
	UBRR0L = ASYNCH_NORM_PRESCALER;		// Set prescaler based on desired baudrate
}

int USART0_sendChar(char data, FILE *stream)
/*
 * Procedure to send a single character over USART0. If character is linefeed, reset 
 * line. 
 * Assumes ASCII code. 
 */ 
{
	if(data == '\n')	// If character is linefeed,
	{					// First send return.
		while(! (UCSR0A & (1<<UDRE0)) );
		UDR0 = '\r';
	}
	while(! (UCSR0A & (1<<UDRE0)) ); // Wait for last data to be transmitted.
	UDR0 = data;	// send data
	return 0;
}

unsigned char rotateLeft(unsigned char x)
/*
 * Given an unsigned character x, rotateLeft will do a logic rotatation of
 * the bits of x to the right. 
 */ 
{
	unsigned char shiftIn = 0;
	if ((x & 0x80) == 0x80)
		shiftIn = 0x01;
	return ((x<<1) | shiftIn);
}

void step_clockwise(unsigned int steps, unsigned int delay)
/*
 * Given the unsigned integers steps, and delay, step_clockwise will send the appropriate
 * signal to PORTB[7:0] to step a stepper motor in the clockwise direction. 
 * A global variable positionSig must be initialized to 0x33.
 */ 
{
	for (; steps > 0; steps--)	// loop steps times. 
	{
		positionSig = rotateLeft(positionSig);	// Rotate value of positionSig
		PORTB = positionSig;					// send data to PORTB
		delay_ms(delay);						// Delay a given value of milliseconds. 
	}
}

unsigned short readADC()
// readADC will read the adcValue after it has been calculated.
{
	ADCSRA |= (1<<ADSC);				// Begin conversion
	while((ADCSRA & (1<<ADIF)) == 0 );	// Wait for conversion to finish.
	return ADC;
}

void ADC0init()
// ADC0init will initialize analog input on ADC0, set voltage reference to Vcc, with
// data right justified on data register.
{
	DDRC	&= ~(0<<DDC0);
	ADCSRA	= 0x87;			// Make ADC enable and select ck/128
	ADMUX	= (1<<REFS0);	// VCC reference, ADC0 single ended input
	// data will be right-justified
}

void delay_ms(unsigned int count)
/*
 * Procedure to perform a delay based on an unsigned short
 * since the _delay_ms macro will not accept parameters 
 * other than constant values. 
 */
{
	int i;
	for(i = 0; i < count; i++)
		_delay_ms(1);
}
