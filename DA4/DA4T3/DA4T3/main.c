/*
 * DA4T3.c
 *
 * Created: 4/13/2018 12:39:44 PM
 * Author : YKengne
 */ 

#define F_CPU 8000000UL // XTAL = 8MHZ

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#define SERVO_MIN	65
#define SERVO_MAX	285
#define BAUDRATE	9600
#define ASYNCH_NORM_PRESCALER (F_CPU/16/BAUDRATE - 1)

void ADC0init();					// Initialize ADC0 input
unsigned short readADC();			// read ADC0 analog input and return it
void delay_ms(unsigned int);		// shell procedule to call _delay_ms on variable input
void PWM_OC1A_init();				// Initialize PWM on OC1A at 50Hz
int USART0_sendChar(char, FILE*);	// Send character on USART0
void usart0_init (void);			// Initialize USART0

// reset stream pointer
// http://www.gnu.org/savannah-checkouts/non-gnu/avr-libc/user-manual/group__avr__stdio.html
FILE USART0_stream = FDEV_SETUP_STREAM(USART0_sendChar, NULL, _FDEV_SETUP_WRITE);

int main()
{
 	unsigned short adcVal;	// Variable to store input ADC Value
	unsigned short newVal;	// new value calculated based on a range for servo
	DDRB = 0xFF; // make portB output pins.

	ADC0init();			// Initialize ADC0 input
	PWM_OC1A_init();	// initialize pwm  on OC1A
	usart0_init();		// Initialize USART0 for debugging and monitoring
	
	stdout = & USART0_stream;	// change standard output to point to a USART stream
	
	while (1)
	{
 		adcVal = readADC();			// read ADC0;
		// Map ADC value to a range from 0 to SERVO_MAX
		newVal = (unsigned short)((float)adcVal / ((1UL<<10) - 1) * SERVO_MAX);
		printf("adcVal = %u\n", adcVal);	// Print monitoring data
		printf("\tnewVal = %u\n", newVal);
		if (newVal <= SERVO_MIN)	// If newVal is less than minimum servo value (0 degrees)
			OCR1A = SERVO_MIN;		// then set OCR1A to minimum value.
		else
		{
			OCR1A = newVal;			// else, update OCR1A to change duty cycle.
		}
		_delay_ms(50);				// Delay to allow servo to move 
	}
}

int USART0_sendChar(char data, FILE *stream)
/*
 * Procedure to send a single character over USART0. If character is linefeed, reset 
 * line. 
 * Assumes ASCII code. 
 */ 
{
	if(data == '\n')
	{
		while(! (UCSR0A & (1<<UDRE0)) );
		UDR0 = '\r';
	}
	while(! (UCSR0A & (1<<UDRE0)) );
	UDR0 = data;
	return 0;
}

void usart0_init (void)
/*
 * Procedure to initialize USART0 asynchronous with enabled RX/TX, 8 bit data,
 * no parity, and 1 stop bit. 
*/
{
	UCSR0B = (1<<TXEN0)  | (1<<RXEN0);	// enable transmit/receive
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);	// asynchronous, 8N1
	UBRR0L = ASYNCH_NORM_PRESCALER;		// To set 9600 baud rate with 8MHz clock
}

unsigned short readADC()
/*
 * Procedure to send a single character over USART0. If character is linefeed, reset 
 * line. 
 * Assumes ASCII code. 
 */ 
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

void PWM_OC1A_init()
{
	//Set PORTB1 pin as output
	DDRB |= (1<<DDB1);	// make OC1A as output.
	// Output compare mode on OC1A. Fast PWM with top = ICR1.
	// Clear OC1A on Compare match and set at bottom.
	TCCR1A |= (1<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(1<<WGM11)|(0<<WGM10);
	// Start timer with prescaler 64
	TCCR1B |= (0<<ICNC1)|(0<<ICES1)|(1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10);
	ICR1 = 2499;  // F_CPU / (N * F_pwm) - 1, where N is the prescaler = 64, and F_pwm is the desired 50Hz frequency.
}
