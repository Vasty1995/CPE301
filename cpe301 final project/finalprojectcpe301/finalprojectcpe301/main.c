/*
 * finalprojectcpe301.c
 *
 * Created: 4/27/2018 1:58:06 PM
 * Author : YKengne
 */ 



#define F_CPU 16000000UL			// Clock Speed

#include <avr/io.h>			
#include <util/delay.h>		// Needed for _delay_ms()
#include <stdlib.h>
// Necessary Libraries for UART
#include <stdint.h>			// needed for uint8_t
#include <stdio.h>			// Needed for printf
#include <math.h>			// Needed for arctan
#define BAUD 9600			// Setup the BUAD Rate
#define MYUBRR F_CPU/16/BAUD -1  // Value being written into UBRR0 Register

// i2c functions
void i2c_stop();	// Function to stop i2c_communication
void i2c_write(unsigned char);	// Function to send a value via i2c
void i2c_start(void);	// Function to start i2c_communication
void i2c_init(void);	// Function to initialize the micrc controller for i2c
uint8_t i2c_ReadACK(void);	// Function to read value via i2c with an acknowledgment
uint8_t i2c_ReadNACK(void); // Function to read value via i2c without an acknowledgment

// Compass  Stuff 
void i2c_ReadCompass(void);	// Function to read Heading from compass
void i2c_CompassSetUp(void); // Function to initialize the compass
float heading;	// float value to hold the heading

// Global Variables for Compass Functions
int Xval = 0;
int Yval = 0;
int Zval = 0;

// USART Functions
int USART0SendByte(char, FILE *stream);	// Declaration of the Method to transmit char
int USART0ReceiveByte(FILE *stream); // Declaration of the Method to Receive a char
void USART0init(void);		// Declaration of USART init function
void delay();	// Method used to delay for 1s
FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, USART0ReceiveByte, _FDEV_SETUP_RW);

// Accelerometer
void i2c_ReadAccel(void); // Method to read values from Accelerometer
void i2c_AccelSetUp(void);	// Method to initialize the accelerometer

float X_accel, Y_accel, Z_accel; // Converted Values from the Accelerometer
int X_accel_Raw, Y_accel_Raw, Z_accel_Raw; // Raw Values from the Accelerometer


// Barometer/ Temperature Sensor functions
void readBarometer(void);	// Method to read the values from the barometer
void getBarometerCalibrations(void);	// Method to read the calibration values from the Barometer
void readBarometerUP(void);	// Method to read the uncalibrated pressure from the barometer
void readBarometerTemp(void);	// Method to read the uncalibrated temperature from the barometer
void ConvertUP(void);	// Method to convert the uncalibrated pressure to calibrated pressure
void convertTemp(void);	// Method to convert uncalibrated temperature to calibrated temperature

// Calibration Values - These values are set via getBarometerCalibrations()
long oss;
long AC1, AC2, AC3, B1, B2, MB, MC, MD;
unsigned long AC4, AC5, AC6;
long X1, X2, X3, B3, B5, B6;
unsigned long B4, B7;
// User Values
long UP = 0; // Uncalibrated Pressure
long BaroTemp = 0; // Uncalibrated Temperature
float altitude = 0.0; // The elevation in feet
float TempC = 0.0;	// The temperature in Celsius
float Std_Pressure = 100380; // Pascals

// Gyro Functions
void Gyro_init(void);	// Method to initialize the Gyro
void Gyro_read(void);	// Method to read values from the Gyro
int X_Gyro_Raw, Y_Gyro_Raw, Z_Gyro_Raw; // Raw Values
float X_Gyro, Y_Gyro, Z_Gyro; // Calibrated Values
void ConvertValues(void); // Converts raw values to usable real values


int main(void)
{
	USART0init(); // Enable the USART
	stdin=stdout=&usart0_str;

	i2c_init();
	delay();
	_delay_ms(100);		// Allow the sensor to boot
	// Initialize all the sensors
	i2c_CompassSetUp();
	i2c_AccelSetUp();
	Gyro_init();
	
	_delay_ms(100);  // Allow for modules to boot	
	while (1)
	{
		i2c_ReadCompass();
		i2c_ReadAccel();
		getBarometerCalibrations(); // This must be done every time since values may change
		readBarometer();		
		Gyro_read();
		ConvertValues();
		// Send the values to the ESP8266
		printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d\n",
		X_accel, Y_accel, Z_accel, altitude, TempC, X_Gyro, Y_Gyro, Z_Gyro, heading, Zval);
		// Wait one second before reading new values
		delay();
	}
}

//i2c functions
void i2c_stop()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}
//*******************************************************
void i2c_write(unsigned char data)
{
	TWDR = data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
}
//*********************************************************
void i2c_start(void)
{
	TWCR = (1<< TWINT) | (1<<TWSTA) | (1<< TWEN);
	while (!(TWCR & (1<<TWINT)));
}
//***********************************************************
void i2c_init(void)
{
	TWSR = 0x00; //set prescaler bits to 0
	TWBR = 0x0C; //0x48; //SCL freq. is 100k for XTAL = 16M
	TWCR = 0x04; //enable TWI module
}
//***********************************************************
uint8_t i2c_ReadACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while ((TWCR & (1<<TWINT)) == 0); // Wait until acknowledment is received
	return TWDR;
}
//read byte with NACK
uint8_t i2c_ReadNACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0); 
	return TWDR;
}
//***********************************************************

// ---------------- Compass Functions -------------------------------
void i2c_CompassSetUp(void)
{
	i2c_start();
	i2c_write(0x3C);	// Address + Write bit of compass
	i2c_write(0x00);	// Address of configuration register A
	i2c_write(0x70);	// 15Hz rate with 8 Samples
	i2c_stop();
	i2c_start();
	i2c_write(0x3C);	// Address + Write bit of compass
	i2c_write(0x01);	// Address of configuration register B
	i2c_write(0xA0);	// Gain = 5
	i2c_stop();

}

void i2c_ReadCompass(void)
{
	// Set up Single Measurement Mode
	i2c_start();
	i2c_write(0x3C);	// Address + Write bit of compass
	i2c_write(0x02);	// Address of configuration register B
	i2c_write(0x01);	// Gain = 5
	i2c_stop();

	_delay_ms(10);		// Delay for at least 6 ms

	i2c_start();
	i2c_write(0x3D);	// Address + Read bit
	// Read the values
	// Address pointer automatically increments after
	// every read
	Xval = i2c_ReadACK() << 8;
	Xval |= i2c_ReadACK();
	Zval = i2c_ReadACK() << 8;
	Zval |= i2c_ReadACK();
	Yval = i2c_ReadACK() << 8;
	Yval |= i2c_ReadNACK();
	i2c_stop();
}

// ------------------- USART Functions ------------------------------

// Sends a character via USART
int USART0SendByte(char u8Data, FILE *stream)
{
	if(u8Data == '\n')
	{
		USART0SendByte('\r', 0);
	}
	//wait while previous byte is completed
	while(!(UCSR0A&(1<<UDRE0))){};
	// Transmit data
	UDR0 = u8Data;
	return 0;
}

// Receives a character via USART
int USART0ReceiveByte(FILE *stream)
{
	uint8_t u8Data;
	// Wait for byte to be received
	while(!(UCSR0A&(1<<RXC0))){};
	u8Data=UDR0;
	//echo input data
	USART0SendByte(u8Data,stream);
	// Return received data
	return u8Data;
}

void USART0init(void)
{
	/*Set baud rate */
	UBRR0L = MYUBRR;
	UCSR0B |= (1 << TXEN0);				// Enable transmitter
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // Set frame: 8-bit data
}

// Function delays for 1 second
void delay(){
	for(int i = 500; i > 0; i--)
	_delay_ms(2);
}

void i2c_ReadAccel(void)
{
////// X /////////
	i2c_start();
	i2c_write(0xA6); // Address to write to Accelo
	i2c_write(0x33); // X msb Address
	i2c_start();
	i2c_write(0xA7); // Address to read from Accelo
	X_accel_Raw = i2c_ReadACK() << 8;

	i2c_start();
	i2c_write(0xA6); // Address to write to Accelo
	i2c_write(0x32); // X lsb Address
	i2c_start();
	i2c_write(0xA7); // Address to read from Accelo
	X_accel_Raw |= i2c_ReadACK();
////// Y ////////
	i2c_start();
	i2c_write(0xA6); // Address to write to Accelo
	i2c_write(0x35); // Y msb Address
	i2c_start();
	i2c_write(0xA7); // Address to read from Accelo
	Y_accel_Raw = i2c_ReadACK() << 8;

	i2c_start();
	i2c_write(0xA6); // Address to write to Accelo
	i2c_write(0x34); // Y lsb Address
	i2c_start();
	i2c_write(0xA7); // Address to read from Accelo
	Y_accel_Raw |= i2c_ReadACK();

////// Z ////////
	i2c_start();
	i2c_write(0xA6); // Address to write to Accelo
	i2c_write(0x37); // Y msb Address
	i2c_start();
	i2c_write(0xA7); // Address to read from Accelo
	Z_accel_Raw = i2c_ReadACK() << 8;

	i2c_start();
	i2c_write(0xA6); // Address to write to Accelo
	i2c_write(0x36); // Y lsb Address
	i2c_start();
	i2c_write(0xA7); // Address to read from Accelo
	Z_accel_Raw |= i2c_ReadNACK();
	i2c_stop();
}

void i2c_AccelSetUp(void)
{	
	i2c_start();
	i2c_write(0xA6);
	i2c_write(0x2D);	// Power Ctl register
	i2c_write(0x08);	// Enable measurement mode
	i2c_stop();
}

// This function converts the raw pressure read from the barometer to
// the actual pressure value.  These conversions were obtained from
// the datasheet
void ConvertUP(void)
{	
	B6 = B5 - 4000;
	X1 = B6*B6;
	X1 = X1 / 4096;
	X1 = X1 * B2;
	X1 = X1 / 2048;
	X2 = AC2 * B6;
	X2 = X2 / 2048;
	X3 = X1 + X2;
	B3 = AC1 * 4;
	B3 = B3 + X3;
	B3 = B3 << oss;
	B3 = B3 + 2;
	B3 = B3 / 4;
	X1 = AC3 * B6;
	X1 = X1 / 8192;
	X2 = B6 * B6;
	X2 = X2 / 4096;
	X2 = X2 * B1;
	X2 = X2 / 65536;
	X3 = X1 + X2;
	X3 = X3 + 2;
	X3 = X3 / 4;
	B4 = ((unsigned long)(X3 + 32768));
	B4 = B4 * AC4;
	B4 = B4 / 32768;
	B7 = (unsigned long)UP - B3;
	B7 = B7 * (50000 >> oss);
	if(B7 < 0x80000000) {
		UP = B7 * 2;
		UP = UP / B4;
	}
	else {
		X1 = UP/256;
		X2 = X1;
		X1 = X1 * X2;
		X1 = X1 * 3038;
		X1 = X1 / 65536;
		X2 = (-7357 * UP);
		X2 = X2 / 65536;
		X3 = X1 + X2 + 3791;
		X3 = X3 / 4;
		UP = UP + X3;
	}
}

// Reads the Raw Barometer Pressure and stores it into UP
void readBarometerUP(void)
{
	// Set up for UP Sampling...
	i2c_start();
	i2c_write(0xEE);	// Barometer address + write
	i2c_write(0xF4);	// Config register for temp/UP
	i2c_write(0x34);	// Oversampling Resolution 0 *4.5 ms
	i2c_stop();

	oss = 0;
	_delay_ms(10);		// Delay at least 4.5 ms for UP reading
	// Begin Reading...
	i2c_start();
	i2c_write(0xEE);	// Barometer address + write
	i2c_write(0xF6);	// MSB of data register
	i2c_start();		// Restart
	i2c_write(0xEF);	// Barometer address + Read
	UP = i2c_ReadACK();
	UP = UP << 8;
	UP = UP + i2c_ReadACK();
	UP = UP << 8;
	UP = UP + i2c_ReadNACK();
	UP = UP >> (8 - oss);
	i2c_stop();
}

// Obtains all the Calibration values needed for converting the raw
// values.
void getBarometerCalibrations(void)
{
	i2c_start();
	i2c_write(0xEE);	// Barometer Address + write
	i2c_write(0xAA);	// go to MSB of AC1
	i2c_start();		// Restart
	i2c_write(0xEF);	// Barometer Address + Read
	AC1 = i2c_ReadACK() << 8;	// MSB
	AC1 |= i2c_ReadACK();		// LSB
	AC2 = i2c_ReadACK() << 8;	// MSB
	AC2 |= i2c_ReadACK();		// LSB
	AC3 = i2c_ReadACK() << 8;	// MSB
	AC3 |= i2c_ReadACK();		// LSB
	AC4 = i2c_ReadACK() << 8;	// MSB
	AC4 |= i2c_ReadACK();		// LSB
	AC5 = i2c_ReadACK() << 8;	// MSB
	AC5 |= i2c_ReadACK();		// LSB
	AC6 = i2c_ReadACK() << 8;	// MSB
	AC6 |= i2c_ReadACK();		// LSB
	B1 = i2c_ReadACK() << 8;	// MSB
	B1 |= i2c_ReadACK();		// LSB
	B2 = i2c_ReadACK() << 8;	// MSB
	B2 |= i2c_ReadACK();		// LSB
	MB = i2c_ReadACK() << 8;	// MSB
	MB |= i2c_ReadACK();		// LSB
	MC = i2c_ReadACK() << 8;	// MSB
	MC |= i2c_ReadACK();		// LSB
	MD = i2c_ReadACK() << 8;	// MSB
	MD |= i2c_ReadNACK();		// LSB
	i2c_stop();
}

// Reads temperature
void readBarometerTemp(void)
{
	// Set up for UP Sampling...
	i2c_start();
	i2c_write(0xEE);	// Barometer address + write
	i2c_write(0xF4);	// Config register for temp/UP
	i2c_write(0x2E);	// Temperature *4.5ms time
	i2c_stop();

	_delay_ms(7);		// Delay at least 4.5ms for temp reading...
	// Begin Reading...
	i2c_start();
	i2c_write(0xEE);	// Barometer address + write
	i2c_write(0xF6);	// MSB of data register
	i2c_start();		// Restart
	i2c_write(0xEF);	// Barometer address + Read
	BaroTemp = i2c_ReadACK() << 8; // MSB
	BaroTemp|= i2c_ReadNACK(); // LSB
	i2c_stop();
}

// This function converts the raw temperature read from the barometer
// into a temperature displayed in degrees Celsius.  This conversion was 
// obtained from the datasheet for the barometer.
void convertTemp(void)
{
	X1 = (BaroTemp - AC6);
	X1 = X1 * AC5;
	X1 = X1 / 32768;
	X2 = MC * 2048;
	X3 = X1 + MD;
	X2 = X2 / X3;
	B5 = X1 + X2;
	BaroTemp = (B5 + 8);
	BaroTemp = BaroTemp/16;
}

// Initializes the Gyro to sample data.
// The range of values sampled is +- 2000 degrees
void Gyro_init(void)
{
	i2c_start();
	i2c_write(0xD0); // +- 2000 degrees/sec
	i2c_write(0x16);
	i2c_write(0x18);
	i2c_stop();
}

void Gyro_read(void)
{
	i2c_start();
	i2c_write(0xD0);	// Write
	i2c_write(0x1D);	// Address of X MSB
	i2c_start();
	i2c_write(0xD1);
	// Read the raw values into their respective variables
	// The address pointer increments automatically after
	// each read.
	X_Gyro_Raw = i2c_ReadACK() << 8;
	X_Gyro_Raw |= i2c_ReadACK();
	Y_Gyro_Raw = i2c_ReadACK() << 8;
	Y_Gyro_Raw |= i2c_ReadACK();
	Z_Gyro_Raw = i2c_ReadACK() << 8;
	Z_Gyro_Raw |= i2c_ReadNACK();
	i2c_stop();
}

// Function to read the values from the barometer
// The values must be read in this order since some of the
// temperature values are used in the barometer conversion.
void readBarometer(void)
{
	readBarometerTemp();
	readBarometerUP();
}


// Converts all the raw values into tangible data that can be used.
void ConvertValues(void)
{
	// Heading from compass
	heading = atan2((double)Yval, (double)Xval); // Returns a value in radians
	// Next three lines convert that radian value to a degree value more
	// commonly seen on a compass
	heading = heading * 180;
	heading = heading /3.14159;
	heading += 180;
	// G force from accelerometer
	// 0.004 is used since that is the value provided by the datasheet
	// Each bit represents 0.004g
	X_accel = X_accel_Raw * 0.004;
	Y_accel = Y_accel_Raw * 0.004;
	Z_accel = Z_accel_Raw * 0.004;
	// Convert Barometer Values
	// Values must be converted in this order since the pressure relies
	// on a few temperature variables.
	convertTemp();
	ConvertUP();
	// BaroTemp is 10x larger than actual Celsius value
	TempC = BaroTemp / 10.0;
	// Altitude formula
	altitude = pow(UP/ Std_Pressure, 0.190284);
	altitude = 1 - altitude;
	altitude = altitude * 44330;
	altitude = altitude * 3.28084;
	// Gyro Values
	// Value to divide by is given in the datsheet for the conversion
	X_Gyro = X_Gyro_Raw / 14.375;
	Y_Gyro = Y_Gyro_Raw / 14.375;
	Z_Gyro = Z_Gyro_Raw / 14.375;
}
