#define F_CPU 8000000UL // used by delay.h

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <math.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define MASTER 1   // 1 for Master, 0 for Slave
#define PCBTESTMODE 0 // 1 for running tests, 0 for experiments

#define FOSC 8000000 // oscillator clock frequency, page 164 datasheet, internal RC oscillator clock source selected by fuse bits
#define BAUD 9600 // baud rate desired
#define MYUBRR ((FOSC / (16L * BAUD)) - 1) // used to set the UBRR high and low registers, Usart Baud Rate registers, for formula see datasheet page 146

//// IMU information
#define accell_slave_addrs 0b11010000									// IMU address on slave board,  [7bit i2c embedded address from IMU chip datasheet,0] = 0xd0
#define accell_master_addrs 0b11010010									// IMU address on master board, [7bit i2c embedded address from IMU chip datasheet,0] = 0xd2
#define IMU_ADDRESS (MASTER ? accell_master_addrs : accell_slave_addrs) // (Condition? true_value: false_value), for unifying master/slave code

//// IMU chip registers
// accelerometer registers 0x3B to 0x40 for MPU-9250 (Mar 2019 order), 0x2D to 0x32 for ICM-20948 (Dec 2018 order)
#define ACCEL_XOUT_H 0x3B // 0x2D //for ICM-20948
#define ACCEL_XOUT_L 0x3C // 0x2E //for ICM-20948

#define ACCEL_YOUT_H 0x3D // 0x2F //for ICM-20948
#define ACCEL_YOUT_L 0x3E // 0x30 //for ICM-20948

#define ACCEL_ZOUT_H 0x3F // 0x31 //for ICM-20948
#define ACCEL_ZOUT_L 0x40 // 0x32 //for ICM-20948

#define PWR_MGMT_1 0x6B // should be set to 0 for Accel to be run, see datasheet page 31

//// system states
#define SETUP 0x10
#define FLIP 0x20

#define UNWIND 0x00
#define REWIND 0x01
#define SLAVE_GRIPPER 0x02
#define MASTER_GRIPPER 0x03
#define FLIPPING1 0x04
#define ATTACHING 0x05
#define DETACHING 0x06

#define atmega_slave 0xf0 // // address of the slave board processor, to be renamed to something more meaningful e.g. MCU_slave_address
#define led_wrt_cmd 0x3A  // led driver write command

//////////////////////////////////////
//
// MACROS FOR PIN HANDLING (STOLEN FROM JULIA)
#define output(directions, pin) (directions |= pin)   //set port direction for output
#define input(directions, pin) (directions &= (~pin)) //set port direction for input
#define set(port, pin) (port |= pin)				  // set port pin
#define clear(port, pin) (port &= (~pin))			  // clear port pin
//////////////////////////////////////

//HELPER MACROS
#define get_switch_input(port, pin) ((port & (1 << pin)) >> pin) //bit shift function to get input from switches
#define convert_negatives(value) (value>8000) ? (-(value ^ 0xffff)-1):value 	// Convert two's complement for negative values, 0x8010 = -32752 is the lowest
#define rad2deg(rad) rad*180/3.14159 //Convert radians to degrees

//#define neutral_bend(angle) (angle)

////////////////
//SETUP AND DEFINE PINS

//Vreg1 Pin (Power on) is PC0
#define vreg1_port_direction DDRC
#define vreg1_port PORTC
#define vreg1_pin (1 << 0)

//LED is PD7
#define led_port_direction DDRD
#define led_port PORTD
#define led_pin (1 << 7)

//Switch S4 PD4
#define S4_port_direction DDRD
#define S4_port PIND //(((PIND & (1<<4)) >> 4))
#define S4_pin 4

//Switch "Power" PD3
#define S3_port_direction DDRD
#define S3_port PIND
#define S3_pin 3

//Switch "Tension"
#define STension_port_direction DDRB
#define STension_port PINB
#define STension_pin 1

//Motor M5 direction is PB6 and speed/PWM is OCR0B (PD5)
#define M5_pin (1 << 6)
#define M5_port_dir DDRB
#define M5_port PORTB

#define M5_PWM_port_dir DDRD
#define M5_PWM_pin (1 << 5)
#define M5_PWM OCR0B

//Motor M3 direction is PB7 and speed/PWM is OCR0A (PD6)
#define M3_pin (1 << 7)
#define M3_port_dir DDRB
#define M3_port PORTB

#define M3_PWM_port_dir DDRD
#define M3_PWM_pin (1 << 6)
#define M3_PWM OCR0A

//Motor M4 - Vibration motor is PB0
#define M4_pin (1 << 0)
#define M4_port_dir DDRB
#define M4_port PORTB

///////////////

//Flip Parameters:

#define WINDSPD 130
#define UNWINDSPD 210
#define GRIPPER_SPD 210
#define NUM_DETACH_FLIP_ATTEMPTS 10

//IMU Parameters

#define IMU_SAMPLE_NUM 15
#define IMU_DELTA_THRESH 5000

//Touch Sensor Thresholds

//Individual Sensors
//#define SURFACE_SEEN 900
//#define TOUCHING 700
//#define CONNECTED 600
//#define WELL_CONNECTED 500

#define CLOSE_ATT_THRESH 700 //660 // At least one IR must be under this (probably the close one)
#define FAR_ATT_THRESH 880 // Both IRs should be under 900 to attach

#define DET_THRESH 915 // IRs must be over this to be considered detached. 

//Combination  Thesholds (didn't work as well)
#define DETACH_THRESH 1840 
#define ATTACH_THRESH 1200

//Other Useful Variables
#define PI 3.14159
#define RAD2DEG 180/PI 

//Define helper functions
void setLED(unsigned char red, unsigned char green, unsigned char blue);
void set_M3(unsigned char dir, unsigned char speed);
void set_M5(unsigned char dir, unsigned char speed);
int switch_power(void);
int switch_tension1(void);
int switch_S4(void);
int get_bend(void);
int get_IR_Flex_U1513(void);
int get_IR_U5(void);
void get_IMU_measurement(unsigned char self);
void master_output_update(void);
void master_input_update(void);
void flipbend(char side, char p);
void zero_motors(void);
void LEDsOff(void);
double get_accel_diff(void);
void detect_pulse();

void init(void);
int i2c_send(void);

// system state
uint8_t system_state = SETUP;

// Robot Inputs/Sensors
struct inputs
{
	unsigned char switch_tension_m;
	unsigned char switch_tension_s;
	unsigned char switch_S4_m;
	unsigned char switch_S4_s;
	int bend_m;
	int bend_s;
	int IR1_m;
	int IR1_s;
	int IR2_m;
	int IR2_s;
	int accell_m[3];
	int accell_s[3];
	unsigned char switch_power;
};

//Define Robot Outputs

struct outputs
{
	uint8_t speed_m3_m;
	uint8_t speed_m3_s;
	uint8_t speed_m5_m;
	uint8_t speed_m5_s;
	uint8_t direction_m5_m;
	uint8_t direction_m5_s;
	uint8_t direction_m3_m;
	uint8_t direction_m3_s;
	uint8_t led_m[3];
	uint8_t led_s[3];
	uint8_t vibration_m;
};

struct outputs output;
struct inputs input;

// define helper variables
static uint8_t power_state;   //
static uint8_t toggle_wakeup; //variable for power/sleep function
static uint8_t sleep_mode;	//

// setup for printf
int uart_putchar(char c, FILE *stream)
{
	if (c == '\n')
		uart_putchar('\r', stream);
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;

	return 0;
}

// ??BH?? the following commented lines can be removed if code compiles/works well in older Atmel Studio projects
/* #define FDEV_SETUP_STREAM(put, get, rwflag) from stdio.h	for use when programming in C, older Atmel Studio projects
Initializer for a user-supplied stdio stream. 
This macro acts similar to fdev_setup_stream(), used as the initializer of a variable of type FILE.
*/
//FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE mystdout;

// rename function to something more meaningful, i.e. USART_init
void ioinit(void)
{
	// Set baud rate
	UBRR0H = (unsigned char)(MYUBRR >> 8);
	UBRR0L = (unsigned char)MYUBRR;
	// Enable USART transmitter to print on PC screen, (1<<TXEN0)
	// Enabel USART receiver to receive commands from PC, (1<<RXEN0), not included in original code
	UCSR0B = (1 << TXEN0);

	/* Set frame format: 8data, 2stop bit, not included in original code
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
    */

	/* #define fdev_setup_stream(stream, put, get, rwflag) from stdio.h	for use when programming in C++, Atmel Studio 7 project
	Setup a user-supplied buffer as an stdio stream.
	This macro takes a user-supplied buffer stream, and sets it up as a stream that is valid for stdio operations, 
	similar to one that has been obtained dynamically from fdevopen(). The buffer to setup must be of type FILE.
	The rwflag argument can take one of the values _FDEV_SETUP_READ, _FDEV_SETUP_WRITE, or _FDEV_SETUP_RW, for read, write, or read/write intent, respectively.
    */
	fdev_setup_stream(&mystdout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
	stdout = &mystdout;
}

/////Setup everything

void init(void)
{
	// Set all pins on Hi-Z, all ports as input & pull-up resistors deactivated
	DDRB = 0;
	PORTB = 0;
	DDRC = 0;
	PORTC = 0;
	DDRD = 0;
	PORTD = 0;

	// USART init
	ioinit();

	sleep_mode = 0;
	//power on
	output(vreg1_port_direction, vreg1_pin);
	set(vreg1_port, vreg1_pin); //turn on PC0 (vreg1)

////////////
	// initialize outputs
	output(led_port_direction, led_pin); // RGB led init
	output(M3_port_dir, M3_pin); //output M3 direction control
	output(M5_port_dir, M5_pin); //output M5 direction control
	output(M3_PWM_port_dir, M3_PWM_pin); //output M4 PWM
	output(M5_PWM_port_dir, M5_PWM_pin); //output M5 PWM
	output(M4_port_dir, M4_pin); //output M4 vibration motor

///////////
	//initialize inputs
	input(STension_port_direction, STension_pin); //tension switch as input PB1
	input(S4_port_direction, S4_pin); //gripper/control switch as input PD4
	input(S3_port_direction, S3_pin); //power switch as input PD3

	TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00); //Timer counter init (for pwm/motor control)
	TCCR0B = 0x03;											//prescaler set to 0
	OCR0B = 0x00;											//start with motor off (PD5)
	OCR0A = 0x00;											//start with motor off (PD6)

	DDRC &= ~(1 << 1); //IR2 or flex sensor as input (PC1)
	DDRC &= ~(1 << 2); //IR1 as input (PC2)
	DDRC &= ~(1 << 3); //strain gage as input (PC3)
	//initalize adc
	ADMUX = (1 << REFS0);				 //|(1<<MUX0);//choose analog pin
	ADCSRA = (1 << ADEN) | (1 << ADPS0); //set up a/d

	// ??BH?? INT0 ISR does not exist, i.e. the interrupt ISR executes no code **MM we haven't added in the wakeup protocol - check old AutoAccel file?
	//enable interrupt INT0/PCINT18/PD2, used by U1/IMU
	DDRD &= ~(1 << 2);				 // DDRxn reset, PD2 configured as input
	PORTD |= 1 << 2;				 // PORTxn set when pin configured as input, PD2 pull-up resistor activated
	EIMSK = 1 << 1;					 // External Interrupt Mask Register (EIMSK) bit 1 set, INT0 enabled
	EICRA = 1 << ISC01 | 1 << ISC00; // External Interrupt Control Register A (EICRA), ISCxx Interrupt Sense Control bits, INT0 set to trigger on rising edge

	TWBR = 0x04; //twi bit rate set

	power_state = 1;
	sleep_mode = 1;
	toggle_wakeup = 0;


//SETUP SLAVE (From old code??? but definitely necessary)
	if(!MASTER){
		TWAR=atmega_slave;
		TWCR= (1<<TWEA)|(1<<TWEN);//|(1<<TWINT);
	}
}

// AVR TWI is byte-oriented and interrupt based
// ISR for the 2 wire Serial interface
ISR(TWI_vect)
{
	cli();
	printf("TWI ISR reached ...\n\r");

	TWCR &= ~(1 << TWIE); // Two Wire Control Register (TWCR),
	// TWEN bit = 1 to enable the 2-wire serial interface
	// TWINT bit = 1 to clear the TWINT flag
	TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
}

//////////////////////////////////////////////////////////////////
//IMU FUNCTIONS

// This function basically initializes the IMU
uint8_t i2c_write_accell(uint8_t chip_address, uint8_t reg_address, uint8_t data) //??? Double check function not missing anything?
{
	// start TWI transmission
	// Two Wire Control Register (TWCR)
	// TWEN bit = 1 to enable the 2-wire serial interface
	// TWSTA bit = 1 to transmit a START condition
	// TWINT bit = 1 to clear the TWINT flag
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

	// wait for TWINT bit in TWCR reg set by the TWI control unit
	// this indicates START condition transmission is finished
	// user application code then checks TWSR reg if START condition transmission successful
	while (!(TWCR & (1 << TWINT)));

	// Checks TWSR reg if START condition transmission successful, see datasheet page 183
	if ((TWSR & 0xF8) != 0x08)
	{
		printf("TWI START condition transmission error\n\r");
		// ToDo: add error handling code, not in original code
	}

	// Load write address of slave into TWDR
	TWDR = chip_address;

	// clear TWINT bit in TWCR to start transmission of address
	TWCR = (1 << TWINT) | (1 << TWEN);

	// wait for TWINT flag to raise, indicating transmission and ack/nack receive
	while (!(TWCR & (1 << TWINT)));

	// Check TWSR reg to if ack received
	if ((TWSR & 0xF8) != 0x18)
		printf("first ack problem 0x%x \n\r", (TWSR & 0xF8));
	// printf("ack recieved OK 0x%x \n\r",(TWSR & 0xF8));

	// Send data, in this case an address
	TWDR = reg_address;				   //0x6b;// accell x value msb's
	TWCR = (1 << TWINT) | (1 << TWEN); //start tx of data

	while (!(TWCR & (1 << TWINT)))
		; //wait for data to tx

	// check for data ack
	if ((TWSR & 0xF8) != 0x28)
		printf("second ack problem is 0x%x\n\r", (TWSR & 0xF8));
	// printf("second ack received OK is 0x%x\n\r",(TWSR & 0xF8));

	//send data
	TWDR = data;					   //0x00;// accell x value msb's
	TWCR = (1 << TWINT) | (1 << TWEN); //start tx of data

	while (!(TWCR & (1 << TWINT)))
		; //wait for data to tx

	// check for data ack
	if ((TWSR & 0xF8) != 0x28)
		printf("third ack problem is 0x%x\n\r", (TWSR & 0xF8));
	// printf("third ack received OK is 0x%x\n\r",(TWSR & 0xF8));

	// transmit STOP condition
	TWCR = (1 << TWINT) | (1 << TWSTO);

	return (0);
}

// This reads the data from the IMU
uint8_t i2c_read_accell(uint8_t chip_address, uint8_t reg_address) //??? Double check function not missing anything?
{

	/* // ??BH?? I think this can be omitted */
	// start TWI transmission
	// Two Wire Control Register (TWCR)
	// TWEN bit = 1 to enable the 2-wire serial interface
	// TWSTA bit = 1 to transmit a START condition
	// TWINT bit = 1 to clear the TWINT flag
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

	// wait for TWINT bit in TWCR reg set by the TWI control unit
	// this indicates START condition transmission is finished
	// user application code then checks TWSR reg if START condition transmission successful
	while (!(TWCR & (1 << TWINT)))
		;

	// user application code checks TWSR reg if START condition transmission successful, see datasheet page 183
	if ((TWSR & 0xF8) != 0x08)
	{
		printf("TWI START condition transmission error\n\r");
		// ToDo: add error handling code
	}

	// load SLA+R/W into TWDR, 7bit slave addres + 1bit (0 for write/1 for read by the master chip)
	// uint8_t	SLA_W=0b11010000;
	uint8_t SLA_W = chip_address;
	TWDR = SLA_W;

	//start transmission
	TWCR = (1 << TWINT) | (1 << TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while (!(TWCR & (1 << TWINT)))
		;

	//check to see if ack received
	if ((TWSR & 0xF8) != 0x18)
		printf("first ack problem 0x%x \n\r", (TWSR & 0xF8));
	//	printf("ack recieved OK 0x%x \n\r",(TWSR & 0xF8));

	//send data, in this case an address
	TWDR = reg_address;				   // accell x value msb's
	TWCR = (1 << TWINT) | (1 << TWEN); //start tx of data

	while (!(TWCR & (1 << TWINT))); //wait for data to tx

	//check for data ack
	if ((TWSR & 0xF8) != 0x28)
		printf("second ack problem is 0x%x\n\r", (TWSR & 0xF8));
	//	printf("second ack received OK is 0x%x\n\r",(TWSR & 0xF8));

	//send stop bit
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	//
	// ??BH?? I think this can bo omitted */

	////master receiver mode
	//start twi transmission
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

	//wait for TWCR flag to set indication start is transmitted
	while (!(TWCR & (1 << TWINT)));

	//check to see if start is an error or not
	if ((TWSR & 0xF8) != 0x10)
		printf("start condition error 0x%x\n\r", (TWSR & 0xF8));

	//Load read addres of slave in to TWDR,
	SLA_W = chip_address | 1; //	 SLA_W=0b11010001;
	TWDR = SLA_W;

	//start transmission
	TWCR = (1 << TWINT) | (1 << TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while (!(TWCR & (1 << TWINT)));

	//check to see if ack received
	if ((TWSR & 0xF8) != 0x40)
		printf("third ack problem 0x%x \n\r", (TWSR & 0xF8));
	//	printf("third ack recieved OK 0x%x \n\r",(TWSR & 0xF8));

	TWCR = (1 << TWINT) | (1 << TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while (!(TWCR & (1 << TWINT)));

	//check to see if ack received
	if ((TWSR & 0xF8) != 0x58)
		printf("third ack problem 0x%x \n\r", (TWSR & 0xF8));
	//	printf("third ack recieved OK 0x%x \n\r",(TWSR & 0xF8));

	// transmit STOP condition
	TWCR = (1 << TWINT) | (1 << TWSTO);
	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	//while(!(TWCR & (1<<TWINT)));
	//_delay_ms(10);
	return TWDR;
}
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
//Between Board Communication Functions 

 //master sends commands to slave
int i2c_send()
{
	cli(); //???

	//See atmega datasheet page 270 for example code

	//start twi transmission
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

	//wait for TWCR flag to set indication start is transmitted
	while (!(TWCR & (1 << TWINT)));

	//check TWI status register to see if start is an error, masking prescaler
	if ((TWSR & 0xF8) != 0x08) //0x08 is START???
	{
		printf("start of i2c read error\n\r");
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei();
		return (-1);
	}

	//Load read address of slave in to TWDR,
	TWDR = atmega_slave; //in example this is TWDR0???

	//start transmission
	TWCR = (1 << TWINT) | (1 << TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while(!(TWCR & (1 << TWINT)));

	//check to see if ack received
	if((TWSR & 0xF8) != 0x18) //MT_SLA_ACK 0x18
	{
		printf("1 ack problem 0x%x \n\r",(TWSR & 0xF8));
		TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
		sei();
		return (-1);
	}

	TWDR = output.speed_m3_s;

	TWCR = (1 << TWINT) | (1 << TWEN); //Clear the TWINT bit to start transmission of data

	while(!(TWCR & (1 << TWINT))); //wait for TWINT flag set; data to transmit

	//check for data ack
	if ((TWSR & 0xF8) != 0x28) //0x28 MT_DATA_ACK
	{
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei(); //???
		return (-1);
	}

	TWDR = output.speed_m5_s;

	TWCR = (1 << TWINT) | (1 << TWEN); //start tx of data

	while (!(TWCR & (1 << TWINT)))
		; //wait for data to tx

	//check for data ack
	if ((TWSR & 0xF8) != 0x28)
	{
		printf("3 ack problem is 0x%x\n\r", (TWSR & 0xF8));
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei();
		return (-1);
	}

	TWDR = output.direction_m5_s;

	TWCR = (1 << TWINT) | (1 << TWEN); //start tx of data

	while (!(TWCR & (1 << TWINT)))
		; //wait for data to tx

	//check for data ack
	if ((TWSR & 0xF8) != 0x28)
	{
		printf("4 ack problem is 0x%x\n\r", (TWSR & 0xF8));
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei();
		return (-1);
	}
	TWDR = output.direction_m3_s;

	TWCR = (1 << TWINT) | (1 << TWEN); //start tx of data

	while (!(TWCR & (1 << TWINT)))
		; //wait for data to tx

	//check for data ack
	if ((TWSR & 0xF8) != 0x28)
	{
		printf("5 ack problem is 0x%x\n\r", (TWSR & 0xF8));
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei();
		return (-1);
	}
	TWDR = output.led_s[0];

	TWCR = (1 << TWINT) | (1 << TWEN); //start tx of data

	while (!(TWCR & (1 << TWINT)))
		; //wait for data to tx

	//check for data ack
	if ((TWSR & 0xF8) != 0x28)
	{
		printf("5 ack problem is 0x%x\n\r", (TWSR & 0xF8));
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei();
		return (-1);
	}
	TWDR = output.led_s[1];

	TWCR = (1 << TWINT) | (1 << TWEN); //start tx of data

	while (!(TWCR & (1 << TWINT)))
		; //wait for data to tx

	//check for data ack
	if ((TWSR & 0xF8) != 0x28)
	{
		printf("5 ack problem is 0x%x\n\r", (TWSR & 0xF8));
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei();
		return (-1);
	}
	TWDR = output.led_s[2];

	TWCR = (1 << TWINT) | (1 << TWEN); //start tx of data

	while (!(TWCR & (1 << TWINT)))
		; //wait for data to tx

	//check for data ack
	if ((TWSR & 0xF8) != 0x28)
	{
		printf("5 ack problem is 0x%x\n\r", (TWSR & 0xF8));
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei();
		return (-1);
	}

	TWDR = sleep_mode;

	TWCR = (1 << TWINT) | (1 << TWEN); //start tx of data

	while (!(TWCR & (1 << TWINT)))
		; //wait for data to tx

	//check for data ack
	if ((TWSR & 0xF8) != 0x28)
	{
		printf("5 ack problem is 0x%x\n\r", (TWSR & 0xF8));
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei();
		return (-1);
	}

	TWCR = (1 << TWINT) | (1 << TWSTO); ////Stop condition??? should also include |(1<<TWEN)

	return (0);
}

int i2c_read() //read all inputs from slave via i2c
{
	cli();

	uint8_t data_counter = 0;
	uint8_t temp_var = 0;
	//start twi transmission
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

	//wait for TWCR flag to set indication start is transmitted
	while (!(TWCR & (1 << TWINT)));

	//check to see if start is an error or not
	if ((TWSR & 0xF8) != 0x08)
	{
		printf("start of i2c read error\n\r");
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei();
		return (-1);
	}

	//Load read addres of slave in to TWDR,
	TWDR = atmega_slave | 1; //	 SLA_W=0b11010001;

	//start transmission
	TWCR = (1 << TWINT) | (1 << TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while (!(TWCR & (1 << TWINT)));

	//check to see if ack received
	if ((TWSR & 0xF8) != 0x40)
	{
		printf("first ack problem 0x%x \n\r", (TWSR & 0xF8));
		TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
		sei();
		return (-1);
	}
	//	printf("third ack recieved OK 0x%x \n\r",(TWSR & 0xF8));
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);

	while (data_counter < 6)
	{

		if (data_counter < 5)
		{
			TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
		}
		else
		{
			TWCR = (1 << TWINT) | (1 << TWEN);
		}
		//wait for TWINT flag to se, indicating transmission and ack/nack receive
		while (!(TWCR & (1 << TWINT)))
			;

		//check to see if ack received
		if (((TWSR & 0xF8) != 0x50) && (data_counter != 5))
		{
			printf("data %d ack problem 0x%x \n\r", data_counter, (TWSR & 0xF8));
			TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
			sei();
			return (-1);
		}
		else if (((TWSR & 0xF8) != 0x58) && (data_counter >= 5))
		{
			printf("data %d ack problem 0x%x \n\r", data_counter, (TWSR & 0xF8));
			TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
			sei();
			return (-1);
		}
		//	printf("%d ack recieved  0x%x \n\r",data_counter,(TWSR & 0xF8));
		//	printf("data rx is 0x%x \n\r",TWDR);
		if (data_counter == 0)
		{
			input.switch_tension_s = TWDR;
		}
		else if (data_counter == 1)
		{
			input.switch_S4_s = TWDR;
		}
		else if (data_counter == 2) //get first half of bend
		{
			temp_var = TWDR;
		}
		else if (data_counter == 3) //second half of bend
		{
			input.IR1_s = (TWDR << 8) + temp_var;
		}

		else if (data_counter == 4) //get first half of bend
		{
			temp_var = TWDR;
		}
		else if (data_counter >= 5) //second half of bend
		{
			input.IR2_s = (TWDR << 8) + temp_var;
		}

		data_counter++;
	}

	TWCR = (1 << TWSTO) | (1 << TWINT);

	if (data_counter == 6)
	{

		sei();
		return 0;
	}
	else
	{
		sei();
		return -1;
	}
}
//////////////////////////////////////////////////////////////////

int main(void)
{
	init();
	sei();
	uint8_t state = 0;
	uint8_t count = 0; //counter for open loop control bits - timing of grippers etc.
	uint8_t count2 =0; //2nd count variable 
	unsigned char toggle = 0; //toggle bit
	double bend_angle=0; //used for recording the angle of the robot (and angle of the grippers during detaching)
	double init_angle=0; //used as a reference angle during detaching
	char flipside=1; //direction of flipping, reference with pcb facing and forward (1) being to the right

	while (1){
//////////////////////////////////////////////////////////////////
	// TEST MODE
	// This state is for calibrating and testing electronics, everything essentially "master mode"
		if (PCBTESTMODE){
			
			//detect_pulse();

			/////Blink and pulse
			//_delay_ms(500);
			//setLED(50,50,50);
			//output.vibration_m=1;
			//master_output_update();
	        //_delay_ms(500);
	        //setLED(0,0,0);
			//output.vibration_m=0;
			//master_output_update();
			
			////Sometimes if statements are useful
			/* 			 
			output.speed_m3_m = 0;
			if (input.switch_S4_m == 0){
				setLED(50,50,50);
				output.speed_m5_m = 100;
				output.direction_m5_m=0;
			}
			else{
				output.speed_m5_m = 0;
				setLED(0,0,0);
				}
			*/
			
			////Motor Testing
			//output.speed_m3_m=225;
			//output.direction_m3_m=0;
			//output.direction_m5_m=1; // 0 positive
			//output.speed_m5_m=225;
			//output.vibration_m=1;

			////Sensor Testing
			//printf("m %d %d %d \n\r",input.accell_m[0],input.accell_m[1], input.accell_m[2]);
			//printf("s %d %d %d \n\r",input.accell_s[0],input.accell_s[1], input.accell_s[2]);
			printf("IR %d %d \n\r", input.IR1_s, input.IR2_s);
			//input.IR2_m=get_IR_U5();
			//input.IR1_m=get_IR_Flex_U1513();
			//printf("IR %d \n\r", input.IR1_m); //for bend sensor reading only - note change function name to reflect.
			
			////Updates
			i2c_send();
			master_output_update();
			i2c_read();
			master_input_update();
			_delay_ms(20);
		}

//////////////////////////////////////////////////////////////////
	// MASTER EXPERIMENT MODE
	// This is the state for running experiments through the master board
		else if(MASTER){

		/////First update everything - needed for all states
			i2c_send();
			master_output_update();
			i2c_read();
			master_input_update();
			_delay_ms(20);
			//setLED(0,0,0);
		/////
			
			switch (system_state){
				case SETUP: //Experiment Set-up and Reset (in case the robot gets stuck)
					//printf("state %d \n\r", state);
					switch(state){
						case UNWIND: //unwind motors on command
							output.led_m[1]=20;
							output.led_s[1]=20;
							output.speed_m5_m=0;
							output.speed_m5_s=0;
							output.speed_m3_m=0;
							output.speed_m3_s=0;

							if((input.switch_S4_m==0)&(input.switch_S4_s==0)){
								state=REWIND;
								count=0;
								toggle=0;
								output.led_m[0]=0;
								output.led_s[0]=0;
								output.led_m[1]=0;
								output.led_s[1]=0;
								output.speed_m5_m=0;
								output.speed_m5_s=0;
								}
							else if(input.switch_S4_m==0)
							{
								output.led_m[0]=30;
								output.led_m[1]=0;	
								output.direction_m5_m=1;
								output.speed_m5_m=100;
							}
							else if(input.switch_S4_s==0)
							{
								output.led_s[0]=30;	
								output.led_s[1]=0;
								output.direction_m5_s=1;
								output.speed_m5_s=100;
							} 
							break;
						case REWIND: //wind the motors

							//default motors off
							output.direction_m5_m=0;
							output.direction_m5_s=0;

							//Make sure the control switch is unpressed before checking again. 
							if ((input.switch_S4_m==1)&(input.switch_S4_s==1)){
								toggle=1;
								//printf("toggle");
								}
							//Main functionality - rewind motors until both tension switches are pressed. 
							if (toggle==1){
								if(input.switch_tension_s==0){
									output.speed_m5_m=60;
									}
								else {
									output.speed_m5_m=0;
									}
								if(input.switch_tension_m==0) {
									output.speed_m5_s=60;
									}
								else {
									output.speed_m5_s=0;
								}	
							}

							//switch states conditions only if tension switches are pressed. 
								//Move on to next state if master switch is pressed
								//Go back to unwind state if S4 is pressed. Should this be the default?
							if ((input.switch_tension_s==1)&(input.switch_tension_m==1)&(toggle=1)){
								if(input.switch_S4_m==0){
									state=SLAVE_GRIPPER;
									toggle=0;
									_delay_ms(2000);
									break;
								}
								else if(input.switch_S4_s==0){
									state=UNWIND;
									toggle=0;
									_delay_ms(2000);
									break;
								}
							}
							break;
						case SLAVE_GRIPPER: //attach or detach slave side
							output.led_m[0]=20;
							output.led_s[0]=0;
							output.led_s[1]=0;
							//Both switches together move to the next state
							if ((input.switch_S4_m==0)&&(input.switch_S4_s==0)){
								output.speed_m3_s=0;
								output.speed_m3_m=0;
								output.led_s[0]=0;
								output.led_s[2]=0;
								output.led_m[0]=0;

								//Update outputs before the state change and delay. 
								i2c_send();
								master_output_update();

								state=MASTER_GRIPPER;
								_delay_ms(2000);
								break;
								}
							else if (input.switch_S4_m==0){
								output.led_s[0]=20;
								output.led_s[2]=0;
								output.direction_m3_s=0;
								output.speed_m3_s=0.7*GRIPPER_SPD;
								}
							else if (input.switch_S4_s==0){
								output.led_s[0]=0;
								output.led_s[2]=20;
								output.direction_m3_s=1;
								output.speed_m3_s=GRIPPER_SPD;
								}
							else{
								output.led_s[0]=0;
								output.led_s[2]=0;
								output.speed_m3_s=0;
								}
							break;
						case MASTER_GRIPPER: //attach or detach master side
							output.led_s[0]=20;
							if ((input.switch_S4_m==0)&&(input.switch_S4_s==0)){
								output.speed_m3_s=0;
								output.speed_m3_m=0;
								output.led_m[0]=0;
								output.led_m[2]=0;
								output.led_s[0]=0;

								//Update outputs before the state change and delay. 
								i2c_send();
								master_output_update();

								state=FLIPPING1;
								system_state=FLIP;
								_delay_ms(9000);
								break;
							}
							else if (input.switch_S4_m==0){
								output.led_m[0]=20;
								output.led_m[2]=0;
								output.direction_m3_m=0;
								output.speed_m3_m=0.7*GRIPPER_SPD;
							}
							else if (input.switch_S4_s==0){
								output.led_m[0]=0;
								output.led_m[2]=20;
								output.direction_m3_m=1;
								output.speed_m3_m=GRIPPER_SPD;
							}
							else{
								output.led_m[0]=0;
								output.led_m[2]=0;
								output.speed_m3_m=0;
							}
							break;
					}
				break; //end SETUP
				case FLIP: //Normal locomotion
					switch(state){
						case FLIPPING1:	//flipping 
							//flipside - define by the moving gripper
							//flipside 0: slave side moving, master goes forward, slave side back.
							//flipside 1: Master side moving, slave side goes forward, master back
  							
							//Light up the moving gripper
							output.led_m[0]=flipside*20;
							output.led_s[0]=(!flipside)*20;

							//spin the opposite dock motor to prevent attaching
							output.direction_m3_m=flipside; //1 to go backwards
							output.direction_m3_s=(!flipside); //1 to go backwards

							output.speed_m3_m=flipside*GRIPPER_SPD; 
							output.speed_m3_s=(!flipside)*GRIPPER_SPD;
							
							//Set direction for bend motors, see guide above
							output.direction_m5_m=flipside; //0 if forward, 1 is back
							output.direction_m5_s=(!flipside); //0 if forward, 1 is back

							if (toggle<2){ //Toggle condition prevents the robot from attaching to a surface too early
								
								bend_angle=get_accel_diff(); //Track the robot's angle

								//The robot first unwinds at a set speed to try to undo any overtensioning.
								//This could possibly be changed to an unwind until no tension condition?
								if(toggle==0){
									//Set the speeds depending on flipside. 
									output.speed_m5_m= flipside ? UNWINDSPD:WINDSPD; //flipside 1 will unwind
									output.speed_m5_s= flipside ? WINDSPD: UNWINDSPD; //flipside 1 will wind

									//Counter for unwind at set speed
									if (count>12){
										toggle=1;
										count=0;
									}
									count++;
								}

								//After set time, switch to unwinding only if tension. 
								else{
									flipbend(flipside, 10); //after set time period, flip normally	

									//This is a safeguard condition if the robot does not recognize/doesn't go thru neutral
									//If the robot is fully bent, it can search for a surface
									if((bend_angle>170)&&(bend_angle<200)){
										toggle=2;
										//printf("toggle");
										output.led_m[2]=flipside*20;
										output.led_s[2]=(!flipside)*20;
									}
	
									//Neutral/0 degree position check
									if(bend_angle<10){
										toggle=2;
									}
								}
							}
							//Check for surface
							if (toggle==2){
								//Check for IR sensors or manual control to switch states
								//Attach if switch is pressed or 
								//if 1. one IR is smaller than the close threshold AND 2. Both IRs are smaller than the far threshold.
								if (flipside ? (input.switch_S4_m==0)|(
									((input.IR1_m<CLOSE_ATT_THRESH)|(input.IR2_m<CLOSE_ATT_THRESH))&
									((input.IR1_m<FAR_ATT_THRESH)&(input.IR2_m<FAR_ATT_THRESH))):
									(input.switch_S4_s==0)|(
									((input.IR1_s<CLOSE_ATT_THRESH)|(input.IR2_s<CLOSE_ATT_THRESH))&
									((input.IR1_s<FAR_ATT_THRESH)&(input.IR2_s<FAR_ATT_THRESH)))){
									
									count++; //Account for noise, make sure the surface is detected twice. 
									if (count>2){
										//Turn off motors and LEDS
										zero_motors();
										LEDsOff();
										//Zero Everything
										count=0;
										toggle=0;
										//Switch States
										state=ATTACHING;
										break;
										}
									}
								else{
									flipbend(flipside,10);
								}
							}

						break;
						case ATTACHING:
							//LED indicates state
							output.led_m[1]=flipside*20;
							output.led_s[1]=(!flipside)*20;

							//Make sure bend motors are off
							output.speed_m5_s=0;
							output.speed_m5_m=0;

							//Set direction and speed for grippers, depending on flipside. 
							output.direction_m3_m=0;
							output.direction_m3_s=0;

							output.speed_m3_m=flipside*0.7*GRIPPER_SPD;
							output.speed_m3_s=(!flipside)*0.7*GRIPPER_SPD;
							//Just run the grippers forward for a set time.							
							if (count>200){
								//Zero everything
								count=0;
								output.led_m[1]=0;
								output.led_s[1]=0;
								output.speed_m3_m=0;
								output.speed_m3_s=0;

								//Prep for next state

								flipside=(!flipside); //switch sides with moving gripper

								//check current orientation of boards to have an initial bend position as a reference for detaching
								init_angle=flipside ? rad2deg(atan2((input.accell_m[0]),(input.accell_m[1]))):rad2deg(atan2((input.accell_s[0]),(input.accell_s[1])));
								
								state=DETACHING;	//New state
								
								break;
							}
							count++;
						break;
						case DETACHING: //detaching the master side
							//Detaching works by running the grippers backward for a set period of time and then trying to flip

							//Keep bend motors default off
							output.speed_m5_m=0;
							output.speed_m5_s=0;
							//Output LED on the moving gripper
							output.led_m[2]=flipside*20;
							output.led_s[2]=(!flipside)*20;
							//set dock motors to go backwards
							output.direction_m3_m=1;
							output.direction_m3_s=1;

							//set motors based on flipside - 0 = slave side moving gripper, 1 = master side moving
							output.speed_m3_m=flipside*GRIPPER_SPD;
							output.speed_m3_s=(!flipside)*GRIPPER_SPD;

							count++;
							//printf("%d \n\r", count); 

							if (toggle==0){
								//Check current angle of detaching gripper
								bend_angle= flipside ? rad2deg(atan2((input.accell_m[0]),(input.accell_m[1]))):rad2deg(atan2((input.accell_s[0]),(input.accell_s[1])));
								//find the change in angle
								bend_angle=abs(bend_angle-init_angle);
								//convert in case the angle went from 360 to 0
								if (bend_angle>300){
								bend_angle=(360-bend_angle);
								}
								//Angle should change at least 30 degrees before robot can consider itself detached.
								if (bend_angle>30){
									toggle=1;
								}
							}
							//if bend angle has changed (toggle condition)
							//change states once no surface is detected (or manually with switches)
							else if (flipside ? (input.switch_S4_m==0)|((input.IR1_m+input.IR2_m)>DETACH_THRESH):
								(input.switch_S4_s==0)|((input.IR1_s+input.IR2_s)>DETACH_THRESH)){
								toggle++; //use toggle to ensure that detach thresh reached at least 2 times.
								if (toggle>=3){
									//change state
									state=FLIPPING1;
									//Zero motors
									zero_motors();
									//Zero LEDS
									output.led_m[2]=0;
									output.led_s[2]=0;
									//Reset counter and toggle
									count=0;
									toggle=0;
									count2=0;
									_delay_ms(100); //just give the robot some time, probably doesn't need to be this long
									break;
								}
							}
							//if you've been trying to detach for a while, unwind so as to not overtension.
							if (count2>NUM_DETACH_FLIP_ATTEMPTS){
								output.direction_m5_m=1; //set motors to unwind
								output.direction_m5_s=1;
								if((input.switch_tension_s==1)&(input.switch_tension_m==1)){
									output.speed_m5_m=(!flipside)*60;
									output.speed_m5_s=flipside*60;
									}
								else {
									output.speed_m5_m=0;
									output.speed_m5_s=0;
									count2=0;
									}
							}
							//otherwise just try to unscrew for 20 ticks
							else if (count>22){
								//then try to flip for 4 ticks
								if (count<26){
									flipbend(flipside,7);
								}
								else{
								//reset count
								count=0;
								count2++; //record how many rounds we've been trying to detach overall. 
								}
							}
						break;
					}
				break; //end FLIP
			}
		}
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
		// SLAVE MODE
		// The slave board just updates outputs and sends sensor values.
		else{
			//printf("slave mode");
			static uint8_t tx_data_counter=0; //counter for slave inputs
			static uint8_t rx_data_count=0; //counter for slave outputs
			if((TWCR & (1<<TWINT))) 
			{
				//printf("status 0x%x , count %d\n\r",(TWSR & 0xF8),rx_data_count);
				//slave receives outputs from master
				if((TWSR & 0xF8)==0x60)
				{
					TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT); //???
					rx_data_count=0;
					//printf("slave rx address start\n\r");
				}
				else if((TWSR & 0xF8)==0x80) // ??? Ok to start data transfer
				{
					if(rx_data_count==0)
					{
						//Motor M3 speed
						output.speed_m3_s=TWDR; 
					}
					else if(rx_data_count==1)
					{
						//Motor M5 dock speed
						output.speed_m5_s=TWDR;
					}
					else if(rx_data_count==2)
					{
						//Motor M5 dock direction

						output.direction_m5_s=TWDR;
						set_M5(output.direction_m5_s, output.speed_m5_s);

					}
					else if(rx_data_count==3) //M3 direction
					{
					//M3 direction
					output.direction_m3_s=TWDR;
					set_M3(output.direction_m3_s, output.speed_m3_s);

					}
					else if(rx_data_count==4)
					{
						output.led_s[0]=TWDR;
					}
					else if(rx_data_count==5)
					{
						output.led_s[1]=TWDR;
					}
					else if(rx_data_count==6)
					{
						output.led_s[2]=TWDR;
						setLED(output.led_s[0],output.led_s[1],output.led_s[2]);
					}
					else if(rx_data_count==7)
					{
						power_state=TWDR;
					
					}
				
					//printf("slave rx--- data = %d, count=%d, status 0x%x\n\r",TWDR,rx_data_count,(TWSR & 0xF8));
					TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT);
					rx_data_count++;
				}
				else if((TWSR & 0xF8)==0xa0)
				{
					TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT);
					//printf("end frame detected\n\r");
				
				}
				else if((TWSR & 0xF8)==0xa8)//twi slave->master initalized 
				{
					//??? Why is he doing the tension switch separately from everything else?
					//Tension Switch
					tx_data_counter=1;
					//TWDR=switch_tension1();
					TWDR=get_switch_input(STension_port, STension_pin);
					TWCR=(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
				
				}
				else if(((TWSR & 0xF8)==0xb8))
				{
												
					if(tx_data_counter==1) //Dock/Control Switch S4
					TWDR=get_switch_input(S4_port, S4_pin);
					else if(tx_data_counter==2) //IR sensor pt 1
					TWDR=get_IR_Flex_U1513()&0xff;
					else if(tx_data_counter==3) //IR sensor pt 2
					TWDR=(get_IR_Flex_U1513()>>8)&0xff;
					else if(tx_data_counter==4) //IR sensor pt 1
					TWDR=get_IR_U5()&0xff;
					else if(tx_data_counter==5) //IR sensor pt 2
					TWDR=(get_IR_U5()>>8)&0xff;

					TWCR=(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
					tx_data_counter++;	
				
				}
				else if(((TWSR & 0xF8)==0xc0))
				{
						
					TWDR=1; //??? What is this? To reset back to receive mode?
					TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA);
					tx_data_counter++;
					
				
				}
				else
				{
					printf("bus error? i2c data 0x%x\n\r",(TWSR & 0xF8));
					TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
				}	

			}
			if((power_state==0)){
				
				printf("sleeping\n\r");
				int i;
				for(i=50;i>0;i--)
				{
					setLED(i,0,0);
					_delay_ms(10);
				}			

				//same as in master code, turn everything (Hbridge) off (and make them inputs?)
				DDRB &= ~(1<<6); 
				PORTB &= ~(1<<6);
				DDRD &= ~(1<<5);
				PORTD &= ~(1<<5);

				DDRB &= ~(1<<7);
				PORTB &= ~(1<<7);
				DDRD &= ~(1<<6);
				PORTD &= ~(1<<6);
			
				//voltage reg off
				DDRC &= ~(1<<0);
				PORTC &= ~(1<<0);

				sei();     
				TWCR |= (1<<TWIE);
		        SMCR = (1<<SE);
		        asm volatile("sleep\n\t");
				SMCR = 0;
				DDRC |= (1<<0); //voltage reg back on
				PORTC |= (1<<0);

				printf("wakeup\n\r");
			//	PRR |= (1<<PRTWI);
				init();
				toggle_wakeup=1;
		        
				power_state=1;
			}
			else {
				if(toggle_wakeup==1){
					toggle_wakeup=0;
					int i;
					for(i=0;i<50;i++)
					{
						setLED(0,i,0);
						_delay_ms(10);
					}
		
					_delay_ms(500);
					setLED(0,0,0);
				}
				//printf("wakeup\n\r");
			}
		}	//end slave code
	} //end of while
} //end of main
//////////////////////////////////////////////////////////////////
		//THIS DELAY IS NECESSARY FOR ALL MODES OF THE ROBOT (actually may not be necessary for slave???)
		//_delay_ms(20); //you can adjust this delay.. eventually if too small it may cause problems

void master_output_update() //motor updates
{
	set_M5(output.direction_m5_m, output.speed_m5_m);
	set_M3(output.direction_m3_m, output.speed_m3_m);

	if (output.vibration_m == 1)
	{
		DDRB |= (1 << 0); //Vibration motor PB0
		PORTB |= (1 << 0);
	}
	else
	{
		DDRB |= (1 << 0); //Vibration motor PB0
		PORTB &= ~(1 << 0);
	}
	setLED(output.led_m[0],output.led_m[1],output.led_m[2]);
}

void master_input_update()
{
	input.switch_S4_m = get_switch_input(S4_port, S4_pin); //0 is button pressed.
	input.switch_tension_m = get_switch_input(STension_port, STension_pin); //1 is button pressed.
	//	input.bend_m=get_bend();
	input.IR1_m=get_IR_Flex_U1513();
		//printf("%d \n\r",input.IR1_m);
	input.IR2_m=get_IR_U5();
		//printf("%d \n\r",input.IR2_m);

	get_IMU_measurement(1); 	// Get accel data from master side
	get_IMU_measurement(0);		// Get accel data from slave side
}

///////////////////INPUT READ FUNCTIONS

void get_IMU_measurement(unsigned char self)
{
	// Measurement data is stored in two’s complement and Little Endian format.
	// Measurement range of each axis is from -32752 ~ 32752 decimal in 16-bit output.
	// 0x8010 is -32752, 0x7ff0 is 32752

	//if master (or slave board in testing mode) is reading for itself, uses IMU_ADDRESS defin. Otherwise, master takes slave address. 
	int addrs= (self ? IMU_ADDRESS:accell_slave_addrs); 


	i2c_write_accell(addrs, PWR_MGMT_1, 0);
	int x = ((i2c_read_accell(addrs, ACCEL_XOUT_H) << 8) & 0xff00) + (i2c_read_accell(addrs, ACCEL_XOUT_L) & 0x00ff);
	int y = ((i2c_read_accell(addrs, ACCEL_YOUT_H) << 8) & 0xff00) + (i2c_read_accell(addrs, ACCEL_YOUT_L) & 0x00ff);
	int z = ((i2c_read_accell(addrs, ACCEL_ZOUT_H) << 8) & 0xff00) + (i2c_read_accell(addrs, ACCEL_ZOUT_L) & 0x00ff);

	x=convert_negatives(x);
	y=convert_negatives(y);
	z=convert_negatives(z);

	// Update values input to MCU from IMU
	if (self) {
		input.accell_m[0] = x;
		input.accell_m[1] = y;
		input.accell_m[2] = z;
		}
	else {
		input.accell_s[0] = x;
		input.accell_s[1] = y;
		input.accell_s[2] = z;
		}
}

int switch_power(void) //Rewrite more efficiently? e.g. as a macro? 
//NOTE THAT MACRO FOR SWITCH INPUT NOT CURRENTLY WORKING FOR POWER SWITCH????
{

	if ((PIND & (1 << 3)) != 0) //power switch connected to PD3, low when connected, so when not low, switch is not connected.
	{
		return (0);
	}
	else
	{
		return (1);
	}
}

int get_IR_Flex_U1513(void)
{
	//initalize adc
	ADMUX &= (1 << REFS0);				 //choose reference and clear analog pins, AVcc chosen as AREF
	ADCSRB |= (1 << ACME);
	ADCSRA &= ~(1 << ADEN);
	ADMUX |= 0x01;				 //choose analog pin ADC1 connected to Flex/IR2
	ADCSRA = (1 << ADEN) | (1 << ADPS0); //ADEN set makes ADC enabled, ADPS0..2 selects ADC clock vs system clock, here divided by 2

	//when the following code was in main, while loop started here
	ADCSRA |= (1 << ADSC); //start adc conversion to sample sensor with led off
	while ((ADCSRA & (1 << ADSC)) != 0); //busy wait for conversion to end

	//		printf("%d \n\r",ADCW);

	return (ADCW);
}

int get_IR_U5(void)
{
	//initalize adc
	ADMUX &= (1 << REFS0); //choose reference and clear analog pins
	ADCSRB |= (1 << ACME);
	ADCSRA &= ~(1 << ADEN);
	ADMUX |= 0x02;						 //choose analog pin ADC2 connected to IR1
	ADCSRA = (1 << ADEN) | (1 << ADPS0); //set up a/d

	//when the following code was in main, while loop started here
	ADCSRA |= (1 << ADSC); //start adc conversion to sample sensor with led off
	while ((ADCSRA & (1 << ADSC)) != 0); //busy wait for conversion to end
	//	printf("%d \n\r",ADCW);

	return (ADCW);
}

int get_bend(void)
{	
	     //initalize adc
		ADMUX &= (1<<REFS0); //choose reference and clear analog pins
		ADMUX |= 0x01;//voltage reference selection AVcc at AREF and choose analog pin - ADC1
		ADCSRA = (1<<ADEN) | (1<<ADPS0); //set up a/d (aden adc enable, )

		//when the following code was in main, while loop started here
		ADCSRA |= (1<<ADSC);//start adc conversion to sample sensor with led off
		while((ADCSRA&(1<<ADSC))!=0);//busy wait for converstion to end

//			printf("%d \n\r",ADCW);	

		return(ADCW);

}

//////////////////OUTPUT FUNCTIONS

void setLED(unsigned char red, unsigned char green, unsigned char blue)
{
	//Bit banging 20-600kHz
	//Send LSB first

	unsigned char array[4] = {led_wrt_cmd, red, blue, green};

	for (char byte = 0; byte <= 3; byte++) //Go through loop 4x - first write command, then each rgb
	{
		for (unsigned char bit = 0; bit <= 7; bit++)
		{
			//bit initiation
			set(led_port, led_pin);
			_delay_us(2);
			clear(led_port, led_pin);
			_delay_us(3);

			if (array[byte] & (led_pin >> bit))
				set(led_port, led_pin);
			else
				clear(led_port, led_pin);
			_delay_us(10);
			clear(led_port, led_pin);

			_delay_us(5); //~50kHz
		}
	}

	_delay_us(80); //End of Sequence
}

void set_M5(unsigned char dir, unsigned char speed)
{
	//	output(M5_port_dir, M5_pin); ??? Do we need to set the outputs again after init? Seems to work w/o and may be unnecessary
	if (dir == 0)
	{							//Note 0 is positive
		clear(M5_port, M5_pin); //set dock motor (M5) PB6 low - (in2/in4 on hbridge)
		M5_PWM = speed;			//sets PWM for OCR0B (PD5) - in1/in3 on hbridge - motor goes forward on high portion
	}
	else
	{
		set(M5_port, M5_pin);
		M5_PWM = 255 - speed; //sets PWM for OCR0B (PD5) - in1/in3 on hbridge - motor goes backward on low portion
	}
}

void set_M3(unsigned char dir, unsigned char speed)
{
	//	output(M3_port_dir, M3_pin); ??? Do we need to set the outputs again after init? Seems to work w/o and may be unnecessary
	if (dir == 0)
	{							//Note 0 is positive
		clear(M3_port, M3_pin); //set dock motor (M5) PB6 low - (in2/in4 on hbridge)
		M3_PWM = speed;			//sets PWM for OCR0B (PD5) - in1/in3 on hbridge - motor goes forward on high portion
	}
	else
	{
		set(M3_port, M3_pin);
		M3_PWM = 255 - speed; //sets PWM for OCR0B (PD5) - in1/in3 on hbridge - motor goes backward on low portion
	}
}

/////////////////utility functions 

//function for detecting another robot's IMU pulses
 void detect_pulse(){
	 int i=0;
	 int m_samples[IMU_SAMPLE_NUM][3];
	 int s_samples[IMU_SAMPLE_NUM][3];

	 int delta=0;
	 for (i=0;i<IMU_SAMPLE_NUM;i++){
		 get_IMU_measurement(1);
		 get_IMU_measurement(0); 
		 //This will load IMU measurement into accel input, a bit sloppy think about fixing??
		 //Also can eventually change to just delta threshold recording to save memory. 
		 m_samples[i][0]=input.accell_m[0];
		 m_samples[i][1]=input.accell_m[1];
		 m_samples[i][2]=input.accell_m[2];
		 s_samples[i][0]=input.accell_s[0];
		 s_samples[i][1]=input.accell_s[1];
		 s_samples[i][2]=input.accell_s[2];

		 if (i>0){
			 //Check if any of the values recorded differe from previous by more than the threshold. 
			 if ((abs(m_samples[i][0]-m_samples[i-1][0])>IMU_DELTA_THRESH)|(abs(m_samples[i][1]-m_samples[i-1][1])>IMU_DELTA_THRESH)|
			 (abs(m_samples[i][2]-m_samples[i-1][2])>IMU_DELTA_THRESH)|(abs(s_samples[i][0]-s_samples[i-1][0])>IMU_DELTA_THRESH)|
			 (abs(s_samples[i][2]-s_samples[i-1][2])>IMU_DELTA_THRESH)|(abs(s_samples[i][2]-s_samples[i-1][2])>IMU_DELTA_THRESH))
		 	delta++;
		 }
	 }
	 //for (i=0;i<IMU_SAMPLE_NUM;i++){
	 //printf("%d %d %d \n\r", m_samples[i][0], m_samples[i][1], m_samples[i][2]);
	 //}
	if (delta>3){
		setLED(20,20,20);
	}
	else{
		setLED(0,0,0);
	}

 }


//Function for normal flipping, dependent on tension switches 

void flipbend(char side, char p){
	//p a percentage to vary speed.

	//Set direction for bend motors, see guide above
	output.direction_m5_m=side; //0 if forward, 1 is back
	output.direction_m5_s=(!side); //0 if forward, 1 is back
 
	if (side==0){ 
		if(input.switch_tension_m==1){
			output.speed_m5_s=UNWINDSPD*p/10;
		}
		else {
			output.speed_m5_s=0;
		}
	output.speed_m5_m=WINDSPD*p/10;	
	}
	else{ //slave side
		if(input.switch_tension_s==1){
			output.speed_m5_m=UNWINDSPD*p/10;
		}
		else {
			output.speed_m5_m=0;
		}
	output.speed_m5_s=WINDSPD*p/10;
	}
}


//Find the angle between the boards (amount of bend) independent of orientation. 

double get_accel_diff(void){
		double diff=abs(atan2((input.accell_s[0]),(input.accell_s[1]))*180/3.14159-atan2((input.accell_m[0]),(input.accell_m[1]))*180/3.14159);
		return(diff);
}

//Zero Motors

void zero_motors(void){
	output.speed_m3_m=0;
	output.speed_m3_s=0;
	output.speed_m5_m=0;
	output.speed_m5_s=0;
}

void LEDsOff(void){
	output.led_m[0]=0; //master side
	output.led_m[1]=0;
	output.led_m[2]=0;
	output.led_s[0]=0; //slave side
	output.led_s[1]=0;
	output.led_s[2]=0;
}
