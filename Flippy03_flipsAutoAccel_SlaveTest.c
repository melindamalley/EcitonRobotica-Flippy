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

#define MASTER 0   // 1 for Master, 0 for Slave 

#define FOSC 8000000 // oscillator clock frequency, page 164 datasheet, internal RC oscillator clock source selected by fuse bits
#define BAUD 9600 // baud rate desired
#define MYUBRR ((FOSC / (16L * BAUD)) - 1) // used to set the UBRR high and low registers, Usart Baud Rate registers, for formula see datasheet page 146
// IMU information
#define accell_slave_addrs 0b11010000									// IMU address on slave board,  [7bit i2c embedded address from IMU chip datasheet,0] = 0xd0
#define accell_master_addrs 0b11010010									// IMU address on master board, [7bit i2c embedded address from IMU chip datasheet,0] = 0xd2
#define IMU_ADDRESS (MASTER ? accell_master_addrs : accell_slave_addrs) // (Condition? true_value: false_value), for unifying master/slave code

// IMU chip registers
// accelerometer registers 0x3B to 0x40 for MPU-9250 (Mar 2019 order), 0x2D to 0x32 for ICM-20948 (Dec 2018 order)
#define ACCEL_XOUT_H 0x3B // 0x2D //for ICM-20948
#define ACCEL_XOUT_L 0x3C // 0x2E //for ICM-20948

#define ACCEL_YOUT_H 0x3D // 0x2F //for ICM-20948
#define ACCEL_YOUT_L 0x3E // 0x30 //for ICM-20948

#define ee_POWER_STATE 0x00 // not used to be removed
#define accell_slave  0b11010000 //IMU as slave address if accel is on slave board
#define	accell_master 0b11010010 //IMU as address if accel is on master board
#define atmega_slave 0xf0 // slave address, to be renamed to something more meaningful

#define PWR_MGMT_1 0x6B // should be set to 0 for Accel to be run, see datasheet page 31

// system states
//#define TEST	0x00
//#define EXPERIMENT	0x01
#define SETUP 0x10
#define FLIP 0x20

#define UNWIND 0x00

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

#define get_switch_input(port, pin) ((port & (1 << pin)) >> pin) //bit shift function to get input from switches

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

#define windspd 130
#define unwindspd 220
#define gripperspd 180


void setLED(unsigned char red, unsigned char green, unsigned char blue);
void set_M3(unsigned char dir, unsigned char speed);
void set_M5(unsigned char dir, unsigned char speed);
int switch_power(void);
int switch_tension1(void); //rename
int switch_S4(void); //rename
int get_bend(void);
int get_IR_Flex_U1513(void);
int get_IR_U5(void);
void get_IMU_measurement(void);
void master_output_update(void);
void master_input_update(void);
void flipbend(char side, char p);
double get_accel_diff(void);

void init(void);
int i2c_send(void);

// system state
uint8_t system_state = SETUP;

// Robot Inputs/Sensors

struct inputs{
	unsigned char switch_power;
	unsigned char switch_tension_m; //rename
	unsigned char switch_tension_s; //rename
	unsigned char switch_dock_m;
	unsigned char switch_dock_s;
	int bend_s;
	int bend_m;
	int IR1_m;
	int IR2_m;
	int accell_m[3];
	int accell_s[3]; 
 
};

struct outputs{
	uint8_t speed_bend_m3_m;
	uint8_t speed_bend_m3_s;
	uint8_t speed_dock_m5_m;
	uint8_t speed_dock_m5_s;
	uint8_t direction_dock_m5_m;
	uint8_t direction_dock_m5_s;
	uint8_t direction_bend_m3_m;
	uint8_t direction_bend_m3_s;
	uint8_t led_m[3];
	uint8_t led_s[3];
	uint8_t vibration_m;

};

struct outputs output;
struct inputs input;

// define helper variables
static uint8_t power_state;
static uint8_t toggle_wakeup; //variable for power/sleep function
static uint8_t sleep_mode;

//setup for printf
int uart_putchar(char c, FILE *stream) { 
    if (c == '\n') 
        uart_putchar('\r', stream); 
    loop_until_bit_is_set(UCSR0A, UDRE0); 
    UDR0 = c; 

    return 0; 
}
/* #define FDEV_SETUP_STREAM(put, get, rwflag) from stdio.h	for use when programming in C, older Atmel Studio projects
Initializer for a user-supplied stdio stream. 
This macro acts similar to fdev_setup_stream(), used as the initializer of a variable of type FILE.
*/
//FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE mystdout;

void ioinit (void) { 
    // Set baud rate
    UBRR0H = (unsigned char)(MYUBRR >> 8);
    UBRR0L = (unsigned char)MYUBRR;
    UCSR0B = (1<<TXEN0);
    
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
	ADMUX = (1 << REFS0); //|(1<<MUX0);//choose analog pin  
	ADCSRA = (1 << ADEN) | (1 << ADPS0); //set up a/d

			//enable power switch interrupt
    
	//enable interrupt INT0/PCINT18/PD2, used by U1/IMU
	DDRD &= ~(1 << 2);				 // DDRxn reset, PD2 configured as input
	PORTD |= 1 << 2;				 // PORTxn set when pin configured as input, PD2 pull-up resistor activated
	EIMSK = 1 << 1;					 // External Interrupt Mask Register (EIMSK) bit 1 set, INT0 enabled
	EICRA = 1 << ISC01 | 1 << ISC00; // External Interrupt Control Register A (EICRA), ISCxx Interrupt Sense Control bits, INT0 set to trigger on rising edge
	
    TWBR = 0x04; //twi bit rate set

	power_state = 1;
	sleep_mode = 1;
	toggle_wakeup = 0;
}
// AVR TWI is byte-oriented and interrupt based
// ISR for the 2 wire Serial interface
ISR(TWI_vect)
{
cli();
	printf("twi int\n\r");
	
	TWCR &= ~(1<<TWIE);	 // Two Wire Control Register (TWCR),
	// TWEN bit = 1 to enable the 2-wire serial interface
	// TWINT bit = 1 to clear the TWINT flag
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
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
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //start condition for master transmitter mode

	//wait for TWCR flag to set indication start is transmitted
	while(!(TWCR &(1<<TWINT)));

	//check TWI status register to see if start is an error, masking prescaler
	if((TWSR & 0xF8) != 0x08){ //0x08 is START???
	printf("start of i2c read error\n\r");
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO); //???
	sei(); //???
	return (-1);
	}

	//Load read address of slave in to TWDR, 
	TWDR=atmega_slave ;	 //in example this is TWDR0???

	//Clear TWINT bit to start transmission
	TWCR=(1<<TWINT)|(1<<TWEN); //in example this is TWCR0???

	//wait for TWINT flag to set, indicating transmission and ack/nack receive
	while(!(TWCR & (1<<TWINT)));

	//check to see if ack received
	if((TWSR & 0xF8) != 0x18)	{ //MT_SLA_ACK 0x18?
	printf("1 ack problem 0x%x \n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}
	

	TWDR=output.speed_bend_m3_s;

	TWCR = (1<<TWINT)|(1<<TWEN);//Clear the TWINT bit to start transmission of data

	while(!(TWCR&(1<<TWINT)));//wait for TWINT flag set; data to transmit

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{ //0x28 MT_DATA_ACK
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}



	TWDR=output.speed_dock_m5_s;

	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{
	printf("3 ack problem is 0x%x\n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}

	TWDR=output.direction_dock_m5_s;

	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{
	printf("4 ack problem is 0x%x\n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}
	TWDR=output.direction_bend_m3_s;

	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{
	printf("5 ack problem is 0x%x\n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}
	TWDR=output.led_s[0];

	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{
	printf("5 ack problem is 0x%x\n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}
	TWDR=output.led_s[1];

	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{
	printf("5 ack problem is 0x%x\n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}
	TWDR=output.led_s[2];

	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{
	printf("5 ack problem is 0x%x\n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}

	TWDR=sleep_mode;

	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{
	printf("5 ack problem is 0x%x\n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}

	TWCR = (1<<TWINT)|(1<<TWSTO); //Stop condition???

	return(0);
}

int i2c_read()//read all inputs from slave via i2c
{
	cli();
	
	uint8_t data_counter=0;
	uint8_t temp_var=0;
	//start twi transmission
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	//wait for TWCR flag to set indication start is transmitted
	while(!(TWCR &(1<<TWINT)));

	//check to see if start is an error or not
	if((TWSR & 0xF8) != 0x08){
	printf("start of i2c read error\n\r");
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}

	//Load read addres of slave in to TWDR, 
	TWDR=atmega_slave | 1;//	 SLA_W=0b11010001;

	//start transmission
	TWCR=(1<<TWINT)|(1<<TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while(!(TWCR & (1<<TWINT)));

	//check to see if ack received
	if((TWSR & 0xF8) != 0x40)	{
	printf("first ack problem 0x%x \n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}
//	printf("third ack recieved OK 0x%x \n\r",(TWSR & 0xF8));
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	
	while(data_counter<4)
	{

		if(data_counter<3)
		{
		TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
		}
		else
		{
			TWCR = (1<<TWINT)|(1<<TWEN);
		}
	//wait for TWINT flag to se, indicating transmission and ack/nack receive
		while(!(TWCR & (1<<TWINT)));

		//check to see if ack received
		if(((TWSR & 0xF8) != 0x50)&&(data_counter!=3))	{
		printf("data %d ack problem 0x%x \n\r",data_counter,(TWSR & 0xF8));
		TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
		sei();
		return (-1);
		}
		else if(((TWSR & 0xF8) != 0x58)&&(data_counter>=3))	{
		printf("data %d ack problem 0x%x \n\r",data_counter,(TWSR & 0xF8));
		TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
		sei();
		return (-1);
		}
	//	printf("%d ack recieved  0x%x \n\r",data_counter,(TWSR & 0xF8));
	//	printf("data rx is 0x%x \n\r",TWDR);
		if(data_counter==0)
		{
			input.switch_tension_s=TWDR;
		}
		else if(data_counter==1)
		{
			input.switch_dock_s=TWDR;
		}
		else if(data_counter==2)
		{
			temp_var=TWDR;
		
		}else if(data_counter>=3)
		{
			input.bend_s=(TWDR<<8)+temp_var;
		}
		 
			data_counter++;


	
	}

	TWCR = (1<<TWSTO)|(1<<TWINT);

	if(data_counter==4)
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
	uint8_t count = 0;
	unsigned char toggle = 0;	

	power_state=1;
	sleep_mode=1;
	toggle_wakeup=0;
	
		
	printf("\n\r\n\r\n\r\n\r--Flippy v0.2 startup--\n\r");
	if(MASTER==1)
	printf("--in MASTER mode---\n\r\n\r");
	else	
	printf("--in SLAVE mode---\n\r\n\r");

	//twi bit rate set 
  	TWBR=0x04;
	//setup accell


	//dock motor off
	DDRB |= (1<<6);
	PORTB &= ~(1<<6);
	OCR0B = 0;	
	
	//bend motor off
	DDRB |= (1<<7);
	PORTB &= ~(1<<7);
	OCR0A = 0;

	//master/slave startup
	if(MASTER==1)
	{
		
	
		
	}
	else
	{
		TWAR=atmega_slave;
		TWCR= (1<<TWEA)|(1<<TWEN);//|(1<<TWINT);
		

	}

int state=0;
int state0=0;
char toggle=0;
int count=0;
char side=0;
double s_angle=0;
double m_angle=0;
char flipdir=0; //direction of flipping, reference with pcb facing and forward (1) being to the right			

	while(1)
	{	
		if(MASTER==1)
		{
		
		
			///////////////////////
			//user code starts here
			///////////////////////


			//Flipping Program
			static uint8_t;
			_delay_ms(100);
			//printf("master side accel sensor x=%d y=%d z=%d \n\r",input.accell_m[0],input.accell_m[1],input.accell_m[2]);
			//printf("slave side accel sensor x=%d y=%d z=%d \n\r",input.accell_s[0],input.accell_s[1],input.accell_s[2]);
			

//Debug program

			output.led_m[0]=20;
//			printf("m s %d %d %d %d %d %d\n\r",input.accell_m[0],input.accell_m[1], input.accell_m[2], input.accell_s[0],input.accell_s[1], input.accell_s[2]);
//			printf("s x=%d y=%d z=%d \n\r",input.accell_s[0],input.accell_s[1], input.accell_s[2]);
//			s_angle=atan2((input.accell_s[0]),(input.accell_s[1]))*180/3.14159;
//			m_angle=atan2((input.accell_m[0]),(input.accell_m[1]))*180/3.14159;
//			state=(int)abs(s_angle-m_angle);
//			printf("difference=%d \n\r", state);
//			printf("s=%d \n\r", (int)s_angle);
//			printf("m=%d \n\r", (int)m_angle);
//			printf("hi!");
//			printf("m %d %d %d \n\r",input.accell_m[0],input.accell_m[1], input.accell_m[2]);
//			printf("IR %d %d \n\r", input.IR1_m, input.IR2_m);
//			printf("IR %d \n\r", input.IR2_m);
//			output.direction_bend_m3_m=1; // 0 positive
//			output.speed_bend_m3_m=0;
//			output.direction_dock_m5_m=0; // 0 positive
//			output.speed_dock_m5_m=0;
//			output.vibration_m=0;	
/*		
			output.speed_bend_m3_m=0;
			output.speed_bend_m3_s=0;	
			if (input.switch_tension_m==1)
			{
//				output.led_s[0]=30;
//				output.led_s[1]=0;	
//				output.direction_bend_m3_s=0;
//				output.speed_bend_m3_s=120;
				printf("hi!");
				}
			else {
//				output.led_s[0]=0;
//				output.led_s[1]=0;	
//				output.direction_bend_m3_s=1;
//				output.speed_bend_m3_s=0;
				}
*/


//			printf("s bend %d\n\r",input.bend_s);
//			printf("m %d\n\r",input.bend_m);

/*
//Docking

if ((input.switch_dock_m==1)&&(input.switch_dock_s==1)){
	output.led_m[0]=0;
	output.led_m[1]=0;
	output.speed_dock_m5_s=0;
	output.speed_dock_m5_m=0;
	if (state==0){
		state=1;
		}
	else{
		state=0;
		}
	_delay_ms(1000);
	}
if (state==0){
	output.led_s[0]=20;
	output.led_m[0]=20;
	if (input.switch_tension_m==1){
		output.led_m[0]=20;
		output.led_s[1]=20;
		output.direction_dock_m5_m=1;
		output.speed_dock_m5_m=200;
	}
	else if (input.switch_dock_s==1){
		output.led_m[1]=20;
		output.direction_dock_m5_m=0;
		output.speed_dock_m5_m=200;
	}
	else{
		output.led_m[0]=0;
		output.led_m[1]=0;
		output.speed_dock_m5_m=0;
		}
}
else{
	if (input.switch_dock_m==1){
		output.led_m[0]=20;
		output.direction_dock_m5_s=1;
		output.speed_dock_m5_s=200;
	}
	else if (input.switch_tension_m==0){
		output.led_m[1]=20;
		output.direction_dock_m5_s=0;
		output.speed_dock_m5_s=200;
	}
	else{
		output.led_m[0]=0;
		output.led_m[1]=0;
		output.speed_dock_m5_s=0;
		}
}

//printf("%d \n\r", state);
*/
/*
			output.speed_bend_m3_m=0;
			output.speed_bend_m3_s=0;
			output.speed_dock_m5_m=0;
			output.speed_dock_m5_s=0;

		switch(state){
			
		case 0: //unwind
			if((input.switch_dock_m==1)&(input.switch_dock_s==1)){
				state=1;
				count=0;
				toggle=0;
				output.led_m[0]=0;
				output.led_s[0]=0;
				output.led_m[1]=0;
				output.led_s[1]=0;
				}
			else if(input.switch_dock_m==1)
			{
				output.led_m[0]=30;
				output.led_m[1]=0;	
				output.direction_bend_m3_m=1;
				output.speed_bend_m3_m=100;
			}
			else if(input.switch_dock_s==1)
			{
				output.led_s[0]=30;	
				output.led_s[1]=0;
				output.direction_bend_m3_s=1;
				output.speed_bend_m3_s=100;
			}
		case 1: //wind
			if((input.switch_dock_m==1)&(input.switch_dock_s==1)){
				state=0;
				count=0;
				toggle=0;
				output.led_m[0]=0;
				output.led_s[0]=0;
				output.led_m[1]=0;
				output.led_s[1]=0;
				}
			else if(input.switch_dock_m==1)
			{
				output.led_m[0]=30;
				output.led_m[1]=0;	
				output.direction_bend_m3_m=1;
				output.speed_bend_m3_m=100;
			}
			else if(input.switch_dock_s==1)
			{
				output.led_s[0]=30;	
				output.led_s[1]=0;
				output.direction_bend_m3_s=1;
				output.speed_bend_m3_s=100;
			}
		}
*/
//Flipping Programs

	switch(state){
		case 0: //do nothing and unwind motors
			output.led_m[1]=20;
			output.led_s[1]=20;
			output.speed_bend_m3_m=0;
			output.speed_bend_m3_s=0;
			output.speed_dock_m5_m=0;
			output.speed_dock_m5_s=0;
			
			if((input.switch_dock_m==1)&(input.switch_dock_s==1)){
				state=10;
				count=0;
				toggle=0;
				output.led_m[0]=0;
				output.led_s[0]=0;
				output.led_m[1]=0;
				output.led_s[1]=0;
				}
			else if(input.switch_dock_m==1)
			{
				output.led_m[0]=30;
				output.led_m[1]=0;	
				output.direction_bend_m3_m=1;
				output.speed_bend_m3_m=100;
			}
			else if(input.switch_dock_s==1)
			{
				output.led_s[0]=30;	
				output.led_s[1]=0;
				output.direction_bend_m3_s=1;
				output.speed_bend_m3_s=100;
			}
			break;

		case 1:	//flipping
			output.led_m[0]=20;
			output.direction_dock_m5_m=1; //spin the opposite dock motor to prevent attaching
			output.speed_dock_m5_m=gripperspd;
			flipdir=0;
			if (toggle<2){ //Do not attach before the going through neutral
				if(toggle==0){
					output.direction_bend_m3_m=1; //master going back, slave going forward
					output.direction_bend_m3_s=0;
					output.speed_bend_m3_s=windspd;
					output.speed_bend_m3_m=unwindspd; //first unwind at a set speed
					count++;
					}
				else{
					flipbend(flipdir, 10);
					}
				m_angle=get_accel_diff();
				if((m_angle>170)&&(m_angle<200)){
					toggle=2;
//					printf("toggle");
					output.led_m[2]=20;
					}
				if(count>12){
					toggle=1;
					count=0;
					}
				if((toggle==1)&&(m_angle<10)){
					toggle=2;
					}
				}
			if (toggle==2){
				if (input.switch_dock_m==1){
					count++;
					if (count>2){
						output.speed_bend_m3_m=0;
						output.speed_bend_m3_s=0;
						output.speed_dock_m5_m=0;
						output.led_m[0]=0;
						output.led_m[2]=0;
						count=0;
						toggle=0;
						state=3;
						break;
						}
					}
//				else if(get_accel_diff()<0.17){
//					flipbend(-(flipdir-1),10)
//					}
				else{
					flipbend(flipdir,10);
					}
			}
//			else{
//			output.speed_bend_m3_m=0;
//			output.speed_bend_m3_s=0;
//			output.led_m[0]=0;
//			count=0;
//			toggle=0;
//			printf("%d\n\r",input.bend_m);
//			state=3;
//			}
			break;

		case 2: //flipping
			output.direction_dock_m5_s=1; //spin the opposite dock motor to prevent attaching
			output.speed_dock_m5_s=gripperspd;
			output.led_s[0]=20;
			flipdir=1;
//			printf("m bend %d\n\r",input.bend_m);
			if (toggle<2){ //Do not attach before the angle of slave board has changed by pi/2
				if(toggle==0){
					output.direction_bend_m3_m=1; //master going back, slave going forward
					output.direction_bend_m3_s=0;
					output.speed_bend_m3_s=windspd;
					output.speed_bend_m3_m=unwindspd;
					count++;
					}
				else{
					flipbend(flipdir, 10);
					}
				m_angle=get_accel_diff();
				if((m_angle>170)&&(m_angle<200)){
					toggle=2;
					output.led_s[2]=20;
					}
				if(count>12){
					toggle=1;
					count=0;
					}
				if((toggle==1)&&(m_angle<10)){
					toggle=2;
					}
				}
			if (toggle==2){
				if (input.switch_dock_s==1){
					count++;
					if (count>2){
						output.speed_bend_m3_m=0;
						output.speed_bend_m3_s=0;
						output.speed_dock_m5_m=0;
						output.led_s[0]=0;
						output.led_s[2]=0;
						count=0;
						toggle=0;
						state=4;
						break;
						}
					}
				else{
					flipbend(flipdir,10);
					}
				}

//			else if (input.bend_s<maxbend_s){
//			printf("%d \n\r", input.bend_s);
//			else{
//			output.speed_bend_m3_s=0;
//			output.speed_bend_m3_m=0;
//			output.led_s[0]=0;
//			toggle=0;
//			printf("%d\n\r",input.bend_s);
//			state=4;
//			}
			break;

		case 3: //attaching master side
			output.led_m[1]=20;
			output.speed_bend_m3_s=0;
			output.speed_bend_m3_m=0;
////////////manual attachment
//			if (input.switch_dock_m==1){
//				output.direction_dock_m5_m=1;
//				output.speed_dock_m5_m=200;
//				toggle=1;
//			printf("%c\n\r",toggle);
//			}
//			else if(toggle==1){
//				output.speed_dock_m5_m=0;
//				output.led_m[1]=0;
//				state=7;
//				}
///////////////////

//// automatic attachment
			output.direction_dock_m5_m=0; 
			output.speed_dock_m5_m=0.7*gripperspd;
			if (count>20){
				count=0;
				output.led_m[1]=0;	
				output.speed_dock_m5_m=0;
					//check current orientation of boards
				m_angle=atan2((input.accell_s[0]),(input.accell_s[1]))*180/3.14159;
				printf("%d \n\r", (int)m_angle); 
//				m_angle=get_accel_diff();
				state=7;
			}
			count++;
////
			break;
		case 4: //attaching slave side
			output.led_s[1]=20;
			output.speed_bend_m3_s=0;
			output.speed_bend_m3_m=0;

////////////manual attachment
//			if (input.switch_dock_s==1){
//				output.direction_dock_m5_s=1;
//				output.speed_dock_m5_s=200;
//				toggle=1;
//			}
//			else if(toggle==1){
//				output.speed_dock_m5_s=0;
//				output.led_s[1]=0;
//				state=5;
//				count=0;
//				}
////////////

/////automatic attachment
			output.direction_dock_m5_s=0;
			output.speed_dock_m5_s=0.7*gripperspd;
			if (count>20){
				count=0;
				output.led_s[1]=0;
				output.speed_dock_m5_s=0;
					//check current orientation of boards
//				s_angle=atan2((input.accell_s[0]),(input.accell_s[1])); 
				m_angle=atan2((input.accell_m[0]),(input.accell_m[1]))*180/3.14159;
				printf("%d \n\r", (int)m_angle);
//				m_angle=get_accel_diff();
				state=5;	
			}
			count++;
//////
			break;
		case 5: //detaching the master side
				output.speed_bend_m3_m=0;
				output.speed_bend_m3_s=0;
				output.led_m[2]=20;
				output.direction_dock_m5_m=1;
				output.speed_dock_m5_m=gripperspd;
				side=0;
				count++;
				if (toggle==0){
					s_angle=atan2((input.accell_m[0]),(input.accell_m[1]))*180/3.14159;
//					printf("%d \n\r", (int)s_angle);
					s_angle=abs(s_angle-m_angle);
					if (s_angle>300){
					s_angle=(360-s_angle);
					}
//					printf("%d %d \n\r", (int)m_angle, (int)get_accel_diff());
//					if (abs(get_accel_diff()-m_angle)>20){
					if (s_angle>30){
						toggle=1;
						}
					}
			if ((input.switch_dock_m==0)&&(toggle==1)){
					//check angle
					state=-(flipdir-2);
					output.speed_bend_m3_m=0;
					output.speed_bend_m3_s=0;
					output.led_m[2]=0;
					count=0;
					toggle=0;
					_delay_ms(1000);
					break;
				}
			if (count>20){
				state=6;
				count=0;
				}
			break;
		case 6: //detaching the master side - flipping disturbance
				output.led_m[2]=20;
//				output.direction_dock_m5_m=0;
//				output.speed_dock_m5_m=100;
			if (count<3){
				flipbend(-(flipdir-1), 10);
				}
//			else if (count<6){
//				flipbend(flipdir);
//				}
//				printf("flipbend");
				count++;
			if (toggle==0){
					s_angle=atan2((input.accell_m[0]),(input.accell_m[1]))*180/3.14159;
//					printf("%d \n\r", (int)s_angle);
					s_angle=abs(s_angle-m_angle);
					if (s_angle>300){
						s_angle=(360-s_angle);
						}
//						printf("%d \n\r", (int)s_angle);
//			printf("%d %d \n\r", (int)m_angle, (int)get_accel_diff());
//					if (abs(get_accel_diff()-m_angle)>20){
					if (s_angle>30){
						toggle=1;
						}
					}
			if ((input.switch_dock_m==0)&&(toggle==1)){
					//check angle
					state=-(flipdir-2);
					count=0;
					output.led_m[2]=0;
					output.speed_bend_m3_m=0;
					output.speed_bend_m3_s=0;
					break;
				}
			if (count>3){
				state=5;
				count=0;
				}
			break;
		case 7: //detaching the slave side
				output.speed_bend_m3_m=0;
				output.speed_bend_m3_s=0;
				output.led_s[2]=20;
				output.direction_dock_m5_s=1;
				output.speed_dock_m5_s=gripperspd;
				side=0;
				count++;
				if (toggle==0){
					s_angle=atan2((input.accell_s[0]),(input.accell_s[1]))*180/3.14159;
//					printf("%d \n\r", (int)s_angle);
					s_angle=abs(s_angle-m_angle);
					if (s_angle>300){
						s_angle=(360-s_angle);
						}
//						printf("%d \n\r", (int)s_angle);
//				printf("%d %d \n\r", (int)m_angle, (int)get_accel_diff());
//					if (abs(get_accel_diff()-m_angle)>20){
					if (s_angle>30){
						toggle=1;
						}
					}
			if ((input.switch_dock_s==0)&&(toggle==1)){
					//check angle
					count=0;
					state=-(flipdir-2);
					output.led_s[2]=0;
					_delay_ms(1000);
					break;
				}
			if (count>20){
				state=8;
				count=0;
				}
			break;
		case 8: //detaching the slave side - flipping disturbance
				output.led_s[2]=20;
//				output.direction_dock_m5_m=0;
//				output.speed_dock_m5_m=100;
			if (count<3){
				flipbend(-(flipdir-1), 10);
				}
			if (toggle==0){
					s_angle=atan2((input.accell_s[0]),(input.accell_s[1]))*180/3.14159;
//					printf("%d \n\r", (int)s_angle);
					s_angle=abs(s_angle-m_angle);
					if (s_angle>300){
						s_angle=(360-s_angle);
						}
//					printf("%d \n\r", (int)s_angle);
//			printf("%d %d \n\r", (int)m_angle, (int)get_accel_diff());
//					if (abs(get_accel_diff()-m_angle)>20){
					if (s_angle>30){
						toggle=1;
						}
					}
//			else if (count<6){
//				flipbend(flipdir);
//				}
//				printf("flipbend");
				count++;
			if ((input.switch_dock_s==0)&&(toggle==1)){
					//check angle
					state=-(flipdir-2);
					count=0;
					output.led_s[2]=0;
					output.speed_bend_m3_m=0;
					output.speed_bend_m3_s=0;
					break;
				}
			if (count>3){
				state=7;
				count=0;
				}
			break;
		case 9 : //unwind motors (reset the motors in error case)
				if ((input.switch_tension_m==0)||(input.switch_tension_s==0)){
					output.direction_bend_m3_m=1;
					output.direction_bend_m3_s=1; //set both motor directiosn to backwards
					output.speed_bend_m3_m=unwindspd; 
					output.speed_bend_m3_s=unwindspd;
				}
				else {
					output.speed_bend_m3_m=0; 
					output.speed_bend_m3_s=0;
					state=state0; //reset back to original state
					}
			break;
		case 10: //rewind motors
			output.direction_bend_m3_m=0;
			output.direction_bend_m3_s=0;
			if ((input.switch_dock_m==0)&(input.switch_dock_s==0)){
				toggle=1;
				}
			if (toggle==1){
				if(input.switch_tension_s==1){
					output.speed_bend_m3_m=60;
					}
				else {
					output.speed_bend_m3_m=0;
					}
				if(input.switch_tension_m==1) {
					output.speed_bend_m3_s=60;
					}
				else {
					output.speed_bend_m3_s=0;
					}	
				}
			if ((input.switch_tension_s==0)&(input.switch_tension_m==0)&(toggle=1)){
				if(input.switch_dock_m==1){
				state=11;
				toggle=0;
				_delay_ms(2000);
				break;
				}
				}
				break;
		case 11: //attach or detach slave side
				output.led_m[0]=20;
			if ((input.switch_dock_m==1)&&(input.switch_dock_s==1)){
				output.speed_dock_m5_s=0;
				output.speed_dock_m5_m=0;
				output.led_s[0]=0;
				output.led_s[1]=0;
				output.led_m[0]=0;
				state=12;
				_delay_ms(2000);
				break;
				}
			else if (input.switch_dock_m==1){
				output.led_s[0]=20;
				output.led_s[1]=0;
				output.direction_dock_m5_s=0;
				output.speed_dock_m5_s=0.7*gripperspd;
				}
			else if (input.switch_dock_s==1){
				output.led_s[0]=0;
				output.led_s[1]=20;
				output.direction_dock_m5_s=1;
				output.speed_dock_m5_s=gripperspd;
				}
			else{
				output.led_s[0]=0;
				output.led_s[1]=0;
				output.speed_dock_m5_s=0;
				}
				break;
	case 12: //attach or detach master side
			output.led_s[0]=20;
		if ((input.switch_dock_m==1)&&(input.switch_dock_s==1)){
			output.speed_dock_m5_s=0;
			output.speed_dock_m5_m=0;
			output.led_m[0]=0;
			output.led_m[1]=0;
			output.led_s[0]=0;
			state=1;
//			s_angle=atan2((input.accell_s[0]),(input.accell_s[1]));
//			m_angle=atan2((input.accell_m[0]),(input.accell_m[1]));
			_delay_ms(9000);
			break;
			}
		else if (input.switch_dock_m==1){
			output.led_m[0]=20;
			output.led_m[1]=0;
			output.direction_dock_m5_m=0;
			output.speed_dock_m5_m=0.7*gripperspd;
		}
		else if (input.switch_dock_s==1){
			output.led_m[0]=0;
			output.led_m[1]=20;
			output.direction_dock_m5_m=1;
			output.speed_dock_m5_m=gripperspd;
		}
		else{
			output.led_m[0]=0;
			output.led_m[1]=0;
			output.speed_dock_m5_m=0;
			}
			break;

	}


			///////////////////////
			//user code ends here
			///////////////////////
			/*

			//IMPORTANT NOTE: these outputs and inputs will not be changed or come into effect until the update 
			//function is executed 

			//example setting all outputs under our control
			//slave side
			output.speed_bend_m3_s=0; //speed of bend motor for slave 0-255
			output.speed_dock_m5_s=1; //speed of dock motor for slave 0-255
			output.direction_dock_m5_s=2; //direction of dock motor for slave 0,1
			output.direction_bend_m3_s=3; //direction of bend motor for slave 0,1
			output.led_s[0]=4; //slave led (RED)0-255
			output.led_s[1]=5;//slave led (GREEN)0-255
			output.led_s[2]=6;//slave led(BLUE)0-255
			//master side		
			output.speed_bend_m3_m=0; //master of bend motor for slave 0-255
			output.speed_dock_m5_m=1; //master of dock motor for slave 0-255
			output.direction_dock_m5_m=2; //direction of dock motor for master 0,1
			output.direction_bend_m3_m=3; //direction of bend motor for master 0,1
			output.led_m[0]=4; //master led (RED)0-255
			output.led_m[1]=5;//master led (GREEN)0-255
			output.led_m[2]=6;//master led(BLUE)0-255
			*/
			/*
			//Notes on inputs:
			input.switch_tension_m==0; switch is pressed
			input.switch_tension_s==0; switch is pressed
			input.switch_dock_m==1; switch is pressed
			input.switch_dock_s==1; switch is pressed
			*/

			/*
			//print out all input values for an example
			printf("slave side tension switch %d\n\r",input.switch_tension_s);
			printf("slave side dock switch %d\n\r",input.switch_dock_s);
			printf("slave side bend sensor %d\n\r",input.bend_s);
			printf("slave side accel sensor x=%d y=%d z=%d \n\r",input.accell_s[0],input.accell_s[1],input.accell_s[2]);
			printf("master side tension switch %d\n\r",input.switch_tension_m);
			printf("master side dock switch %d\n\r",input.switch_dock_m);
			printf("master side bend sensor %d\n\r",input.bend_m);
			printf("master side accel sensor x=%d y=%d z=%d \n\r",input.accell_m[0],input.accell_m[1],input.accell_m[2]);
			*/

		//	printf("%4d %4d \n\r",input.bend_s,input.bend_m);
			//below are the 4 update functions
			//send output commands to slave side						
			i2c_send();
			//update output commands on master side
			master_output_update();
			//update slave side input values
			i2c_read();
			//update master side input values
			master_input_update();
		
			//you can adjust this delay.. eventually if too small it may cause problems, but you can fix this by changing it back
			_delay_ms(20);




		
			
		}
		else//slave code 
		{
	
			////////////
			//slave code, do not modify
			////////////
			static uint8_t dock_speed=0;
			static uint8_t flex_speed=0;

			static uint8_t tx_data_counter=0; //counter for slave inputs
			static uint8_t rx_data_count=0; //counter for slave outputs
			if((TWCR & (1<<TWINT))) 
			{
				//printf("status 0x%x , count %d\n\r",(TWSR & 0xF8),rx_data_count);
				//slave receiver stuff
				if((TWSR & 0xF8)==0x60)
				{
					TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT);
					rx_data_count=0;
					//printf("slave rx address start\n\r");
				}
				else if((TWSR & 0xF8)==0x80)
				{
					if(rx_data_count==0)
					{
						output.speed_bend_m3_s=TWDR; //rename
					}
					else if(rx_data_count==1)
					{
						//dock speed
						output.speed_dock_m5_s=TWDR;
					}
					else if(rx_data_count==2)
					{
						//dock direction
						output.direction_dock_m5_s=TWDR;
						set_M5(output.direction_dock_m5_s, output.speed_dock_m5_s);
						/*	
						if(TWDR==0)
						{
						DDRB |= (1<<6); //old HB2-2 is PC2, now PB6
						PORTB &= ~(1<<6);
						OCR0B = dock_speed;

						}
						else
						{
							DDRB |= (1<<6); //old HB2-2 is PC2, now PB6
							PORTB |= (1<<6);
							OCR0B = 255-dock_speed; //HB2-1
						}
						*/

					}
					else if(rx_data_count==3)
					{

					//M3 direction
					output.direction_bend_m3_s=TWDR;
					set_M3(output.direction_bend_m3_s, output.speed_bend_m3_s);
						/*
						if(TWDR==0)
						{
						DDRB |= (1<<7); //old HB1-2 is PC3, now PB7
						PORTB &= ~(1<<7);
						OCR0A = flex_speed;

						}
						else
						{
							DDRB |= (1<<7); //old HB1-2 is PC3, now PB7
							PORTB |= (1<<7);
							OCR0A = 255-flex_speed;
						} */
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
					tx_data_counter=1;
					//TWDR=switch_tension1();
					TWDR=get_switch_input(STension_port, STension_pin);
					
					TWCR=(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
				
				}
				else if(((TWSR & 0xF8)==0xb8))
				{
				
								
					if(tx_data_counter==1)
					//TWDR=switch_dock();
					TWDR=get_switch_input(S4_port, S4_pin);
					else if(tx_data_counter==2)
					TWDR=get_bend()&0xff;
					else if(tx_data_counter==3)
					TWDR=(get_bend()>>8)&0xff;

					TWCR=(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
					tx_data_counter++;	
				
				}
				else if(((TWSR & 0xF8)==0xc0))
				{
						
					TWDR=1;
					TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA);
					tx_data_counter++;
					
				
				}
				else
				{
					printf("bus error? i2c data 0x%x\n\r",(TWSR & 0xF8));
					TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
				}	

			}
			if((power_state==0))
			{
				
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
			else
			{
				if(toggle_wakeup==1)
				{
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
		//	printf("wakeup\n\r");
			}
				

		}

	}

}

int switch_tension1(void)
{

	if((PINB & (1<<1))!=0)//tension switch connected to PB1, high when connected, so when not low, switch is not connected. 
	{
		return(0);
	}
	else
	{
		return(1);
	}
}

int switch_power(void)
{

	if((PIND & (1<<3))!=0)//power switch connected to PD3, low when connected, so when not low, switch is not connected. 
	{
		return(0);
	}
	else
	{
		return(1);
	}
}

int switch_dock(void) //consider renaming (control switch)
{
	
	if((PIND & (1<<4))!=0)//switch connected to PD4, low when connected, so when not low, switch is not connected.
	{
		return(0);
	}
	else
	{
		return(1);
	}


}

void flipbend(char side, char p){
	//p a percentage to vary speed. 
	if (side==1){ //master side
			output.direction_bend_m3_m=0; //master going forward, slave going back
			output.direction_bend_m3_s=1;
		if(input.switch_tension_m==0){
			output.speed_bend_m3_s=unwindspd*p/10;
		}
		else {
			output.speed_bend_m3_s=0;
		}
	output.speed_bend_m3_m=windspd*p/10;	
	}
	else{ //slave side
		output.direction_bend_m3_m=1; //slave going forward, master going back
		output.direction_bend_m3_s=0;
		if(input.switch_tension_s==0){
			output.speed_bend_m3_m=unwindspd*p/10;
		}
		else {
			output.speed_bend_m3_m=0;
		}
	output.speed_bend_m3_s=windspd*p/10;
	}
}

double get_accel_diff(void){
		double diff=abs(atan2((input.accell_s[0]),(input.accell_s[1]))*180/3.14159-atan2((input.accell_m[0]),(input.accell_m[1]))*180/3.14159);
		return(diff);
}

int get_IR1(void)
{	
	     //initalize adc
		ADMUX &= (1<<REFS0); //choose reference and clear analog pins, AVcc chosen as AREF
		ADMUX |=(1<<MUX0);//choose analog pin ADC1 connected to Flex/IR2
		ADCSRA = (1<<ADEN) |    (1<<ADPS0); //ADEN set makes ADC enabled, ADPS0..2 selects ADC clock vs system clock, here divided by 2




		//when the following code was in main, while loop started here
		ADCSRA |= (1<<ADSC);//start adc conversion to sample sensor with led off
		while((ADCSRA&(1<<ADSC))!=0);//busy wait for conversion to end

//		printf("%d \n\r",ADCW);	

	return(ADCW);

}

int get_IR2(void)
{	
	     //initalize adc
		ADMUX &= (1<<REFS0); //choose reference and clear analog pins
		ADMUX |=0x02;//choose analog pin ADC2 connected to IR1
		ADCSRA = (1<<ADEN) |    (1<<ADPS0); //set up a/d


		//when the following code was in main, while loop started here
		ADCSRA |= (1<<ADSC);//start adc conversion to sample sensor with led off
		while((ADCSRA&(1<<ADSC))!=0);//busy wait for conversion to end

//		printf("%d \n\r",ADCW);	

	return(ADCW);

}

int get_bend(void)
{	
	     //initalize adc
		ADMUX &= (1<<REFS0); //choose reference and clear analog pins
		ADMUX |= 0x01;//voltage reference selection AVcc at AREF and choose analog pin - ADC1
		ADCSRA = (1<<ADEN) |    (1<<ADPS0); //set up a/d (aden adc enable, )




		//when the following code was in main, while loop started here
		ADCSRA |= (1<<ADSC);//start adc conversion to sample sensor with led off
		while((ADCSRA&(1<<ADSC))!=0);//busy wait for converstion to end

//			printf("%d \n\r",ADCW);	

		return(ADCW);

}

void master_output_update() //motor updates
{

	if(output.direction_dock_m5_m==0)
	{
//		DDRC |= (1<<2); //old HB2-2 is PC2, now PB6
//		PORTC &= ~(1<<2);
		DDRB |= (1<<6);
		PORTB &= ~(1<<6);
		OCR0B = output.speed_dock_m5_m;

	}
	else
	{
//		DDRC |= (1<<2);
//		PORTC |= (1<<2);
		DDRB |= (1<<6);
		PORTB |= (1<<6);
		OCR0B = 255-output.speed_dock_m5_m;
	}


	if(output.direction_bend_m3_m==0)
	{
//		DDRC |= (1<<3); //old HB1-2 is PC3, now PB7
//		PORTC &= ~(1<<3);
		DDRB |= (1<<7);
		PORTB &= ~(1<<7);
		OCR0A = output.speed_bend_m3_m;

	}
	else
	{
//		DDRC |= (1<<3);
//		PORTC |= (1<<3);
		DDRB |= (1<<7);
		PORTB |= (1<<7);
		OCR0A = 255-output.speed_bend_m3_m;
	}

	if (output.vibration_m==1)
	{	
		DDRB |=(1<<0); //Vibration motor PB0
		PORTB |=(1<<0);
	}
	else 
	{
		DDRB |= (1<<0); //Vibration motor PB0
		PORTB &= ~(1<<0);
	}

	setLED(output.led_m[0],output.led_m[1],output.led_m[2]);

}
void master_input_update()  
{
	input.switch_dock_m=switch_dock();
	input.switch_tension_m=switch_tension1();
	input.bend_m=get_bend();
	input.IR1_m=get_IR1();
//	printf("%d \n\r",input.IR1_m);
	input.IR2_m=get_IR2();
//	printf("%d \n\r",input.IR2_m);

		//get accel data from master side
	
	//i2c_write_accell( 0b11010010,0x1c,0b11100000);

	i2c_write_accell( 0b11010010,0x6b,0);
	int x=((i2c_read_accell( 0b11010010, 0x3b)<<8)&0xff00)+(i2c_read_accell( 0b11010010, 0x3c)&0x00ff);
	int y=((i2c_read_accell( 0b11010010, 0x3d)<<8)&0xff00)+(i2c_read_accell( 0b11010010, 0x3e)&0x00ff);			
	int z=((i2c_read_accell( 0b11010010, 0x3f)<<8)&0xff00)+(i2c_read_accell( 0b11010010, 0x40)&0x00ff);
		//convert from 2's complement
    if(x>0x8000)
    {
        x=x ^ 0xffff;
        x=-x-1;
    }
	if(y>0x8000)
    {
        y=y ^ 0xffff;
        y=-y-1;
    }
	if(z>0x8000)
    {
        z=z ^ 0xffff;
        z=-z-1;
    }

	input.accell_m[0]=x;
	input.accell_m[1]=y;
	input.accell_m[2]=z;

		//i2c_write_accell( 0b11010010,0x1c,0b11100000);
	i2c_write_accell( 0b11010000,0x6b,0);
	x=((i2c_read_accell( 0b11010000, 0x3b)<<8)&0xff00)+(i2c_read_accell( 0b11010000, 0x3c)&0x00ff);
	y=((i2c_read_accell( 0b11010000, 0x3d)<<8)&0xff00)+(i2c_read_accell( 0b11010000, 0x3e)&0x00ff);			
	z=((i2c_read_accell( 0b11010000, 0x3f)<<8)&0xff00)+(i2c_read_accell( 0b11010000, 0x40)&0x00ff);
		//convert from 2's complement
    if(x>0x8000)
    {
        x=x ^ 0xffff;
        x=-x-1;
    }
	if(y>0x8000)
    {
        y=y ^ 0xffff;
        y=-y-1;
    }
	if(z>0x8000)
    {
        z=z ^ 0xffff;
        z=-z-1;
    }

	input.accell_s[0]=x;
	input.accell_s[1]=y;
	input.accell_s[2]=z;

//	printf("bend is %d\n\r",input.bend_m);
}
/* 
//Integrated this into the master outputs instead
void VibrationMotorOn(){
	DDRB |=(1<<0); //Vibration motor PB0
	PORTB |=(1<<0);
}

void VibrationMotorOff(){
	DDRB |=(1<<0); //Vibration motor PB0
	PORTB |=(1<<0);
}
*/
void setLED(unsigned char red, unsigned char green, unsigned char blue)
{
	//Bit banging 20-600kHz
	//Send LSB first
	
	unsigned char array[4] = {0x3A, red, blue,green};

	for(char byte = 0; byte <= 3; byte++)
	{
		for(unsigned char bit=0; bit<=7; bit++)
		{
			//bit initiation
			PORTD |= 0x80; //old RGB driver at PD0 (0x01), new RGB driver will be at PD7 (0x80)
			_delay_us(2);
			PORTD &=~ 0x80;
			_delay_us(3);

			if(array[byte] & (0x80>>bit))
				PORTD |= 0x80;
			else
				PORTD &=~0x80;
			_delay_us(10); 
			PORTD &=~0x80;
			_delay_us(5);  //~50kHz	
		}
	}

	_delay_us(80);//End of Sequence
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
