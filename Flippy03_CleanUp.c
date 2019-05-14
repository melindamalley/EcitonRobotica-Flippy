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

#define MASTER 1 // 1 for Master, 0 for Slave 

#define FOSC 8000000 // oscillator clock frequency, page 164 datasheet, internal RC oscillator clock source selected by fuse bits
#define BAUD 9600 // baud rate desired
#define MYUBRR ((FOSC/(16L * BAUD)) - 1) // used to set the UBRR high and low registers, Usart Baud Rate registers, for formula see datasheet page 146

// IMU information
#define accell_slave_addrs  0b11010000 // IMU address on slave board,  [7bit i2c embedded address from IMU chip datasheet,0] = 0xd0
#define	accell_master_addrs 0b11010010 // IMU address on master board, [7bit i2c embedded address from IMU chip datasheet,0] = 0xd2
#define IMU_ADDRESS (MASTER ? accell_master_addrs : accell_slave_addrs)  // (Condition? true_value: false_value), for unifying master/slave code

// IMU chip registers
// accelerometer registers 0x3B to 0x40 for MPU-9250 (Mar 2019 order), 0x2D to 0x32 for ICM-20948 (Dec 2018 order)
#define ACCEL_XOUT_H	0x3B 	// 0x2D //for ICM-20948
#define ACCEL_XOUT_L	0x3C 	// 0x2E //for ICM-20948

#define ACCEL_YOUT_H	0x3D	// 0x2F //for ICM-20948
#define ACCEL_YOUT_L	0x3E    // 0x30 //for ICM-20948

#define ACCEL_ZOUT_H	0x3F	// 0x31 //for ICM-20948
#define ACCEL_ZOUT_L	0x40	// 0x32 //for ICM-20948

#define PWR_MGMT_1		0x6B   	// should be set to 0 for Accel to be run, see datasheet page 31

// system states
#define TEST	0x00
#define EXPERIMENT	0x01
#define SETUP	0x0F
#define FLIP	0x1F

// 
#define atmega_slave 0xf0 // // address of the slave board processor, to be renamed to something more meaningful e.g. SLAVE_MCU_ADDRESS
#define led_wrt_cmd 0x3A // led driver write command

//////////////////////////////////////
//
// MACROS FOR PIN HANDLING (STOLEN FROM JULIA)
#define output(directions,pin) (directions|= pin) //set port direction for output
#define input(directions,pin) (directions &=(~pin)) //set port direction for input
#define set(port,pin) (port |= pin) // set port pin
#define clear(port,pin) (port &= (~pin)) // clear port pin
//////////////////////////////////////

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

//

///////////////

//Flip Parameters:

#define windspd 130
#define unwindspd 220
#define gripperspd 180

//Define helper functions
void setLED(unsigned char red, unsigned char green, unsigned char blue);
int switch_power(void);
int switch_tension1(void);
int switch_dock(void);
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
uint8_t system_state = TEST;

// Robot Inputs/Sensors
struct inputs{
	uint8_t	switch_power;
 	uint8_t switch_tension_m;
	uint8_t switch_tension_s;
	uint8_t switch_dock_m;
	uint8_t switch_dock_s;
	int bend_s;
	int bend_m;
	int IR1_m; 
	int IR2_m;
	int accell_m[3];
	int accell_s[3]; 
 
};

// Robot Outputs
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
static uint8_t power_state; //
static uint8_t toggle_wakeup; //variable for power/sleep function
static uint8_t sleep_mode; //

// setup for printf
int uart_putchar(char c, FILE *stream) {
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
void ioinit (void) { 
	// Set baud rate
    UBRR0H = (unsigned char) (MYUBRR >> 8);
    UBRR0L = (unsigned char) MYUBRR;  
    // Enable USART transmitter to print on PC screen, (1<<TXEN0)
    // Enabel USART receiver to receive commands from PC, (1<<RXEN0), not included in original code
    UCSR0B = (1<<TXEN0);

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

void init(void)
{
	// Set all pins on Hi-Z, all ports as input & pull-up resistors deactivated
	DDRB=0;
	PORTB=0;
	DDRC=0;
	PORTC=0;
	DDRD=0;
	PORTD=0;

	// USART init
	ioinit(); 

	sleep_mode=0;

	//power on 
	output(vreg1_port_direction, vreg1_pin);
	set(vreg1_port, vreg1_pin); //turn on PC0 (vreg1)
	//DDRC |= (1<<0); //output PC0
	//PORTC |= (1<<0);  //turn on PC0 (Vreg1)

	// RGB led init
	output(led_port_direction, led_pin); 	

	DDRB &= ~(1<<1); //tension switch as input PB1
	DDRD &= ~(1<<4); //gripper/control switch as input PD4
	DDRD &= ~(1<<3); //power switch as input PD3

	DDRB |= (1<<6); //Hbridge 2-1 output PB6
	DDRB |= (1<<7); //Hbridge 1-1 output PB7
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM00); //Timer counter init (for pwm/motor control)
	TCCR0B =0x03; //prescaler set to 0
	OCR0B = 0x00; //start with motor off (PD5)
	OCR0A = 0x00; //start with motor off (PD6)
	DDRB |= (1<<0); //vibration motor output PB0

	DDRC &= ~(1<<1); //IR2 or flex sensor as input (PC1)
	DDRC &= ~(1<<2); //IR1 as input (PC2)
	DDRC &= ~(1<<3); //strain gage as input (PC3)
	//initalize adc
	ADMUX = (1<<REFS0); //|(1<<MUX0);//choose analog pin  
	ADCSRA = (1<<ADEN) | (1<<ADPS0); //set up a/d

	// ??BH?? INT0 ISR does not exist, i.e. the interrupt ISR executes no code **MM we haven't added in the wakeup protocol - check old AutoAccel file?
	//enable interrupt INT0/PCINT18/PD2, used by U1/IMU 
	DDRD &= ~(1<<2);		// DDRxn reset, PD2 configured as input
	PORTD |= 1<<2;		// PORTxn set when pin configured as input, PD2 pull-up resistor activated
	EIMSK = 1<<1;					// External Interrupt Mask Register (EIMSK) bit 1 set, INT0 enabled
	EICRA = 1<<ISC01 | 1<<ISC00;	// External Interrupt Control Register A (EICRA), ISCxx Interrupt Sense Control bits, INT0 set to trigger on rising edge 

  	TWBR=0x04; //twi bit rate set 
	
    //dock motor off
	DDRB |= (1<<6);
	PORTB &= ~(1<<6);
	DDRD |= (1<<6);
	PORTD &= ~(1<<6);
	OCR0B = 0;	
	
	//bend motor off
	DDRB |= (1<<7);
	PORTB &= ~(1<<7);
	DDRD |= (1<<5);
	PORTD &= ~(1<<5);
	OCR0A = 0;

	power_state=1;
	sleep_mode=1;
	toggle_wakeup=0;

}

// AVR TWI is byte-oriented and interrupt based
// ISR for the 2 wire Serial interface
ISR(TWI_vect) 
{
	cli();
	printf("TWI ISR reached ...\n\r");
	
	TWCR &= ~(1<<TWIE);	// Two Wire Control Register (TWCR), 
	// TWEN bit = 1 to enable the 2-wire serial interface
	// TWINT bit = 1 to clear the TWINT flag
	TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
}	

// ??BH?? function to be renamed in future revisions, i.e. i2c_imu_write 
uint8_t i2c_write_accell(uint8_t chip_address,uint8_t reg_address,uint8_t data) //??? Double check function not missing anything?
{
	// start TWI transmission
	// Two Wire Control Register (TWCR)
	// TWEN bit = 1 to enable the 2-wire serial interface
	// TWSTA bit = 1 to transmit a START condition
	// TWINT bit = 1 to clear the TWINT flag
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); 

	// wait for TWINT bit in TWCR reg set by the TWI control unit
	// this indicates START condition transmission is finished
	// user application code then checks TWSR reg if START condition transmission successful
	while(!(TWCR &(1<<TWINT)));

	// Checks TWSR reg if START condition transmission successful, see datasheet page 183
	if((TWSR & 0xF8) != 0x08)
	{
		printf("TWI START condition transmission error\n\r");
		// ToDo: add error handling code, not in original code
	}

	// Load write address of slave into TWDR
	TWDR = chip_address;

	// clear TWINT bit in TWCR to start transmission of address
	TWCR = (1<<TWINT)|(1<<TWEN);

	// wait for TWINT flag to raise, indicating transmission and ack/nack receive
	while(!(TWCR & (1<<TWINT)));

	// Check TWSR reg to if ack received
	if((TWSR & 0xF8) != 0x18)	
	printf("first ack problem 0x%x \n\r",(TWSR & 0xF8));
	// printf("ack recieved OK 0x%x \n\r",(TWSR & 0xF8));


	// Send data, in this case an address
	TWDR = reg_address;//0x6b;// accell x value msb's
	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	// check for data ack	
	if((TWSR & 0xF8) != 0x28)	
	printf("second ack problem is 0x%x\n\r",(TWSR & 0xF8));
	// printf("second ack received OK is 0x%x\n\r",(TWSR & 0xF8));

	//send data
	TWDR = data; //0x00;// accell x value msb's
	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	// check for data ack	
	if((TWSR & 0xF8) != 0x28)	
	printf("third ack problem is 0x%x\n\r",(TWSR & 0xF8));
	// printf("third ack received OK is 0x%x\n\r",(TWSR & 0xF8));

	// transmit STOP condition 
	TWCR = (1<<TWINT)|(1<<TWSTO);

	return(0);

}


// ??BH?? function to be renamed in future revisions, i.e. i2c_imu_read 
uint8_t i2c_read_accell(uint8_t chip_address, uint8_t reg_address) //??? Double check function not missing anything?
{
	
	/* // ??BH?? I think this can be omitted */
	// start TWI transmission
	// Two Wire Control Register (TWCR)
	// TWEN bit = 1 to enable the 2-wire serial interface
	// TWSTA bit = 1 to transmit a START condition
	// TWINT bit = 1 to clear the TWINT flag
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); 

	// wait for TWINT bit in TWCR reg set by the TWI control unit
	// this indicates START condition transmission is finished
	// user application code then checks TWSR reg if START condition transmission successful
	while(!(TWCR &(1<<TWINT)));

	// user application code checks TWSR reg if START condition transmission successful, see datasheet page 183
	if((TWSR & 0xF8) != 0x08)
	{
		printf("TWI START condition transmission error\n\r");
		// ToDo: add error handling code
	}

	// load SLA+R/W into TWDR, 7bit slave addres + 1bit (0 for write/1 for read by the master chip)  
	// uint8_t	SLA_W=0b11010000;
	uint8_t	SLA_W = chip_address;
	TWDR = SLA_W;

	//start transmission
	TWCR=(1<<TWINT)|(1<<TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while(!(TWCR & (1<<TWINT)));

	//check to see if ack received
	if((TWSR & 0xF8) != 0x18)	
	printf("first ack problem 0x%x \n\r",(TWSR & 0xF8));
    //	printf("ack recieved OK 0x%x \n\r",(TWSR & 0xF8));

	//send data, in this case an address
	TWDR = reg_address;// accell x value msb's
	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	
	printf("second ack problem is 0x%x\n\r",(TWSR & 0xF8));
	//	printf("second ack received OK is 0x%x\n\r",(TWSR & 0xF8));

	//send stop bit
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	//
	// ??BH?? I think this can bo omitted */


	////master receiver mode
	//start twi transmission
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	//wait for TWCR flag to set indication start is transmitted
	while(!(TWCR &(1<<TWINT)));


	//check to see if start is an error or not
	if((TWSR & 0xF8) != 0x10)
	printf("start condition error 0x%x\n\r",(TWSR & 0xF8));

	//Load read addres of slave in to TWDR, 
	SLA_W=chip_address | 1;//	 SLA_W=0b11010001;
	TWDR=SLA_W;

	//start transmission
	TWCR=(1<<TWINT)|(1<<TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while(!(TWCR & (1<<TWINT)));

	//check to see if ack received
	if((TWSR & 0xF8) != 0x40)	
	printf("third ack problem 0x%x \n\r",(TWSR & 0xF8));
	//	printf("third ack recieved OK 0x%x \n\r",(TWSR & 0xF8));
	
	TWCR = (1<<TWINT)|(1<<TWEN);

	
	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while(!(TWCR & (1<<TWINT)));

	//check to see if ack received
	if((TWSR & 0xF8) != 0x58)	
	printf("third ack problem 0x%x \n\r",(TWSR & 0xF8));
	//	printf("third ack recieved OK 0x%x \n\r",(TWSR & 0xF8));

	// transmit STOP condition 
	TWCR = (1<<TWINT)|(1<<TWSTO);
	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	//while(!(TWCR & (1<<TWINT)));
	//_delay_ms(10);
	return TWDR;
}

// ??BH?? insert switch-case structure for debug vs normal operation code
int main(void)
{
	init();	
	sei();	

	while(1)
	{
		switch(system_state & 0x0F) {

			case TEST :				
				_delay_ms(500);
	            setLED(50,50,50);
	            _delay_ms(500);
	            setLED(0,0,0);

				//output.speed_dock_m5_m=225;
				//output.direction_dock_m5_m=0;
				//output.direction_bend_m3_m=1; // 0 positive
				//output.speed_bend_m3_m=225;
				//output.vibration_m=1;
				printf("m %d %d %d \n\r",input.accell_m[0],input.accell_m[1], input.accell_m[2]);
				//printf("%#08X \n\r", IMU_ADDRESS);
				//printf("IR %d %d \n\r", input.IR1_m, input.IR2_m);
				//nput.IR2_m=get_IR_U5();
				//input.IR1_m=get_IR_Flex_U1513();
				//printf("IR %d \n\r", input.IR1_m); //for bend sensor reading only - note change function name to reflect.
							
				//you can adjust this delay.. eventually if too small it may cause problems, but you can fix this by changing it back
				_delay_ms(20);
				master_output_update();
				master_input_update();		
			break;

			case EXPERIMENT :

				switch(system_state & 0xF0) {

					case SETUP :
					break;

					case FLIP :
					break;
				}
			break;
		}
	}
}

void master_output_update() //motor updates
{

	if(output.direction_dock_m5_m==0)
	{
		DDRB |= (1<<6); //set direction dock motor (M5) PB6 output
		PORTB &= ~(1<<6); //set dock motor (M5) PB6 low - (in2/in4 on hbridge)
		OCR0B = output.speed_dock_m5_m; //sets PWM for OCR0B (PD5) - in1/in3 on hbridge - motor goes forward on high portion

	}
	else
	{
		DDRB |= (1<<6); //set direction dock motor (M5) PB6 output
		PORTB |= (1<<6); //set dock motor (M5) PB6 high - (in2 and in4 on hbridge)
		OCR0B = 255-output.speed_dock_m5_m; //sets PWM for OCR0B (PD5) - in1/in3 on hbridge - motor goes backward on low portion
	}


	if(output.direction_bend_m3_m==0)
	{
		DDRB |= (1<<7); //set direction bend motor (M3) PB7 output
		PORTB &= ~(1<<7); //set bend motor (M3) PB7 low - (in2/in4 on hbridge)
		OCR0A = output.speed_bend_m3_m; //sets PWM for OCR0A (PD6) - in1/in3 on hbridge - motor goes forward on high portion

	}
	else
	{
		DDRB |= (1<<7); //set direction bend motor (M3) PB7 output
		PORTB |= (1<<7); //set bend motor (M3) PB7 high - (in2/in4 on hbridge)
		OCR0A = 255-output.speed_bend_m3_m; //sets PWM for OCR0A (PD6) - in1/in3 on hbridge - motor goes backward on low portion
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
}

void master_input_update()  
{
	//	input.switch_dock_m=switch_dock();
	//	input.switch_tension_m=switch_tension1();
	//	input.bend_m=get_bend();
	input.IR1_m=get_IR_Flex_U1513();
	//	printf("%d \n\r",input.IR1_m);
	input.IR2_m=get_IR_U5();
	//	printf("%d \n\r",input.IR2_m);

	// Get accel data from master side
	get_IMU_measurement();	
}

void get_IMU_measurement(void)
{	
	// Measurement data is stored in two’s complement and Little Endian format. 
	// Measurement range of each axis is from -32752 ~ 32752 decimal in 16-bit output.
	// 0x8010 is -32752, 0x7ff0 is 32752

	i2c_write_accell(IMU_ADDRESS,PWR_MGMT_1,0); 
	int x=((i2c_read_accell( IMU_ADDRESS, ACCEL_XOUT_H)<<8)&0xff00)+(i2c_read_accell( IMU_ADDRESS, ACCEL_XOUT_L)&0x00ff);
	int y=((i2c_read_accell( IMU_ADDRESS, ACCEL_YOUT_H)<<8)&0xff00)+(i2c_read_accell( IMU_ADDRESS, ACCEL_YOUT_L)&0x00ff);			
	int z=((i2c_read_accell( IMU_ADDRESS, ACCEL_ZOUT_H)<<8)&0xff00)+(i2c_read_accell( IMU_ADDRESS, ACCEL_ZOUT_L)&0x00ff);

	// Convert two's complement for negative values, 0x8010 = -32752 is the lowest 
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

    // Update values input to MCU from IMU
	input.accell_m[0]=x;
	input.accell_m[1]=y;
	input.accell_m[2]=z;
}

///////////////////INPUT READ FUNCTIONS

int switch_tension1(void) //Rewrite more efficiently? e.g. as a macro?
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

int switch_power(void) //Rewrite more efficiently? e.g. as a macro?
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

int get_IR_Flex_U1513(void)
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

int get_IR_U5(void)
{	
	     //initalize adc
		ADMUX &= (1<<REFS0); //choose reference and clear analog pins
		ADCSRB |= (1<<ACME); 
		ADCSRA &= ~(1<<ADEN);
		ADMUX |=0x02;//choose analog pin ADC2 connected to IR1
		ADCSRA = (1<<ADEN) |    (1<<ADPS0); //set up a/d


		//when the following code was in main, while loop started here
		ADCSRA |= (1<<ADSC);//start adc conversion to sample sensor with led off
		while((ADCSRA&(1<<ADSC))!=0);//busy wait for conversion to end
	//	printf("%d \n\r",ADCW);	

	return(ADCW);

}

//////////////////OUTPUT FUNCTIONS

void setLED(unsigned char red, unsigned char green, unsigned char blue)
{
	//Bit banging 20-600kHz
	//Send LSB first
	
	unsigned char array[4] = {led_wrt_cmd, red, blue,green};

	for(char byte = 0; byte <= 3; byte++)  //Go through loop 4x - first write command, then each rgb
	{
		for(unsigned char bit=0; bit<=7; bit++)
		{
			//bit initiation
			set(led_port, led_pin);
			_delay_us(2);
			clear(led_port, led_pin);
			_delay_us(3);

			if(array[byte] & (led_pin>>bit))
				set(led_port, led_pin);
			else
				clear(led_port, led_pin);
			_delay_us(10); 
			clear(led_port, led_pin);

			_delay_us(5);  //~50kHz	
		}
	}

	_delay_us(80);//End of Sequence
}
