#define F_CPU 8000000UL //???

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <math.h>
#include <stdio.h>
#include <avr/interrupt.h>
#define FOSC 8000000 //???
#define BAUD 9600
#define MYUBRR (((((FOSC * 10) / (16L * BAUD)) + 5) / 10))n //???

#define ee_POWER_STATE 0x00 //???
#define accell_slave_addrs  0b11010000
#define	accell_master_addrs 0b11010010
#define atmega_slave 0xf0

//////////////////////////////////////
//
// MACROS FOR PIN HANDLING (STOLEN FROM JULIA)
#define output(directions,pin) (port |= pin) //set port direction for output
#define input(directions,pin) (directions &=(~pin)) //set port direction for input
//////////////////////////////////////

//SETUP AND DEFINE PINS

#define LED

//Flip Parameters:

#define windspd 130
#define unwindspd 220
#define gripperspd 180


#define MASTER 1 //choose master or slave 

void setLED(unsigned char red, unsigned char green, unsigned char blue);
int switch_power(void);
int switch_tension1(void);
int switch_dock(void);
int get_bend(void);
void master_output_update(void);
void master_input_update(void);
void flipbend(char side, char p);
double get_accel_diff(void);

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

struct outputs{
	uint8_t speed_bend_m;
	uint8_t speed_bend_s;
	uint8_t speed_dock_m;
	uint8_t speed_dock_s;
	uint8_t direction_dock_m;
	uint8_t direction_dock_s;
	uint8_t direction_bend_m;
	uint8_t direction_bend_s;
	uint8_t led_m[3];
	uint8_t led_s[3];
	int vibration_m;

};

struct outputs output;
struct inputs input;

static int power_state;
static uint8_t toggle_wakeup;
void init(void);
int i2c_send(void);
static uint8_t sleep_mode;
//setup for printf
int uart_putchar(char c, FILE *stream) { 
    if (c == '\n') 
        uart_putchar('\r', stream); 
    loop_until_bit_is_set(UCSR0A, UDRE0); 
    UDR0 = c; 

    return 0; 
}
FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void ioinit (void) { /*
    UBRR0H = MYUBRR >> 8; 
    UBRR0L = MYUBRR;
    UCSR0B = (1<<TXEN0);
    
    fdev_setup_stream(&mystdout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &mystdout;*/
}


void init(void)
{

	//enable printf output
	DDRB=0;
	PORTB=0;
	DDRC=0;
	PORTC=0;
	DDRD=0;
	PORTD=0;

	 ioinit(); //usart init

	sleep_mode=0;
	//power on 
	DDRC |= (1<<0); //output PortC0
	PORTC |= (1<<0);  //turn on PortC0 (Vreg1)

	//rgb led init
	DDRD |= (1<<7); //data direction output for rgb driver (Prev. PD0, now PD7)

	DDRB &= ~(1<<1); //tension switch as input
	DDRD &= ~(1<<4); //gripper/control switch as input
	DDRD &= ~(1<<3);//power switch as input

	DDRD |= (1<<5); //Hbridge 2-1 output
	DDRD |= (1<<6); //Hbridge 1-1 output
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM00); //Timer counter init
	TCCR0B =0x03; //prescaler set to 0
	OCR0B = 0x00;//start with motor off
	OCR0A = 0x00;//start with motor off
	DDRB |= (1<<0); //vibration motor output

	DDRC &= ~(1<<1); //flex sensor as input
	DDRC &= ~(1<<2); //IR1 as input
	DDRC &= ~(1<<3); //IR2/strain gage as input
	//initalize adc
	ADMUX &= (1<<REFS0); //|(1<<MUX0);//choose analog pin  
	ADCSRA = (1<<ADEN) | (1<<ADPS0); //set up a/d

			//enable power switch interrupt
	DDRD |= 1<<3;		// Set PD2 as input (Using for interupt INT0)
	PORTD |= 1<<3;		// Enable PD2 pull-up resistor
	EIMSK = 1<<1;					// Enable INT0
	EICRA = 1<<ISC01 | 1<<ISC00;	// Trigger INT0 on rising edge 

  	TWBR=0x04; //twi bit rate set 
	
    //dock motor off
	DDRB |= (1<<6);
	PORTB &= ~(1<<6);
	OCR0B = 0;	
	
	//bend motor off
	DDRB |= (1<<7);
	PORTB &= ~(1<<7);
	OCR0A = 0;

	power_state=1;
	sleep_mode=1;
	toggle_wakeup=0;

}
ISR(TWI_vect) //SIG_2WIRE_SERIAL - 2 wire Serial interface
{
}	


int main(void)
{

	init();	
	sei();	

	while(1)
	{	
			_delay_ms(500);

            setLED(50,50,50);

            _delay_ms(500);

            setLED(0,0,0);

			//you can adjust this delay.. eventually if too small it may cause problems, but you can fix this by changing it back
			_delay_ms(20);

	}

}

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
