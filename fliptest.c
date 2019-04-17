#define F_CPU 8000000UL

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <math.h>
#define ee_POWER_STATE 0x00


void setLED(unsigned char red, unsigned char green, unsigned char blue);
int switch_a(void);
int switch_b(void);
void motor_a(int);
void motor_b(int);
int bend_a(void);
int bend_b(void);



int main(void)
{
	
	DDRC=0;
	PORTC=0;
	DDRB=0;
	PORTB=0;
	OCR0B = 0x00;// motor off
	OCR0A = 0x00;// motor off
	DDRB=0;
	PORTB=0;
	DDRC=0;
	PORTC=0;
	DDRD=0;
	PORTD=0;

	DDRC |= (1<<5);

	DDRD |= (1<<5);
	DDRD |= (1<<6);
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM00);
	TCCR0B =0x03; //prescaler set to 0
	OCR0B = 0x00;//start with motor off
	OCR0A = 0x00;//start with motor off

	//initalize adc
	ADMUX = (1<<REFS0)|(1<<MUX1);//choose analog pin 
	ADCSRA = (1<<ADEN) |    (1<<ADPS0); //set up a/d
	

	PORTC &= ~(1<<4);


	//code starts here
	while(1)
	{
	
		_delay_ms(5);
		int red, green, blue;
		int state;
		int bend1;
		int bend_offset_1=150;
		bend1=(bend_a()-bend_offset_1)/1;

		int bend2;
		int bend_offset_2=70;
		bend2=(bend_b()-bend_offset_2)/1;
		state=1;

/*		if(bend1>800){
		red=20;
		}
		else {
		red=0;
		}

		if(bend2>600){
		green=20;
		}
		else {
		green=0;
		motor_b(0);
		}
		
		if((switch_a()==1)||(switch_b()==1)) {
		}
		else {
		blue=0;
		}

		setLED(red,green,blue); 
*/


	motor_a(0);
	motor_b(0);

// Program to rewind motors
/*		if(switch_a()==0) {
			motor_a(60);
			}
		else {
			motor_a(0);
			}
		if(switch_b()==0) {
			motor_b(50);
			}
		else {
			motor_b(0);
			}
*/


//state 1 is bend1>0.
//state 2 is bend2>0.


/*while(1){
	switch(state){
		case 1:
			setLED(0,0,30);
			while (bend1<872){
			if(switch_b()==1){
				motor_b(-100);
				}
			else {
				motor_b(0);
				}
			motor_a(200);
			bend1=(bend_a()-bend_offset_1)/1;
			bend2=(bend_b()-bend_offset_2)/1;
			}
			motor_a(0);
			motor_b(0);
			state=2;
			setLED(0,30,0);
			break;
		case 2:
			setLED(0,30,0);
			while (bend2<770){
			if(switch_a()==1){
				motor_a(-100);
				}
			else {
				motor_a(0);
				}
			motor_b(200);
			bend1=(bend_a()-bend_offset_1)/1;
			bend2=(bend_b()-bend_offset_2)/1;
			}
			motor_b(0);
			motor_a(0);
			state=1;
			setLED(0,0,30);
			break;
		case 3:
			motor_b(0);
			motor_a(0);
			break;
	}
}
*/

	
	//functions to use
	/*
	//blocking delay in ms
	_delay_ms(10);

	//set color, R,G,B, values 0-255
	setLED(0,0,0);

	//check status of switch a, 0 not pushed, 1 pushed
	switch_a();

	//check status of switch b, 0 not pushed, 1 pushed
	switch_b();

	//get bend sensor a value (0-1024)
	bend_a();

	//get bend sensor b value (0-1024)
	bend_b();

	//set motor a speed (-255 (CW) to 255 (CCW), 0 is off)
	motor_a(0);

	//set motor b speed (-255 (CW) to 255 (CCW), 0 is off)
	motor_b(0);

	//sets motor a to loose (no holding torque)
	motor_a_loose();

	//sets motor b to loose (no holding torque)
	motor_b_loose();

	*/






	}

}

int switch_b(void)
{

	if((PINB & (1<<0))!=0)//switch a
	{
		return(1);
	}
	else
	{
		return(0);
	}
}

int switch_a(void)
{

	if((PINB & (1<<1))!=0)//switch b
	{
		return(1);
	}
	else
	{
		return(0);
	}
}



void motor_a(int speed)
{

	if(speed>255)//switch b
	{
		speed=255;
	}
	else if(speed<-255)
	{
		speed=-255;
	}


	if(speed>=0)
	{
	

		DDRC |= (1<<4);
		PORTC |= (1<<4);
		OCR0B = (uint8_t)(255-speed);

	}
	else
	{
		DDRC |= (1<<4);
		PORTC &= ~(1<<4);
	
		OCR0B = (uint8_t)(255-speed);
	}
}

void motor_a_loose()
{

}

void motor_b_loose()
{

}

void motor_b(int speed)
{
	//correction for reverse wireing
	//speed=250;

	if(speed>255)//switch b
	{
		speed=255;
	}
	else if(speed<-255)
	{
		speed=-255;
	}


	if(speed<0)
	{
		//cw
		DDRC |= (1<<3);
		PORTC |= (1<<3);
		OCR0A = (uint8_t)(speed);

	}
	else
	{
			//ccw
		DDRC |= (1<<3);
		PORTC &= ~(1<<3);
	
		OCR0A = (uint8_t)(speed);		
	}
}



int bend_a(void)
{
	ADMUX &= 0xf0;//clear adc port selection 
	ADMUX |= 1;//set adc port to left rx
	//delay for 0.02 seconds
	_delay_ms(10);


	//when the following code was in main, while loop started here
	ADCSRA |= (1<<ADSC);//start adc conversion to sample sensor with led off
	while((ADCSRA&(1<<ADSC))!=0);//busy wait for converstion to end





	return(ADCW);

}

int bend_b(void)
{
	ADMUX &= 0xf0;//clear adc port selection 
	ADMUX |= 0;//set adc port to left rx
	//delay for 0.02 seconds
	_delay_ms(10);


	//when the following code was in main, while loop started here
	ADCSRA |= (1<<ADSC);//start adc conversion to sample sensor with led off
	while((ADCSRA&(1<<ADSC))!=0);//busy wait for converstion to end





	return(ADCW);


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
			PORTC |= (1<<5);
			_delay_us(2);
			PORTC &=~(1<<5);

			_delay_us(3);

			if(array[byte] & (0x80>>bit))
				PORTC |= (1<<5);
			else
				PORTC &=~(1<<5);

			_delay_us(10); 
			PORTC &=~(1<<5);

			_delay_us(5);  //~50kHz	
		}
	}
	_delay_us(80);//End of Sequence
}
