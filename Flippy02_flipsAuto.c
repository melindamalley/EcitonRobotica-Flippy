#define F_CPU 8000000UL

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <math.h>
#include <stdio.h>
#include <avr/interrupt.h>
#define FOSC 8000000
#define BAUD 9600
#define MYUBRR (((((FOSC * 10) / (16L * BAUD)) + 5) / 10))


#define ee_POWER_STATE 0x00
#define accell_slave  0b11010000
#define	accell_master 0b11010010
#define atmega_slave 0xf0
#define num_a2d_samples 15

#define bendoffset_m 90
#define bendoffset_s 255
#define maxbend_m 540 //490 //555 for bending controlled by slave
#define maxbend_s 890 //795 //810for bending controlled by master
#define neutralbend 120
#define windspd 100
#define unwindspd 150


#define MASTER 1

void setLED(unsigned char red, unsigned char green, unsigned char blue);
int switch_power(void);
int switch_tension1(void);
int switch_dock(void);
int get_bend(void);
void master_output_update(void);
void master_input_update(void);
void flipbend(char side);

struct inputs{
	uint8_t	switch_power;
 	uint8_t switch_tension_m;
	uint8_t switch_tension_s;
	uint8_t switch_dock_m;
	uint8_t switch_dock_s;
	int bend_s;
	int bend_m;
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

void ioinit (void) {
    UBRR0H = MYUBRR >> 8;
    UBRR0L = MYUBRR;
    UCSR0B = (1<<TXEN0);
    
    fdev_setup_stream(&mystdout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &mystdout;
}


void init(void)
{

	//enable printf output
	DDRC=0;
	PORTC=0;
	DDRB=0;
	PORTB=0;
	OCR0B = 0xff;// motor off
	OCR0A = 0x00;// motor off
	DDRB=0;
	PORTB=0;
	DDRC=0;
	PORTC=0;
	DDRD=0;
	PORTD=0;

	 ioinit();

	sleep_mode=0;
	//power on 
	DDRC |= (1<<0);
	PORTC |= (1<<0);

	//rgb led init
	DDRD |= (1<<0);

	DDRB &= ~(1<<1);
	DDRD &= ~(1<<4);
	DDRD &= ~(1<<3);//power switch as input

	DDRD |= (1<<5);
	DDRD |= (1<<6);
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM00);
	TCCR0B =0x03; //prescaler set to 0
	OCR0B = 0x00;//start with motor off
	OCR0A = 0x00;//start with motor off

	DDRC &= ~(1>>1);
	//initalize adc
	ADMUX = (1<<REFS0)|(1<<MUX0);//choose analog pin 
	ADCSRA = (1<<ADEN) |    (1<<ADPS0); //set up a/d

			//enable power switch interrupt
	DDRD |= 1<<3;		// Set PD2 as input (Using for interupt INT0)
	PORTD |= 1<<3;		// Enable PD2 pull-up resistor
	EIMSK = 1<<1;					// Enable INT0
	EICRA = 1<<ISC01 | 1<<ISC00;	// Trigger INT0 on rising edge 
	



	

}
ISR(TWI_vect)
{
cli();
	printf("twi int\n\r");
	
	TWCR &= ~(1<<TWIE);
	
		TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
}

ISR(INT1_vect)
{
	
	while((PIND & (1<<3))==0)
	{

//printf("power state %d, pin %d\n\r",power_state,(PIND & (1<<3)));
	}
	if(power_state==1)
	{
	
		printf("sleeping  \n\r\n\r");
		sleep_mode=0;
	//	TWCR= (1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
		_delay_ms(50);
		i2c_send();

		int i;
		for(i=50;i>0;i--)
		{
			setLED(i,0,0);
		_delay_ms(10);
		}

		DDRC &= ~(1<<4);
		PORTC &= ~(1<<4);
		DDRD &= ~(1<<5);
		PORTD &= ~(1<<5);

		DDRC &= ~(1<<3);
		PORTC &= ~(1<<3);
		DDRD &= ~(1<<6);
		PORTD &= ~(1<<6);



		DDRC &= ~(1<<0);
		PORTC &= ~(1<<0);

		power_state=0;
		
	

		set_sleep_mode (SLEEP_MODE_PWR_SAVE); 
		sei();
		sleep_mode();
		cli();
		
		//power off 
	
		printf("wakup \n\r\n\r");
		//enable printf output
		init();
		sleep_mode=1;
		power_state=1;
	//	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	
		i2c_send();
		for(i=0;i<50;i++)
		{
			setLED(0,i,0);
		_delay_ms(10);
		}
		
			_delay_ms(500);
			setLED(0,0,0);
	

	}
	else
	{
		//printf("setting powerstate 0\n\r");
		power_state=1;
	}


}	



//master sends commands to slave
int i2c_send()
{
	cli();

	
	
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
	TWDR=atmega_slave ;	 

	//start transmission
	TWCR=(1<<TWINT)|(1<<TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while(!(TWCR & (1<<TWINT)));

	//check to see if ack received
	if((TWSR & 0xF8) != 0x18)	{
	printf("1 ack problem 0x%x \n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}
	

	TWDR=output.speed_bend_s;



	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}



	TWDR=output.speed_dock_s;

	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{
	printf("3 ack problem is 0x%x\n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}

	TWDR=output.direction_dock_s;

	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	{
	printf("4 ack problem is 0x%x\n\r",(TWSR & 0xF8));
	TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
	sei();
	return (-1);
	}
	TWDR=output.direction_bend_s;

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

	TWCR = (1<<TWINT)|(1<<TWSTO);


	
	


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




uint8_t i2c_write_accell(uint8_t accell,uint8_t address,uint8_t data)
{
	//start twi transmission
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	//wait for TWCR flag to set indication start is transmitted
	while(!(TWCR &(1<<TWINT)));

	//check to see if start is an error or not
	if((TWSR & 0xF8) != 0x08)
	printf("twi error\n\r");

	//Load write addres of slave in to TWDR, 
	
	//uint8_t SLA_W=0b11010000;
	uint8_t SLA_W=accell;
	TWDR=SLA_W;

	//start transmission
	TWCR=(1<<TWINT)|(1<<TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while(!(TWCR & (1<<TWINT)));

	//check to see if ack received
	if((TWSR & 0xF8) != 0x18)	
	printf("first ack problem 0x%x \n\r",(TWSR & 0xF8));
//	printf("ack recieved OK 0x%x \n\r",(TWSR & 0xF8));


	//send data, in this case an address
	TWDR=address;//0x6b;// accell x value msb's
	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	
	printf("second ack problem is 0x%x\n\r",(TWSR & 0xF8));
//	printf("second ack received OK is 0x%x\n\r",(TWSR & 0xF8));


/////

	//send data, in this case an address
	TWDR=data;//0x00;// accell x value msb's
	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	
	printf("third ack problem is 0x%x\n\r",(TWSR & 0xF8));
//	printf("third ack received OK is 0x%x\n\r",(TWSR & 0xF8));

	//send stop bit
	TWCR = (1<<TWINT)|(1<<TWSTO);

	return(0);

}
uint8_t i2c_read_accell(uint8_t accell,uint8_t address)
{

	//start twi transmission
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	//wait for TWCR flag to set indication start is transmitted
	while(!(TWCR &(1<<TWINT)));

	//check to see if start is an error or not
	if((TWSR & 0xF8) != 0x08)
	printf("twi error\n\r");

	//Load write addres of slave in to TWDR, 
	//uint8_t	SLA_W=0b11010000;
	uint8_t	SLA_W=accell;
	TWDR=SLA_W;

	//start transmission
	TWCR=(1<<TWINT)|(1<<TWEN);

	//wait for TWINT flag to se, indicating transmission and ack/nack receive
	while(!(TWCR & (1<<TWINT)));

	//check to see if ack received
	if((TWSR & 0xF8) != 0x18)	
	printf("first ack problem 0x%x \n\r",(TWSR & 0xF8));
//	printf("ack recieved OK 0x%x \n\r",(TWSR & 0xF8));

	//send data, in this case an address
	TWDR=address;// accell x value msb's
	TWCR = (1<<TWINT)|(1<<TWEN);//start tx of data

	while(!(TWCR&(1<<TWINT)));//wait for data to tx

	//check for data ack	
	if((TWSR & 0xF8) != 0x28)	
	printf("second ack problem is 0x%x\n\r",(TWSR & 0xF8));
//	printf("second ack received OK is 0x%x\n\r",(TWSR & 0xF8));

	//send stop bit
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);


	////master receiver mode
	//start twi transmission
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	//wait for TWCR flag to set indication start is transmitted
	while(!(TWCR &(1<<TWINT)));


	//check to see if start is an error or not
	if((TWSR & 0xF8) != 0x10)
	printf("start condition error 0x%x\n\r",(TWSR & 0xF8));

	//Load read addres of slave in to TWDR, 
	 SLA_W=accell | 1;//	 SLA_W=0b11010001;
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
	
	TWCR = (1<<TWINT)|(1<<TWSTO);
//wait for TWINT flag to se, indicating transmission and ack/nack receive
	//while(!(TWCR & (1<<TWINT)));
	//_delay_ms(10);
	return TWDR;

}


int main(void)
{

	init();	
		//enable power switch interrupt
	DDRD |= 1<<3;		// Set PD2 as input (Using for interupt INT0)
	PORTD |= 1<<3;		// Enable PD2 pull-up resistor
	EIMSK = 1<<1;					// Enable INT0
	EICRA = 1<<ISC01 | 1<<ISC00;	// Trigger INT0 on rising edge 
	sei();	

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
	DDRC |= (1<<2);
	PORTC &= ~(1<<2);
	OCR0B = 0;	
	
	//bend motor off
	DDRC |= (1<<3);
	PORTC &= ~(1<<3);
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
char toggle=0;
int count=0;
char side=0;
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

//			printf("s bend %d\n\r",input.bend_s);
//			printf("m bend %d\n\r",input.bend_m);

/*
//Docking

if ((input.switch_dock_m==1)&&(input.switch_dock_s==1)){
	output.led_m[0]=0;
	output.led_m[1]=0;
	output.speed_dock_s=0;
	output.speed_dock_m=0;
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
	if (input.switch_dock_m==1){
		output.led_m[0]=20;
		output.led_s[1]=20;
		output.direction_dock_m=1;
		output.speed_dock_m=200;
	}
	else if (input.switch_dock_s==1){
		output.led_m[1]=20;
		output.direction_dock_m=0;
		output.speed_dock_m=200;
	}
	else{
		output.led_m[0]=0;
		output.led_m[1]=0;
		output.speed_dock_m=0;
		}
}
else{
	if (input.switch_dock_m==1){
		output.led_m[0]=20;
		output.direction_dock_s=1;
		output.speed_dock_s=200;
	}
	else if (input.switch_dock_s==1){
		output.led_m[1]=20;
		output.direction_dock_s=0;
		output.speed_dock_s=200;
	}
	else{
		output.led_m[0]=0;
		output.led_m[1]=0;
		output.speed_dock_s=0;
		}
}

//printf("%d \n\r", state);
*/

//Flipping Programs

	switch(state){
		case 0: //do nothing and unwind motors
			output.led_m[1]=20;
			output.led_s[1]=20;
			output.speed_bend_m=0;
			output.speed_bend_s=0;
			output.speed_dock_m=0;
			output.speed_dock_s=0;
			
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
				output.direction_bend_m=1;
				output.speed_bend_m=100;
			}
			else if(input.switch_dock_s==1)
			{
				output.led_s[0]=30;	
				output.led_s[1]=0;
				output.direction_bend_s=1;
				output.speed_bend_s=100;
			}
			break;

		case 1:	//flipping
			output.led_m[0]=20;
			output.direction_dock_m=0; //spin the opposite dock motor to prevent attaching
			output.speed_dock_m=200;
			flipdir=0;
			if (toggle==0){
				output.direction_bend_m=1; //master going back, slave going forward
				output.direction_bend_s=0;
				output.speed_bend_s=windspd;
				output.speed_bend_m=unwindspd;
				count++;
				if(count>12){
					toggle=1;
//					printf("toggle");
					count=0;
					output.led_m[2]=20;
					}
				}
			if ((input.switch_dock_m==1)&&(toggle==1)){
				count++;
				if (count>3){
					output.speed_bend_m=0;
					output.speed_bend_s=0;
					output.speed_dock_m=0;
					output.led_m[0]=0;
					output.led_m[2]=0;
					count=0;
					toggle=0;
					state=3;
					break;
					}
				}
//			else {if (input.bend_m<maxbend_m){
			else if(toggle==1){
			flipbend(flipdir);
//			printf("%d %d \n\r",input.bend_m, input.bend_s);
			}
//			else{
//			output.speed_bend_m=0;
//			output.speed_bend_s=0;
//			output.led_m[0]=0;
//			count=0;
//			toggle=0;
//			printf("%d\n\r",input.bend_m);
//			state=3;
//			}
			break;

		case 2: //flipping
			output.direction_dock_s=0; //spin the opposite dock motor to prevent attaching
			output.speed_dock_s=200;
			output.led_s[0]=20;
			flipdir=1;
//			printf("m bend %d\n\r",input.bend_m);
			if (toggle==0){
				output.direction_bend_m=0; //master going forward, slave going back
				output.direction_bend_s=1;
				output.speed_bend_s=unwindspd;
				output.speed_bend_m=windspd;
				count++;
				if(count>12){
					toggle=1;
					count=0;
					output.led_s[2]=20;
					}
				}
			if ((input.switch_dock_s==1)&&(toggle==1)){
				count++;
				if (count>4){
					output.speed_bend_m=0;
					output.speed_bend_s=0;
					output.speed_dock_m=0;
					output.led_s[0]=0;
					output.led_s[2]=0;
					count=0;
					toggle=0;
					state=4;
					break;
					}
				}
//			if (input.bend_s>1000){
//				output.speed_bend_m=0;
//				output.speed_bend_s=0;
//				}
//			else if (input.bend_s<maxbend_s){
			else if(toggle==1){
				flipbend(flipdir);	
//			printf("%d \n\r", input.bend_s);
				}
//			else{
//			output.speed_bend_s=0;
//			output.speed_bend_m=0;
//			output.led_s[0]=0;
//			toggle=0;
//			printf("%d\n\r",input.bend_s);
//			state=4;
//			}
			break;

		case 3: //attaching master side
			output.led_m[1]=20;
			output.speed_bend_s=0;
			output.speed_bend_m=0;
////////////manual attachment
//			if (input.switch_dock_m==1){
//				output.direction_dock_m=1;
//				output.speed_dock_m=200;
//				toggle=1;
//			printf("%c\n\r",toggle);
//			}
//			else if(toggle==1){
//				output.speed_dock_m=0;
//				output.led_m[1]=0;
//				state=7;
//				}
///////////////////

//// automatic attachment
			output.direction_dock_m=1;
			output.speed_dock_m=200;
			if (count>20){
				count=0;
				output.led_m[1]=0;	
				output.speed_dock_m=0;
				state=7;
			}
			count++;
////
			break;
		case 4: //attaching slave side
			output.led_s[1]=20;
			output.speed_bend_s=0;
			output.speed_bend_m=0;

////////////manual attachment
//			if (input.switch_dock_s==1){
//				output.direction_dock_s=1;
//				output.speed_dock_s=200;
//				toggle=1;
//			}
//			else if(toggle==1){
//				output.speed_dock_s=0;
//				output.led_s[1]=0;
//				state=5;
//				count=0;
//				}
////////////

/////automatic attachment
			output.direction_dock_s=1;
			output.speed_dock_s=200;
			if (count>20){
				count=0;
				output.led_s[1]=0;
				output.speed_dock_s=0;
				state=5;	
			}
			count++;
//////
			break;
		case 5: //detaching the master side
				output.speed_bend_m=0;
				output.speed_bend_s=0;
				output.led_m[2]=20;
				output.direction_dock_m=0;
				output.speed_dock_m=200;
				side=0;
				count++;
			if (input.switch_dock_m==0){
				state=-(flipdir-2);
				output.speed_bend_m=0;
				output.speed_bend_s=0;
				output.led_m[2]=0;
				count=0;
				_delay_ms(1000);
				}
			else if (count>25){
				state=6;
				count=0;
				}
			break;
		case 6: //detaching the master side - flipping disturbance
				output.led_m[2]=20;
//				output.direction_dock_m=0;
//				output.speed_dock_m=100;
			if (count<3){
				flipbend(-(flipdir-1));
				}
//			else if (count<6){
//				flipbend(flipdir);
//				}
//				printf("flipbend");
				count++;
			if (input.switch_dock_m==0){
				state=-(flipdir-2);
				count=0;
				output.led_m[2]=0;
				output.speed_bend_m=0;
				output.speed_bend_s=0;
				}
			else if (count>3){
				state=5;
				count=0;
				}
			break;
		case 7: //detaching the slave side
				output.speed_bend_m=0;
				output.speed_bend_s=0;
				output.led_s[2]=20;
				output.direction_dock_s=0;
				output.speed_dock_s=200;
				side=0;
				count++;
			if (input.switch_dock_s==0){
				//state=2;
				count=0;
				state=-(flipdir-2);
				output.led_s[2]=0;
				_delay_ms(1000);
				}
			else if (count>25){
				state=8;
				count=0;
				}
			break;
		case 8: //detaching the slave side - flipping disturbance
				output.led_s[2]=20;
//				output.direction_dock_m=0;
//				output.speed_dock_m=100;
			if (count<3){
				flipbend(-(flipdir-1));
				}
//			else if (count<6){
//				flipbend(flipdir);
//				}
//				printf("flipbend");
				count++;
			if (input.switch_dock_s==0){
				//state=1;
				state=-(flipdir-2);
				count=0;
				output.led_s[2]=0;
				output.speed_bend_m=0;
				output.speed_bend_s=0;
				}
			else if (count>3){
				state=7;
				count=0;
				}
			break;
		case 10: //rewind motors
			output.direction_bend_m=0;
			output.direction_bend_s=0;
			if ((input.switch_dock_m==0)&(input.switch_dock_s==0)){
				toggle=1;
				}
			if (toggle==1){
				if(input.switch_tension_m==1){
					output.speed_bend_m=60;
					}
				else {
					output.speed_bend_m=0;
					}
				if(input.switch_tension_s==1) {
					output.speed_bend_s=60;
					}
				else {
					output.speed_bend_s=0;
					}	
				}
			if ((input.switch_tension_s==0)&(input.switch_tension_m==0)&(toggle=1)){
				if(input.switch_dock_m==1){
				state=11;
				toggle=0;
				_delay_ms(2000);
				}
				}
				break;
		case 11: //attach or detach slave side
				output.led_m[0]=20;
			if ((input.switch_dock_m==1)&&(input.switch_dock_s==1)){
				output.led_s[0]=0;
				output.led_s[1]=0;
				output.led_m[0]=0;
				output.speed_dock_s=0;
				output.speed_dock_m=0;
				state=12;
				_delay_ms(2000);
				break;
				}
			else if (input.switch_dock_m==1){
				output.led_s[0]=20;
				output.led_s[1]=0;
				output.direction_dock_s=1;
				output.speed_dock_s=200;
				}
			else if (input.switch_dock_s==1){
				output.led_s[0]=0;
				output.led_s[1]=20;
				output.direction_dock_s=0;
				output.speed_dock_s=200;
				}
			else{
				output.led_s[0]=0;
				output.led_s[1]=0;
				output.speed_dock_s=0;
				}
				break;
	case 12: //attach or detach master side
			output.led_s[0]=20;
		if ((input.switch_dock_m==1)&&(input.switch_dock_s==1)){
			output.led_m[0]=0;
			output.led_m[1]=0;
			output.led_s[0]=0;
			output.speed_dock_s=0;
			output.speed_dock_m=0;
			state=1;
			_delay_ms(9000);
			break;
			}
		if (input.switch_dock_m==1){
			output.led_m[0]=20;
			output.led_m[1]=0;
			output.direction_dock_m=1;
			output.speed_dock_m=200;
		}
		else if (input.switch_dock_s==1){
			output.led_m[0]=0;
			output.led_m[1]=20;
			output.direction_dock_m=0;
			output.speed_dock_m=200;
		}
		else{
			output.led_m[0]=0;
			output.led_m[1]=0;
			output.speed_dock_m=0;
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
			output.speed_bend_s=0; //speed of bend motor for slave 0-255
			output.speed_dock_s=1; //speed of dock motor for slave 0-255
			output.direction_dock_s=2; //direction of dock motor for slave 0,1
			output.direction_bend_s=3; //direction of bend motor for slave 0,1
			output.led_s[0]=4; //slave led (RED)0-255
			output.led_s[1]=5;//slave led (GREEN)0-255
			output.led_s[2]=6;//slave led(BLUE)0-255
			//master side		
			output.speed_bend_m=0; //master of bend motor for slave 0-255
			output.speed_dock_m=1; //master of dock motor for slave 0-255
			output.direction_dock_m=2; //direction of dock motor for master 0,1
			output.direction_bend_m=3; //direction of bend motor for master 0,1
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
			_delay_ms(10);




		
			
		}
		else//slave code 
		{
	
			////////////
			//slave code, do not modify
			////////////
			static uint8_t dock_speed=0;
			static uint8_t flex_speed=0;

			static uint8_t tx_data_counter=0;
			static uint8_t rx_data_count=0;
			if((TWCR & (1<<TWINT)))
			{
//				printf("status 0x%x , count %d\n\r",(TWSR & 0xF8),rx_data_count);
				//slave receiver stuff
				if((TWSR & 0xF8)==0x60)
				{
					TWCR= (1<<TWEA)|(1<<TWEN)|(1<<TWINT);
					rx_data_count=0;
				//	printf("slave rx address start\n\r");
				}
				else if((TWSR & 0xF8)==0x80)
				{
					if(rx_data_count==0)
					{
						flex_speed=TWDR;
					}
					else if(rx_data_count==1)
					{
						//dock speed
						dock_speed=TWDR;
					}
					else if(rx_data_count==2)
					{
						//dock direction
							
						if(TWDR==0)
						{
						DDRC |= (1<<2);
						PORTC &= ~(1<<2);
						OCR0B = dock_speed;

						}
						else
						{
							DDRC |= (1<<2);
							PORTC |= (1<<2);
							OCR0B = 255-dock_speed;
						}
						

					}
					else if(rx_data_count==3)
					{
						if(TWDR==0)
						{
						DDRC |= (1<<3);
						PORTC &= ~(1<<3);
						OCR0A = flex_speed;

						}
						else
						{
							DDRC |= (1<<3);
							PORTC |= (1<<3);
							OCR0A = 255-flex_speed;
						}
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
					TWDR=switch_tension1();
					TWCR=(1<<TWINT)|(1<<TWEA)|(1<<TWEN);
				
				}
				else if(((TWSR & 0xF8)==0xb8))
				{
				
								
					if(tx_data_counter==1)
					TWDR=switch_dock();
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
					printf("buss error? i2c data 0x%x\n\r",(TWSR & 0xF8));
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

			

			
				DDRC &= ~(1<<4);
				PORTC &= ~(1<<4);
				DDRD &= ~(1<<5);
				PORTD &= ~(1<<5);

				DDRC &= ~(1<<3);
				PORTC &= ~(1<<3);
				DDRD &= ~(1<<6);
				PORTD &= ~(1<<6);

				

				DDRC &= ~(1<<0);
				PORTC &= ~(1<<0);

				sei();     
				TWCR |= (1<<TWIE);
		        SMCR = (1<<SE);
		        asm volatile("sleep\n\t");
				SMCR = 0;
				DDRC |= (1<<0);
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

	if((PINB & (1<<1))!=0)//switch b
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

	if((PIND & (1<<3))!=0)//switch b
	{
		return(0);
	}
	else
	{
		return(1);
	}
}

int switch_dock(void)
{
	
	if((PIND & (1<<4))!=0)//switch b
	{
		return(0);
	}
	else
	{
		return(1);
	}


}

void flipbend(char side){
	if (side==1){ //master side
			output.direction_bend_m=0; //master going forward, slave going back
			output.direction_bend_s=1;
		if(input.switch_tension_s==0){
			output.speed_bend_s=unwindspd;
		}
		else {
			output.speed_bend_s=0;
		}
	output.speed_bend_m=windspd;	
	}
	else{ //slave side
		output.direction_bend_m=1; //slave going forward, master going back
		output.direction_bend_s=0;
		if(input.switch_tension_m==0){
			output.speed_bend_m=unwindspd;
		}
		else {
			output.speed_bend_m=0;
		}
	output.speed_bend_s=windspd;
	}
}


int get_bend(void)
{
	int i=0;
	double samples[num_a2d_samples];

	for(i=0;i<num_a2d_samples;i++)
	{
	
	     //initalize adc
		ADMUX = (1<<REFS0)|(1<<MUX0);//choose analog pin 
		ADCSRA = (1<<ADEN) |    (1<<ADPS0); //set up a/d




		//when the following code was in main, while loop started here
		ADCSRA |= (1<<ADSC);//start adc conversion to sample sensor with led off
		while((ADCSRA&(1<<ADSC))!=0);//busy wait for converstion to end

	//	printf("%d \n\r",ADCW);

		samples[i]=ADCW;
	}

	//compute average
	double sum=0;
	for(i=0;i<num_a2d_samples;i++)
	{
		sum+=samples[i];
	}
	int average=sum/num_a2d_samples;

	//compute standard deviation
	double variance=0;

	for(i=0;i<num_a2d_samples;i++)
	{
		variance+=(samples[i]-average)*(samples[i]-average);

	}
	int stdev=sqrt(variance/(num_a2d_samples-1));

	sum=0;
	int num_samples=0;
	//compute average of samples within one STDev
	for(i=0;i<num_a2d_samples;i++)
	{
		if(((average+stdev)>samples[i])&&((average-stdev)<samples[i]))
		{
			sum+=samples[i];
			num_samples++;
		}

	}

	

	return((int)(sum/num_samples));

}
void master_output_update()
{

	if(output.direction_dock_m==0)
	{
		DDRC |= (1<<2);
		PORTC &= ~(1<<2);
		OCR0B = output.speed_dock_m;

	}
	else
	{
		DDRC |= (1<<2);
		PORTC |= (1<<2);
		OCR0B = 255-output.speed_dock_m;
	}


	if(output.direction_bend_m==0)
	{
		DDRC |= (1<<3);
		PORTC &= ~(1<<3);
		OCR0A = output.speed_bend_m;

	}
	else
	{
		DDRC |= (1<<3);
		PORTC |= (1<<3);
		OCR0A = 255-output.speed_bend_m;
	}


	setLED(output.led_m[0],output.led_m[1],output.led_m[2]);

}
void master_input_update()
{
	input.switch_dock_m=switch_dock();
	input.switch_tension_m=switch_tension1();
	input.bend_m=get_bend();
//	printf("bend is %d\n\r",input.bend_m);
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
			PORTD |= 0x01;
			_delay_us(2);
			PORTD &=~ 0x01;
			_delay_us(3);

			if(array[byte] & (0x80>>bit))
				PORTD |= 0x01;
			else
				PORTD &=~0x01;
			_delay_us(10); 
			PORTD &=~0x01;
			_delay_us(5);  //~50kHz	
		}
	}

	_delay_us(80);//End of Sequence
}
