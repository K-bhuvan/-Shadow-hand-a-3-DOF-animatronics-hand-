/*
 * anematronics.c
 *
 * Created: 4/24/2020 11:34:49 PM
 *  Author: bhuvan and Ram
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <stdio.h>
#include <avr/io.h>
//#include <Servo.h>
#include <avr/interrupt.h>
#include "Ring_Buffer.h"

#define BAUD 9600
#define BAUDRATE ((F_CPU/(BAUD*16UL))-1)
 
// Flag to raise for sending data to MATLAB
volatile uint8_t flag_0 = 0;

// nth order Filter co-efficients
//float a[4] = 
//float b[4] =

//defining servos
//Servo servo1;         //Connect them to the digital pin says 1
//Servo servo2;         //Connect them to the digital pin says 2// 
//Servo servo3;         //Connect them to the digital pin says 3// 3 is for the middle finger::connected to A2


//initialization
void pin_init(void){
	
	//A0 is connected to flex sensor 1
	//A1 is connected to flex sensor 2
	//A3 is connected to flex sensor 3
	DDRB = 0xFF;
	//D1 is connected to servo1
	//D2 is connected to servo2
	//D3 is connected to servo3 
	
}

void pwm_init(void){
	TCNT2 = 0;
	// Fast PWM mode with 0xff as TOP and compare output mode for fast PWM in non-inverting mode
	TCCR2A = (1<<COM2A1)|(1<<WGM21)|(1<<WGM20);
	// Fast PWM mode with 0xff as TOP and 8 prescaler for TIMER2
	TCCR2B = (1<<CS21)|(0<<CS20);
}

void timer_init(void){
	// Set initial timer value
	TCNT0 = 0; // Serial communication
	TCNT1 = 0; //  
	// CTC mode with prescaler as 8 for TIMER1
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
	// Set ICR1A as top value for 10ms duty cycle
	ICR1 = 4999;
	// CTC mode with prescaler as 1024 for TIMER0
	TCCR0B = (1<<WGM02)|(1<<CS02)|(1<<CS00);
	// Compare value for 10ms sampling rate
	OCR0A = 156;
	// Enable compare interrupt
	TIMSK1 |= (1<<OCIE1A);
	TIMSK0 = (1<<OCIE0A);		
}

ISR(TIMER1_COMPA_vect){
	PORTB = 0xFF;
}

ISR(TIMER0_COMPA_vect){
	flag_0 = 1;
}

// Initialize ADC
void adc_init(void)
{
	// Set V_ref and set to read from pin ADC1
	ADMUX = (1<<REFS0);
	// Set prescaler to 128 and Enable ADC
	ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}

// Read and return ADC value
uint16_t adc_read()
{
	// Return the ADC value
	return(ADC);
}



void uart_init(void)
{
	// Initialize baudrate registers
	UBRR0H = (BAUDRATE>>8);
	UBRR0L = BAUDRATE;
	// Initialize transmission and receiver pins
	UCSR0B = (1<<TXEN0)|(1<<RXEN0);
	// Set data bits to be 8-bits long
	UCSR0C = (0<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01);
}

// Transmit data serially
void uart_transmit(unsigned char data)
{
	// Load to UART buffer
	UDR0 = data;
}

// Function to interpolate middle finger position
int sensor_middle_to_motor(int sense_pos)
{
	int motor_pos;
	motor_pos = (-4.7*sense_pos) + 2494;
	return motor_pos;
}

// Function to interpolate index finger position
int sensor_index_to_motor(int sense_pos)
{
	int motor_pos;
	motor_pos = (-4.3*sense_pos) + 2822.4;
	return motor_pos;
}

int main(void){
	// Variable for flexThumb
	int flexthumb = 0;
	// variable for flexmiddle
	int flexmiddle = 0;
	// variable for flexindex
	int flexindex = 0;
	// variable to control PWM input for middle finger and index finger
	int middle_count, index_count;
	// Clear all interrupts
	cli();
	// Initialize the pins
	pin_init();
	// Initialize serial communication pins
	uart_init();
	// Initialize timer pins and interrupts
	timer_init();
	// Initialize PWM
	pwm_init();
	// Initialize ADC
	adc_init();
	// Initialize global interrupts
	sei();
	// Start conversion and wait for it to be done
	ADCSRA |= (1<<ADSC);
	// Filter initialization
	struct Ring_Buffer_F buffer_thumb;
	rb_initialize_F(&buffer_thumb);
	struct Ring_Buffer_F buffer_pointing;
	rb_initialize_F(&buffer_pointing);
	struct Ring_Buffer_F buffer_middle;
	rb_initialize_F(&buffer_middle);
	// Initialize output char buffer
	struct Ring_Buffer_C p_buf;
	rb_initialize_C(&p_buf);
	//main loop
	while (1)
	{
		if (TCNT1>100 && TCNT1<800)
		{
			if (TCNT1>=middle_count && bit_is_set(PORTB,PINB1))
			{
				PORTB &= ~(1<<PINB1);
			}
			if (TCNT1>=index_count && bit_is_set(PORTB,PINB2))
			{
				PORTB &= ~(1<<PINB2);
			}
		}
		
		if (flag_0)
		{
			// Check if ADC conversion is done
			if (!(ADCSRA & (1<<ADSC)))
			{
				uint16_t adcValue = adc_read();
				// switch the pin
				switch (ADMUX)
				{
					case 0x40:
						flexmiddle = adcValue;
						if (flexmiddle<405) flexmiddle = 405;
						else if (flexmiddle>500) flexmiddle = 500;
						middle_count = sensor_middle_to_motor(flexmiddle);
						ADMUX = 0x41;
						break;
					case 0x41:
						flexindex = adcValue;
						if (flexindex<505) flexindex = 505;
						else if (flexindex>620) flexindex = 620;
						index_count = sensor_index_to_motor(flexindex);
						ADMUX = 0x42;
						break;
					case 0x42:
						flexthumb = adcValue;
						ADMUX = 0x40;
						break;
				}
				// start the next conversion
				ADCSRA |= (1<<ADSC);
				
				/*uint8_t *numByte = (uint8_t *) &flexindex;
				rb_push_back_C(&p_buf, numByte[0]);
				rb_push_back_C(&p_buf, numByte[1]);
				rb_push_back_C(&p_buf, numByte[2]);
				rb_push_back_C(&p_buf, numByte[3]);*/
				//sets the servo position according to the scaled value, by writing data to servos
				//servo1.write(val1)
				//servo2.write(val2)
				//servo3.write(val3)  
			}
			// Reset flag for timer0
			flag_0 = 0;
		}
		  
		/*// Check if output buffer has byte and UART serial is free
		if ((rb_length_C(&p_buf)!=0) && (UCSR0A & (1<<UDRE0)))
		{
			// Load to UART buffer
			uart_transmit(rb_pop_front_C(&p_buf));
		}*/
	}
	return 0;
}
