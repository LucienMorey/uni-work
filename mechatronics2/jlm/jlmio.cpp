#include "jlmio.h"
#include <avr/io.h>
#include <avr/interrupt.h>

volatile unsigned long timer2_overflow_count = 0;
volatile unsigned long timer2_millis = 0;
static unsigned char timer2_fract = 0;



ISR(TIMER2_OVF_vect){
	
	

	timer2_millis += MILLIS_INC; //increment past millis by precaluclated overflow value
	timer2_fract += FRACT_INC;  //increment past micros component of timer by precaluclated overflow value
	if (timer2_fract >= FRACT_MAX) {
		timer2_fract -= FRACT_MAX; //If micro component is greater than 1 milli then increment millis again
		timer2_millis += 1;
	}

	timer2_overflow_count++; //keep track of overflows for micross calculation
}

void jlm::timer2Init(){
    TCCR2A = 0;
    TCCR2B = (1<<CS22); ///start timer with 1/64 prescaler
    TIMSK2 = (1<<TOIE2); //enable timer2 overflow interrupt

    TCNT2 = 0x00; //clear timer
    
}

unsigned long jlm::GetMillis(){
	unsigned long m;
	uint8_t oldSREG = SREG;
	cli(); //disable interrupts to ensure correct millis return is used
	m = timer2_millis;
	SREG = oldSREG; //enable interrupts again

	return m;
}

unsigned long jlm::GetMicros(){
	unsigned long m;
	uint8_t oldSREG = SREG, t;
	
	cli();
	m = timer2_overflow_count; 
    
	t = TCNT2;

	if((TIFR2 & _BV(TOV2)) && (t < 255)){
		m++;
    }

	SREG = oldSREG;
	
	return ((m << 8) + t) * (64 / 16);
}

// analog read from specified analog analog pin
unsigned int jlm::analog_read(unsigned char analog_pin) {
    ADMUX &= 0b11110000;        // clear ADC mux bits
    ADMUX |= analog_pin;        // select mux channel
    ADCSRA |= (1<<ADSC);        // start conversion
    //_delay_ms(10);
    while((ADCSRA & 0b00010000) == 0){
        // wait untill conversion finishes
        
    } 

    return (ADCL) | (ADCH << 8);
}

// initialise the adc module
void jlm::adc_setup() {
    ADMUX = 0b01000000;         // set the voltage reference to Vcc +5v
    ADCSRA = 0b10000111;        // turn on ADC, disable auto trigger, disable interrupts, set prescale
}

//Set pinmode for digital pins
void jlm::PinMode(volatile uint8_t* pin, uint8_t position, uint8_t direction){
    if (direction == INPUT){
        *pin &= ~(1<<position);

    } else if(direction == OUTPUT){
        *pin |= (1<<position);

    }
}

//read digital pin values
int jlm::ReadPin(volatile uint8_t* pin, uint8_t position){
    uint8_t output;

    output = (~(*pin) & 1<<position);
    return output;
}

//read write value to digital pin
void jlm::WritePin(volatile uint8_t* pin, uint8_t position,uint8_t value){
    if(value == HIGH){
        *pin |= (1<<position);
        
    }else if(value==LOW){
        *pin &= ~(1<<position);
        
    }
}

//map values between a range
float jlm::Map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// restrict values between a range of values
int jlm::Constrain(float x, int lowerbound, int upperbound){
    if (x > upperbound){
        x = upperbound;
    } else if(x < lowerbound){
        x = lowerbound;
    }
    return x;
}

//check sign of value
int jlm::Sign(int x){
    if (x < 0){
        return -1;
    } else{
        return 1;
    }
}

