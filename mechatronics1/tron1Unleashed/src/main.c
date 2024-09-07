//////////////////////////////// main.c ///////////////////////////////////////
// Filename:	main.c
// Description: Control code for Mechatronics 1 unleashed project and Warmann competition
// Author:		Lucien Morey
// Date:		2019-05-30
//////////////////////////////// main.c ///////////////////////////////////////

//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
// Header Files
//-----------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "LCDMap.h"
#include <stdlib.h>
#include <math.h>

//*****************************************************************************
// Private Macro Definitions
//-----------------------------------------------------------------------------
#define INPUT  0x00
#define OUTPUT 0x01
#define HIGH   0x01
#define LOW    0x00

#define USART_BAUDRATE 9600   
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW 1024

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

#define RECEIVERCHANNEL1 0
#define RECEIVERCHANNEL2 1
#define RECEIVERCHANNEL3 2
#define RECEIVERCHANNEL4 3
#define RECEIVERCHANNEL1FIRSTREAD 4
#define RECEIVERCHANNEL2FIRSTREAD 5
#define RECEIVERCHANNEL3FIRSTREAD 6
#define RECEIVERCHANNEL4FIRSTREAD 7

//*****************************************************************************
// Private Structure Type Definitions
//-----------------------------------------------------------------------------
struct V8FRii_receiver {
    uint16_t ch[8];
    };
volatile unsigned char ch_index;
struct V8FRii_receiver receiver;


//*****************************************************************************
// Private Global Variables
//-----------------------------------------------------------------------------

//SlaveAddresses slaveAddress = PCF8574;
//uint16_t flag = 0x0000;
volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

volatile uint8_t pwmRead = 0b11111111;
volatile unsigned long timer0;
volatile unsigned long timer1;
volatile unsigned long timer2;
volatile unsigned long timer3;
volatile unsigned long highcounts0;
volatile unsigned long highcounts1;
volatile unsigned long highcounts2;
volatile unsigned long highcounts3;
volatile float dutyCycle0;
volatile float dutyCycle1;
volatile float dutyCycle2;
volatile float dutyCycle3;

double vx;
double vy;
double vz;
int vmotor0_sign;
int vmotor1_sign;
int vmotor2_sign;
int vmotor3_sign;


int vmotor[3];

char str[100];

volatile float vmotor0_velocity;
volatile float vmotor1_velocity;
volatile float vmotor2_velocity;
volatile float vmotor3_velocity;

volatile unsigned long vmotor0_velocityTimer = 0;
volatile unsigned long vmotor1_velocityTimer = 0;
volatile unsigned long vmotor2_velocityTimer = 0;
volatile unsigned long vmotor3_velocityTimer = 0;


//*****************************************************************************
// Private Function Definitions
//-----------------------------------------------------------------------------

//******************************************************************************
// USART FUNCTIONS
//******************************************************************************

void USART0_Init(void){

    
   // Set baud rate
   UBRR0L = BAUD_PRESCALE;// Load lower 8-bits into the low byte of the UBRR register
   UBRR0H = (BAUD_PRESCALE >> 8); 
	 /* Load upper 8-bits into the high byte of the UBRR register
    Default frame format is 8 data bits, no parity, 1 stop bit
  to change use UCSRC, see AVR datasheet*/ 

  // Enable receiver and transmitter and receive complete interrupt 
  UCSR0B = ((1<<TXEN0)|(1<<RXEN0) | (1<<RXCIE0));
}

void USART0_SendByte(uint8_t u8Data){

  // Wait until last byte has been transmitted
  while((UCSR0A &(1<<UDRE0)) == 0);

  // Transmit data
  UDR0 = u8Data;
}

void sendString(const char *str, size_t n) {
    for (int i = 0; i < n && str[i]; i++) { 
        USART0_SendByte(str[i]); //send full string and stop before reaching end of array string to avoid strage behaviour
    }
}



//******************************************************************************
// AGNOSTIC PIN FUNCTIONS
//******************************************************************************

void pinMode(volatile uint8_t* pin, uint8_t position, uint8_t direction){
    if (direction == INPUT){
        *pin &= ~(1<<position);

    } else if(direction == OUTPUT){
        *pin |= (1<<position);

    }
}

int readPin(volatile uint8_t* pin, uint8_t position){
    uint8_t output;

    output = (~(*pin) & 1<<position);
    return output;
}

void writePin(volatile uint8_t* pin, uint8_t position,uint8_t value){
    if(value == HIGH){
        *pin |= (1<<position);
        
    }else if(value==LOW){
        *pin &= ~(1<<position);
        
    }
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int constrain(float x, int lowerbound, int upperbound){
    if (x > upperbound){
        x = upperbound;
    } else if(x < lowerbound){
        x = lowerbound;
    }
    return x;
}

int sign(int x){
    if (x < 0){
        return 0;
    } else{
        return 1;
    }
}

//******************************************************************************
// MILLIS AND MICROS FUNCTIONS
//******************************************************************************


void timer0Init(){
    TCCR0A = 0;
    TCCR0B = (1<<CS01)|(1<<CS00); ///start timer with 1/64 prescaler
    TIMSK0 = (1<<TOIE0); //enable timer0 overflow interrupt

    TCNT0 = 0x00; //clear timer
    
}

ISR(TIMER0_OVF_vect){
	
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC; //increment past millis by precaluclated overflow value
	f += FRACT_INC;  //increment past micros component of timer by precaluclated overflow value
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX; //If micro component is greater than 1 milli then increment millis again
		m += 1;
	}

	timer0_fract = f; //store micros
	timer0_millis = m; //store millis
	timer0_overflow_count++; //keep track of overflows for micross calculation
}

unsigned long millis(){
	unsigned long m;
	uint8_t oldSREG = SREG;
	cli(); //disable interrupts to ensure correct millis return is used
	m = timer0_millis;
	SREG = oldSREG; //enable interrupts again

	return m;
}

unsigned long micros(){
	unsigned long m;
	uint8_t oldSREG = SREG, t;
	
	cli();
	m = timer0_overflow_count; 
    
	t = TCNT0;

	if((TIFR0 & _BV(TOV0)) && (t < 255)){
		m++;
    }

	SREG = oldSREG;
	
	return ((m << 8) + t) * (64 / 16);
}

//****************************************************************************** 
//CONTROLLER IMPLEMENTATION
//******************************************************************************

// External interrupts will record timing of rising and falling edges 
// of pwm signal to calculate duty cycle.


void receiverInit(void){
    pinMode(&PORTD,PORTD2,INPUT); //Pin 19
    pinMode(&PORTD,PORTD3,INPUT); //Pin 18
    pinMode(&PORTE,PORTE4,INPUT); //Pin 2
    pinMode(&PORTE,PORTE5,INPUT); //Pin 3
    EICRA |= (1<<ISC20)|(1<<ISC21)|(1<<ISC30)|(1<<ISC31); //Setting External interrupts on pins 19 and 18 to look for a rising edge
    EICRB |= (1<<ISC40)|(1<<ISC41)|(1<<ISC50)|(1<<ISC51); //Setting External interrupts on pins 3 and 2 to look for a rising edge
    EIMSK |= (1<<INT2)|(1<<INT3)|(1<<INT4)|(1<<INT5); //Enabling External interrupts on pins 2,3,18,19
}

ISR (INT2_vect){
    if(pwmRead & (1<<RECEIVERCHANNEL1) ){
            if (pwmRead & (1<<RECEIVERCHANNEL1FIRSTREAD)){
                //set start time of next signal
                timer0 = micros();
                
                //toggle to look for falling edge
                EICRA ^= (1<<ISC20);
                pwmRead ^= (1<<RECEIVERCHANNEL1);
                pwmRead ^= (1<<RECEIVERCHANNEL1FIRSTREAD);

            } else {
                
                unsigned long totalCounts;
                //read total counts
                totalCounts = micros() - timer0;
                
                //set start time of next signal
                timer0 = micros();
                 
                //toggle to look for falling edge
                EICRA ^= (1<<ISC20);                
                pwmRead ^= (1<<RECEIVERCHANNEL1);
                
                //duty cycle calculation
                dutyCycle0 = ((float)highcounts0/(float)totalCounts)*100.0;     
                    

            }
    }else if ((~(pwmRead) & (1<<RECEIVERCHANNEL1)) && (~(pwmRead) & (1<<RECEIVERCHANNEL1FIRSTREAD)) ){
        //record high cycles
        highcounts0 = micros() - timer0;
        //toggle to look for rising edge
        EICRA ^= (1<<ISC20);        
        pwmRead ^= (1<<RECEIVERCHANNEL1);

    }
    EIFR |= (1<<INTF2);
}

ISR (INT3_vect){
    if(pwmRead & (1<<RECEIVERCHANNEL2) ){
            if (pwmRead & (1<<RECEIVERCHANNEL2FIRSTREAD)){
                //set start time of next signal
                timer1 = micros();
                
                //toggle to look for falling edge
                EICRA ^= (1<<ISC30);
                pwmRead ^= (1<<RECEIVERCHANNEL2);
                pwmRead ^= (1<<RECEIVERCHANNEL2FIRSTREAD);

            } else {
                
                unsigned long totalCounts;
                //read total counts
                totalCounts = micros() - timer1;
                
                //set start time of next signal
                timer1 = micros();
                 
                //toggle to look for falling edge
                EICRA ^= (1<<ISC30);                
                pwmRead ^= (1<<RECEIVERCHANNEL2);
                
                //duty cycle calculation
                dutyCycle1 = ((float)highcounts1/(float)totalCounts)*100.0;     
                    

            }
    }else if ((~(pwmRead) & (1<<RECEIVERCHANNEL2)) && (~(pwmRead) & (1<<RECEIVERCHANNEL2FIRSTREAD)) ){
        //record high cycles
        highcounts1 = micros() - timer1;
        //toggle to look for rising edge
        EICRA ^= (1<<ISC30);        
        pwmRead ^= (1<<RECEIVERCHANNEL2);

    }
    EIFR |= (1<<INTF3);
}

ISR (INT4_vect){
    if(pwmRead & (1<<RECEIVERCHANNEL3) ){
            if (pwmRead & (1<<RECEIVERCHANNEL3FIRSTREAD)){
                //set start time of next signal
                timer2 = micros();
                
                //toggle to look for falling edge
                EICRB ^= (1<<ISC40);
                pwmRead ^= (1<<RECEIVERCHANNEL3);
                pwmRead ^= (1<<RECEIVERCHANNEL3FIRSTREAD);

            } else {
                
                unsigned long totalCounts;
                //read total counts
                totalCounts = micros() - timer2;
                
                //set start time of next signal
                timer2 = micros();
                 
                //toggle to look for falling edge
                EICRB ^= (1<<ISC40);                
                pwmRead ^= (1<<RECEIVERCHANNEL3);
                
                //duty cycle calculation
                dutyCycle2 = ((float)highcounts2/(float)totalCounts)*100.0;     
                    

            }
    }else if ((~(pwmRead) & (1<<RECEIVERCHANNEL3)) && (~(pwmRead) & (1<<RECEIVERCHANNEL3FIRSTREAD)) ){
        //record high cycles
        highcounts2 = micros() - timer2;
        //toggle to look for rising edge
        EICRB ^= (1<<ISC40);        
        pwmRead ^= (1<<RECEIVERCHANNEL3);

    }
    EIFR |= (1<<INTF4);
}

ISR (INT5_vect){
    if(pwmRead & (1<<RECEIVERCHANNEL4) ){
            if (pwmRead & (1<<RECEIVERCHANNEL4FIRSTREAD)){
                //set start time of next signal
                timer3 = micros();
                
                //toggle to look for falling edge
                EICRB ^= (1<<ISC50);
                pwmRead ^= (1<<RECEIVERCHANNEL4);
                pwmRead ^= (1<<RECEIVERCHANNEL4FIRSTREAD);

            } else {
                
                unsigned long totalCounts;
                //read total counts
                totalCounts = micros() - timer3;
                
                //set start time of next signal
                timer3 = micros();
                 
                //toggle to look for falling edge
                EICRB ^= (1<<ISC50);                
                pwmRead ^= (1<<RECEIVERCHANNEL4);
                
                //duty cycle calculation
                dutyCycle3 = ((float)highcounts3/(float)totalCounts)*100.0;     
                    

            }
    }else if ((~(pwmRead) & (1<<RECEIVERCHANNEL4)) && (~(pwmRead) & (1<<RECEIVERCHANNEL4FIRSTREAD)) ){
        //record high cycles
        highcounts3 = micros() - timer3;
        //toggle to look for rising edge
        EICRB ^= (1<<ISC50);        
        pwmRead ^= (1<<RECEIVERCHANNEL4);

    }
    EIFR |= (1<<INTF5);
}

//******************************************************************************
// PWM Write FUNCTIONS
//******************************************************************************

void pwmTimerInit(){
    pinMode(&DDRL, DDL5, OUTPUT);
    pinMode(&DDRL, DDL4, OUTPUT);
    pinMode(&DDRL, DDL3, OUTPUT);
    pinMode(&DDRB, DDB5, OUTPUT);

    OCR5A = 0; //Initiialise outputcompare register to 0 until main loop sets desired value
    OCR5B = 0;
    OCR5C = 0;

    TCCR5A = 0; //zero timer control registers before setting up pwm timer
    TCCR5B = 0;
    TCCR5C = 0;
    //set 8 bit phase correct pwm modeset all channels to set on down clear on up
    TCCR5A = (1<<WGM50)|(1<<COM5C1)|(1<<COM5B1)|(1<<COM5A1);

    //Set timer with prescaler of 8 
    TCCR5B = (1<<CS51);//|(1<<CS50);

    OCR1A = 0; //Initiialise outputcompare register to 0 until main loop sets desired value
    OCR1B = 0;
    OCR1C = 0;

    TCCR1A = 0; //zero timer control registers before setting up pwm timer
    TCCR1B = 0;
    TCCR1C = 0;
    //set 8 bit phase correct pwm modeset all channels to set on down clear on up
    TCCR1A = (1<<WGM10)|(1<<COM1C1)|(1<<COM1B1)|(1<<COM1A1);

    //Set timer with prescaler of 8 
    TCCR1B = (1<<CS11);//|(1<<CS50);

}

//******************************************************************************
// Encoder Check FUNCTIONS
//******************************************************************************

void encoderInit(void){
    pinMode(&PORTD,PORTD0,INPUT); //Pin 19
    pinMode(&PORTD,PORTD1,INPUT); //Pin 18
    EICRA |= (1<<ISC00)|(1<<ISC01)|(1<<ISC10)|(1<<ISC11); //Setting External interrupts on pins 21 and 20 to look for a rising edge
    EIMSK |= (1<<INT0)|(1<<INT1); //Enabling External interrupts on pins 21,20
}

//Pseudo code for speed calculation

/*
Pin interrupt
    if first interrupt
        record timestamp
    else
        previous timestamp = timestamp
        record current timestamp
        speed = Encoder distance/(current timestamp - previous timestamp)
end interrupt 

pin change interrupt will also have to check if current pin state is high or not 
to function correctly
*/




//=============================================================================
// main Function Definition
//-----------------------------------------------------------------------------
int main(void){
    
    //Initialise controller receiver
    receiverInit();

    //Initialise timer0 for millis and micros functions
    timer0Init();

    //Initialise timer5 with PWM output
    pwmTimerInit();

    USART0_Init();
    
    //enable interrupts
    sei();


    
    pinMode(&DDRG,DDG2,OUTPUT);
    pinMode(&DDRG,DDG1,OUTPUT);
    pinMode(&DDRG,DDG0,OUTPUT);
    pinMode(&DDRD,DDD7,OUTPUT);
    pinMode(&DDRH,DDH5,OUTPUT);
    pinMode(&DDRB,DDB2,OUTPUT);
    pinMode(&DDRB,DDB3,OUTPUT);
    pinMode(&DDRB,DDB1,OUTPUT);
    pinMode(&DDRB,DDB0,OUTPUT);
    pinMode(&DDRC,DDC7,OUTPUT);
    pinMode(&DDRC,DDC6,OUTPUT);
    pinMode(&DDRC,DDC5,OUTPUT);
    pinMode(&DDRC,DDC4,OUTPUT);

    

    
    

    while (1){
        
        USART0_SendByte('\f');
        //map duty cycle input to percentage output of motors
        vx = map(dutyCycle0,5.6,10.4,-100.0,100.0);
        vy = map(dutyCycle1,5.7,10.3,-100.0,100.0);
        vz = map(dutyCycle2,5.7,10.3,-100.0,100.0);

        //scale down vz 
        //vz = vz*0.5;

        //invert vy so it is correct with co-ordinate system
        vy = vy*(-1);
        
        //apply deadzones

        if ((vx < 40)&&(vx > -40)){
            vx=0.0;
        }

        if ((vy < 40)&&(vy > -40)){
            vy=0.0;
        }

        if ((vz < 40)&&(vz > -40)){
            vz=0.0;
        }

        
        

        int tmpInt1 = vx;                  // Get the integer component.
        double tmpFrac = vx - tmpInt1;      // Get decimal component.
        int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer component.

        // Print as parts, note that decimal component will be incorrect because
        // I wasnt sure how to acheive the correct 0 padding every time

        sprintf (str, " vx = %d.%04d", tmpInt1, tmpInt2); //use sprintf to turn string into array of characters 

        sendString(str, 32);

        USART0_SendByte(13); //carriage return command
        USART0_SendByte('\n'); //new line command

        tmpInt1 = vy;                  // Get the integer (678).
        tmpFrac = vy - tmpInt1;      // Get fraction (0.0123).
        tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

        // Print as parts, note that you need 0-padding for fractional bit.

        sprintf (str, "vy = %d.%04d", tmpInt1, tmpInt2);

        sendString(str, 32);

        USART0_SendByte(13);
        USART0_SendByte('\n');

        tmpInt1 = vz;                  // Get the integer (678).
        tmpFrac = vz - tmpInt1;      // Get fraction (0.0123).
        tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

        // Print as parts, note that you need 0-padding for fractional bit.

        sprintf (str, "vz = %d.%04d", tmpInt1, tmpInt2);

        sendString(str, 32);

        USART0_SendByte(13);
        USART0_SendByte('\n');

        
        //determine output at each motor by solving system of equations for forward motion 
        vmotor[0] = vx - vz;
        vmotor[1] = vy - vz;
        vmotor[2] = -vx - vz;
        vmotor[3] = -vy - vz;

        tmpInt1 = vmotor[0];                  // Get the integer (678).
        tmpFrac = vmotor[0] - tmpInt1;      // Get fraction (0.0123).
        tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

        // Print as parts, note that you need 0-padding for fractional bit.

        sprintf (str, "vmotor[0] = %d.%04d", tmpInt1, tmpInt2);

        sendString(str, 32);

        USART0_SendByte(13);
        USART0_SendByte('\n');

        tmpInt1 = vmotor[1];                  // Get the integer (678).
        tmpFrac = vmotor[1] - tmpInt1;      // Get fraction (0.0123).
        tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

        // Print as parts, note that you need 0-padding for fractional bit.

        sprintf (str, "vmotor[1] = %d.%04d", tmpInt1, tmpInt2);

        sendString(str, 32);

        USART0_SendByte(13);
        USART0_SendByte('\n');

        tmpInt1 = vmotor[2];                  // Get the integer (678).
        tmpFrac = vmotor[2] - tmpInt1;      // Get fraction (0.0123).
        tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

        // Print as parts, note that you need 0-padding for fractional bit.

        sprintf (str, "vmotor[2] = %d.%04d", tmpInt1, tmpInt2);

        sendString(str, 32);

        USART0_SendByte(13);
        USART0_SendByte('\n');

        tmpInt1 = vmotor[3];                  // Get the integer (678).
        tmpFrac = vmotor[3] - tmpInt1;      // Get fraction (0.0123).
        tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

        // Print as parts, note that you need 0-padding for fractional bit.

        sprintf (str, "vmotor[3] = %d.%04d", tmpInt1, tmpInt2);

        sendString(str, 32);

        USART0_SendByte(13);
        USART0_SendByte('\n');

        
        // write motor directions
        // motor controller has two pins for motor output to determine motor direction.
        // At any given time one must by high and the other must be low so to change direction
        // the state of these pins must be inverted
        if(vmotor[1] < 0){
            writePin(&PORTB,PORTB2,HIGH); //PIN51
            writePin(&PORTB,PORTB3,LOW); //PIN50
        } else{
            writePin(&PORTB,PORTB2,LOW); //PIN51
            writePin(&PORTB,PORTB3,HIGH); //PIN50
        }

        if(vmotor[0] > 0){
            writePin(&PORTB,PORTB1,HIGH); //PIN52
            writePin(&PORTB,PORTB0,LOW); //PIN53
        } else{
            writePin(&PORTB,PORTB1,LOW); //PIN52
            writePin(&PORTB,PORTB0,HIGH); //PIN53
        }

        if(vmotor[3] > 0){
            writePin(&PORTC,PORTC7,HIGH);//Pin 30
            writePin(&PORTC,PORTC6,LOW);//Pin 31
        } else{
            writePin(&PORTC,PORTC7,LOW);//Pin 30
            writePin(&PORTC,PORTC6,HIGH);//Pin 31
        }

        if(vmotor[2] > 0){
            writePin(&PORTC,PORTC5,HIGH);//Pin 32
            writePin(&PORTC,PORTC4,LOW);//Pin 33
        } else{
            writePin(&PORTC,PORTC5,LOW);//Pin 32
            writePin(&PORTC,PORTC4,HIGH);//Pin 33
        }
        
        


        OCR5A = (uint8_t)120*((double)abs(vmotor[0])/100.0); //PIN46
        OCR5B = (uint8_t)120*((double)abs(vmotor[1])/100.0); //PIN45
        OCR5C = (uint8_t)120*((double)abs(vmotor[2])/100.0); //PIN44
        OCR1A = (uint8_t)120*((double)abs(vmotor[3])/100.0); //PIN11

        tmpInt1 = OCR5A;                  // Get the integer (678).
        tmpFrac = OCR5A - tmpInt1;      // Get fraction (0.0123).
        tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

        // Print as parts, note that you need 0-padding for fractional bit.

        sprintf (str, "OCR5A = %d.%04d", tmpInt1, tmpInt2);

        sendString(str, 32);

        USART0_SendByte(13);
        USART0_SendByte('\n');

        tmpInt1 = OCR5B;                  // Get the integer (678).
        tmpFrac = OCR5B - tmpInt1;      // Get fraction (0.0123).
        tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

        // Print as parts, note that you need 0-padding for fractional bit.

        sprintf (str, "OCR5B = %d.%04d", tmpInt1, tmpInt2);

        sendString(str, 32);

        USART0_SendByte(13);
        USART0_SendByte('\n');

        tmpInt1 = OCR5C;                  // Get the integer (678).
        tmpFrac = OCR5C - tmpInt1;      // Get fraction (0.0123).
        tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

        // Print as parts, note that you need 0-padding for fractional bit.

        sprintf (str, "OCR5C = %d.%04d", tmpInt1, tmpInt2);

        sendString(str, 32);

        USART0_SendByte(13);
        USART0_SendByte('\n');

        
        
        _delay_ms(200);

        
        


    }


    return 0;

}


// main.c EOF