#ifndef JLMIO_H
#define JLMIO_H


#include <stdio.h>

#define INPUT  0x0
#define OUTPUT 0x1
#define HIGH   0x1
#define LOW    0x0

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER2_OVERFLOW 1024

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER2_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER2_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

//extern volatile unsigned long timer0_overflow_count;
//extern volatile unsigned long timer0_millis;

class jlm{
    public:
        
        

        static void timer2Init();

        static unsigned long GetMillis();

        static unsigned long GetMicros();

        // analog read from specified analog analog pin
        static unsigned int analog_read(unsigned char analog_pin);

        // setup adc to use Vcc +5v internal source
        static void adc_setup();

        //Set pinmode for digital pins
        static void PinMode(volatile uint8_t* pin, uint8_t position, uint8_t direction);

        //read digital pin values
        static int ReadPin(volatile uint8_t* pin, uint8_t position);

        //read write value to digital pin
        static void WritePin(volatile uint8_t* pin, uint8_t position,uint8_t value);

        //map values between a range
        static float Map(float x, float in_min, float in_max, float out_min, float out_max);

        // restrict values between a range of values
        static int Constrain(float x, int lowerbound, int upperbound);

        //check sign of value
        static int Sign(int x);

};

#endif