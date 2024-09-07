#include <avr/io.h>
#include <avr/interrupt.h>

#include "LiquidCrystal.h"
#include "StepperMotor.h"
#include "jlmio.h"
#include <math.h>
enum systemState{
  ClockDisplay,
  StudentID,
  DistanceSensor,
  Continuous,
  Step,
  Link,
};
enum buttonPress{
  NOTHING = 1023,
  SELECT = 642,
  LEFT = 410,
  UP = 99,
  DOWN = 255,
  RIGHT = 0
};

systemState currentState;

buttonPress pressedButton = NOTHING;

volatile float timeSinceLastStep = 0;
unsigned long currentTime;
int currentSeconds;
unsigned long currentMins;
uint16_t distance;
double procDistance;
uint8_t rpm =1;
char data[18];
int tempButtonState;
unsigned long timeSinceLastButton=0;
unsigned long delayStartTime;
int oldButtonState;
bool firstLink= true;
int samples=0;
int tempDistance;

LiquidCrystal lcd(8,9,4,5,6,7);

StepperMotor stepper(2038,&DDRB,&PORTB,PORTB5,&DDRB,&PORTB,PORTB4,&DDRB,&PORTB,PORTB3,&DDRD,&PORTD,PORTB2);

void timer1Init();

int main(){   
  stepper.setDirection(1);
  
  //start LCD with 16 columns and 2 rows
  lcd.begin(16,2);
  //initisilise timer 0 as a millis timer
  jlm::timer2Init();
  jlm::adc_setup();
  sei();
  timer1Init();
  
  while(1){
    
    currentTime = jlm::GetMillis();
    
    if((currentTime-timeSinceLastButton) > 200){
      tempButtonState = jlm::analog_read(0);
      timeSinceLastButton = jlm::GetMillis();
    } else {
      tempButtonState = 5000;
    }  
    
    if (samples == 0){
      tempDistance = 0;
    }
    if (samples < 5){
      tempDistance = tempDistance + jlm::analog_read(1);
      samples++;
    } else if(samples == 5){
      distance = tempDistance/5;
      samples = 0; 
    }

    procDistance = 8835.4*pow(distance,-0.944);
    jlm::Constrain(procDistance,15,150);

    lcd.clear();
    switch (currentState){
    case ClockDisplay:
      //convert current time to seconds
      currentTime = currentTime/1000;
      currentMins = currentTime / 60;
      currentSeconds = currentTime % 60;
      if ((currentMins<10)&&(currentSeconds<10)){
        sprintf(data,"0%lu:0%d", currentMins, currentSeconds);
      } else if (currentMins<10){
        sprintf(data,"0%lu:%d", currentMins, currentSeconds);
      } else {
        sprintf(data,"%lu:%d", currentMins, currentSeconds);
      }
      
      lcd.print("Clock Mode");
      lcd.setCursor(0,1);
      lcd.print(data);

      if ((tempButtonState > (buttonPress::SELECT - 5)) && (tempButtonState < (buttonPress::SELECT + 5))){
        currentState = StudentID;
      }
      break;
    case StudentID:
      lcd.print("Lucien M");
      lcd.setCursor(0,1);
      lcd.print("12904090");
      
      if ((tempButtonState > (buttonPress::SELECT - 5)) && (tempButtonState < (buttonPress::SELECT + 5))){
        currentState = DistanceSensor;
      }
    break;

    case DistanceSensor:
      sprintf(data,"distance=%dcm",(int)procDistance);
      lcd.print("Distance mode");
      lcd.setCursor(0,1);
      lcd.print(data);

      if ((tempButtonState > (buttonPress::SELECT - 5)) && (tempButtonState < (buttonPress::SELECT + 5))){
        currentState = Continuous;
      }

    break;

    case Continuous:
      stepper.stepping = true;

      if ((tempButtonState > (buttonPress::SELECT - 5)) && (tempButtonState < (buttonPress::SELECT + 5))){
        currentState = Step;
      } else if ((tempButtonState > (buttonPress::UP - 5)) && (tempButtonState < (buttonPress::UP + 5))){
        rpm++;
        if (rpm > 7){
          rpm = 7;
        }
      } else if ((tempButtonState > (buttonPress::DOWN - 5)) && (tempButtonState < (buttonPress::DOWN + 5))){
        rpm--;
        if (rpm < 1){
          rpm = 1;
        }
      } else if ((tempButtonState > (buttonPress::RIGHT - 5)) && (tempButtonState < (buttonPress::RIGHT + 5))){
        stepper.setDirection(-1);
      } else if ((tempButtonState > (buttonPress::LEFT - 5)) && (tempButtonState < (buttonPress::LEFT + 5))){
        stepper.setDirection(1);
      }
      lcd.print("Continuous mode");
      lcd.setCursor(0,1);
      
      if(stepper.getDirection() == -1){
        sprintf(data,"%drpm clkwise",rpm); 
      } else if (stepper.getDirection() == 1){
        sprintf(data,"%drpm anticlkwise",rpm); 
      }
      
      lcd.print(data);   
    break;

    case Step:
      if ((tempButtonState > (buttonPress::SELECT - 5)) && (tempButtonState < (buttonPress::SELECT + 5))){
        currentState = Link;
      } else if ((tempButtonState > (buttonPress::UP - 5)) && (tempButtonState < (buttonPress::UP + 5))){
        stepper.steps = 0;
      } else if ((tempButtonState > (buttonPress::DOWN - 5)) && (tempButtonState < (buttonPress::DOWN + 5))){
        stepper.stepping = true;
      } else if ((tempButtonState > (buttonPress::RIGHT - 5)) && (tempButtonState < (buttonPress::RIGHT + 5))){
        stepper.steps++;
      } else if ((tempButtonState > (buttonPress::LEFT - 5)) && (tempButtonState < (buttonPress::LEFT + 5))){
        stepper.steps--;
      }
      
      
      lcd.print("requested steps:");
      lcd.setCursor(0,1);
      lcd.print(stepper.steps);

      if (stepper.stepping){
        while(stepper.steps > 0){
          if ((tempButtonState > (buttonPress::UP - 5)) && (tempButtonState < (buttonPress::UP + 5))){
            stepper.steps = 0;
          }
          lcd.clear();
          lcd.print("remaining steps");
          lcd.setCursor(0,1);
          lcd.print(stepper.steps);
          delayStartTime = jlm::GetMillis();
          currentTime = jlm::GetMillis(); 
          while((currentTime-delayStartTime)<100UL){
            currentTime=jlm::GetMillis();

          }

        }
        stepper.stepping = false;
      }
    break;

    case Link:
      if(firstLink == true){
        firstLink = false;
        stepper.stepsTaken = abs(stepper.stepsTaken);
        while(stepper.stepsTaken > stepper.stepsPerRev){
          stepper.stepsTaken = stepper.stepsTaken - stepper.stepsPerRev;
        }

      }
      lcd.print("Link Mode");
      lcd.setCursor(0,1);      
      stepper.steps = (jlm::Map(procDistance,15,150,0,2038)-stepper.stepsTaken);
      lcd.print(abs(stepper.steps));
      stepper.setDirection(jlm::Sign(stepper.steps));
      
      if (stepper.steps!=0){
        stepper.stepping = true;
      }

      if ((tempButtonState > (buttonPress::SELECT - 5)) && (tempButtonState < (buttonPress::SELECT + 5))){
        currentState = ClockDisplay;
        firstLink=true;
        stepper.steps = 0;
      }
    
    break;
    }
    
    stepper.setSpeed(rpm);
    delayStartTime = jlm::GetMillis();
    currentTime = jlm::GetMillis(); 
    while((currentTime-delayStartTime)<200UL){
      currentTime=jlm::GetMillis();
    }
    stepper.stepping = false;
  }

  return 0;
}

void timer1Init(){
  TCNT1 = 55536;
  TCCR1A = 0;
  TCCR1B = (1<<CS10); ///start timer1 with no prescaler
  TIMSK1 = (1<<TOIE1); //enable timer1 overflow interrupt
}

ISR(TIMER1_OVF_vect){
  timeSinceLastStep = timeSinceLastStep +  625;
  if ((timeSinceLastStep > stepper.stepPeriod)&&(stepper.stepping)){
    timeSinceLastStep = 0;
    stepper.step();
    stepper.steps --;
    if (stepper.steps < 0){
      stepper.steps =0;
    }
  }

  TCNT1 = 55536;
}


