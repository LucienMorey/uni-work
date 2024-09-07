#include "StepperMotor.h"
#include "avr/interrupt.h"
#include <Arduino.h>

StepperMotor::StepperMotor(int stepsPerRev, volatile uint8_t* dataDirection1, volatile uint8_t* pin_1, uint8_t position_1, volatile uint8_t* dataDirection2, 
        volatile uint8_t* pin_2, uint8_t position_2, volatile uint8_t* dataDirection3, volatile uint8_t* pin_3, uint8_t position_3, 
        volatile uint8_t* dataDirection4, volatile uint8_t* pin_4, uint8_t position_4){
    
    this->stepsPerRev = stepsPerRev;
    
    
    (pin1) = pin_1;
    (position1) = position_1;
    
    (pin2) = pin_2;
    (position2) = position_2;
    
    (pin3) = pin_3;
    (position3) = position_3;
    
    (pin4) = pin_4;
    (position4) = position_4;

    jlm::PinMode(dataDirection1,this->position1,OUTPUT);
    jlm::PinMode(dataDirection2,this->position2,OUTPUT);
    jlm::PinMode(dataDirection3,this->position3,OUTPUT);
    jlm::PinMode(dataDirection4,this->position4,OUTPUT);

    jlm::WritePin(this->pin1,this->position1,LOW);
    jlm::WritePin(this->pin2,this->position2,LOW);
    jlm::WritePin(this->pin3,this->position3,LOW);
    jlm::WritePin(this->pin4,this->position4,LOW);

    stepState = 1;
    stepsTaken = 0; 


}

StepperMotor::~StepperMotor(){

}

void StepperMotor::step(){
    switch (stepState){
    case 1:
        jlm::WritePin(pin1,position1,HIGH);
        jlm::WritePin(pin2,position2,LOW);
        jlm::WritePin(pin3,position3,HIGH);
        jlm::WritePin(pin4,position4,LOW);
        break;
    
    case 2:
        jlm::WritePin(pin1,position1,LOW);
        jlm::WritePin(pin2,position2,HIGH);
        jlm::WritePin(pin3,position3,HIGH);
        jlm::WritePin(pin4,position4,LOW);
        break;
    
    case 3:
        jlm::WritePin(pin1,position1,LOW);
        jlm::WritePin(pin2,position2,HIGH);
        jlm::WritePin(pin3,position3,LOW);
        jlm::WritePin(pin4,position4,HIGH);
        
        break;
    
    case 4:
        jlm::WritePin(pin1,position1,HIGH);
        jlm::WritePin(pin2,position2,LOW);
        jlm::WritePin(pin3,position3,LOW);
        jlm::WritePin(pin4,position4,HIGH);
        break;
    }
    if (direction == 1){
        stepState ++;
        stepsTaken++;
    } else if (direction == -1){
        stepState --;
        stepsTaken--;
    }

    if(stepState < 1){
        stepState = 4;
    } else if (stepState > 4){
        stepState = 1;
    }
}

void StepperMotor::setSpeed(int rpm){
    this->rpm = rpm;
    stepPeriod =((60000000.0f/(this->rpm*stepsPerRev)));
}



void StepperMotor::setDirection(int directionToSet){
    if (directionToSet == 1){
        direction = 1;
    } else if (directionToSet == -1){
        direction = -1;
    }

}

int StepperMotor::getDirection(){
    return direction;
}