#include "car.h"
#include <cmath>
#include <iostream>

Car::Car(std::string make, std::string model,double height, double width,
         double horsePower, double dragCoefficient, double weight) :
  make_(make),model_(model),dragCoefficient_(dragCoefficient),
  currentSpeed_(0.0), weight_(weight)
{
  area_=width*height;
  power_= power_conversion * horsePower;
  auto tp = std::chrono::system_clock::now();
  time_ = tp.time_since_epoch().count()*0.000000001;
}

std::string Car::getMake(void) {
  return make_;
}

std::string Car::getModel(void) {
  return model_;
}

double Car::getCurrentSpeed(void) {
  return currentSpeed_;
}

void Car::setMake(std::string make) {
  make_ = make;
}

void Car::setModel(std::string model) {
  model_ = model;
}

double Car::calculateTopSpeed(){
  // Calculation of top speed
  top_Speed_ = pow( 2 * power_ / (dragCoefficient_ * airDensity * area_) ,1.0/3);
  return top_Speed_;
}

void Car::accelerate(void){

  // Car can acelerate to top speed
  if(currentSpeed_>= top_Speed_){
    return;
  }


  //! \brief time_point_seconds
  //! count() returns an integer of nanoseconds, so it's divided by
  //! one billion to covert it to seconds
  auto tp = std::chrono::system_clock::now();
  double currentTime = tp.time_since_epoch().count()*0.000000001;
  //! time lapsed, current - previous time
  double dt = currentTime - time_;
  time_=currentTime;

  // At low sppeds (<40kmph) the acceleration
  // is governed by the tyre friction
  // at higher speeds we are limited by the drag force
  // This is a simplification we will use
  if(currentSpeed_<(40/3.6)){
    double a = tyreFriction*g;
    currentSpeed_+= a*dt;
  }
  else{
    double a = (power_/weight_)/currentSpeed_;
    currentSpeed_+= a*dt;
  }

}


void Car::decelerate(void){

  //! We need a dt here to decelerate
  // Compute lapsed time
  auto tp = std::chrono::system_clock::now();
  double currentTime = tp.time_since_epoch().count()*0.000000001;
  double dt = currentTime - time_;
  time_=currentTime;

  // When decelerating we are bound by tyre friction
  // This is a simplification we will use
  if(currentSpeed_<= 0.0){
    return;
  }
  double a = -tyreFriction*g;;
  currentSpeed_+= a*dt;

  if(currentSpeed_<= 0.0){
    currentSpeed_=0.0;
  }

}
