#ifndef CAR_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define CAR_H

#include <string>
#include "controllerinterface.h"
#include <chrono>         // std::chrono::seconds, time lapse etc

const double airDensity = 1.25; // air density (in kg/m³), at sea level has the value 1.25 kg/m³.
const double g = 9.81; //gravity  m/s2
const double power_conversion = 746.0; // converts from horse power HP to Watts
const double tyreFriction=1.5;

class Car : public ControllerInterface {
public:
  //!TODO - TASK 1 : Modify Car so that it inherits behaviour from the base class, ControllerInterface]
  Car(std::string make, std::string model,double height, double width, double horsePower, double dragCoefficient, double weight);

  //getters
  std::string getMake(void);
  std::string getModel(void);
  //! Gets the current speed
  /*!
    \return current speed [m/s].
    \sa Car() and accelerate() and decelerate();
  */
  double getCurrentSpeed(void);

  //setters
  void setMake(std::string);
  void setModel(std::string);

  //! Computes the top speed
  /*!
    \return top speed [m/s].
    \sa Car()
  */
  double calculateTopSpeed(void);
  void decelerate(void);
  void accelerate(void);

protected:
  std::string make_;  //!< make of car
  std::string model_; //!< model of car
  double top_Speed_;  //!< top speed of car
  double currentSpeed_;//!< current speed of car

  double area_;       //!< area in front of car
  double power_;      //!< power of car [W]
  //!< Drag coefficient ranges between 0.25 for the Honda Insight to 0.58 for the Jeep Wrangler TJ Soft Top.
  //!< The average value is 0.33
  double dragCoefficient_ ;
  double weight_;     //!< Weight of car [kg]

  bool carStationary=true;
};


#endif // CAR_H
