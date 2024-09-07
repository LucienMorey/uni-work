#ifndef RANGER_H
#define RANGER_H

#include <string>
#include <random>

#include "rangerinterface.h"

class Ranger: public RangerInterface
{
  public:
  //Default constructors should set all sensor attributes to a default value
  Ranger();
  ~Ranger();

  //See rangerinterface.h for more information

  //Generates raw data for sensor
  virtual std::vector<double> generateData() = 0;

  //Essential getters for obtaining internal private variables
  unsigned int getAngularResolution(void);
  int getOffset(void);
  unsigned int getFieldOfView(void);
  double getMaxRange(void);
  double getMinRange(void);
  SensingMethod getSensingMethod(void);
  std::string getSensorModel(void);

  //Essential setters for setting internal private variables
  bool setAngularResolution(unsigned int)=0;
  bool setOffset(int offset);
  bool setFieldOfView(unsigned int)=0;

  protected:
  unsigned int angle_resolution_;
  int offset_;
  unsigned int field_of_view_;
  double min_range_;
  double max_range_;
  SensingMethod sensing_method_;
  std::string sensor_model_;

  std::default_random_engine* generator;
  std::normal_distribution<double>* distribution;


};

#endif // RANGER_H
