#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"

class Sonar: public Ranger
{
public:
  //Default constructor should set all sensor attributes to a default value
  Sonar();
  std::vector<double> generateData();
  bool setAngularResolution(unsigned int anglular_resolution);
  bool setFieldOfView(unsigned int field_of_view);

private:
const unsigned int DEFAULT_ANGLE_RESOLUTION = 20;
const int DEFAULT_OFFSET = 0;
const int DEFAULT_FIELD_OF_VIEW = 20;
const double SONAR_MIN_RANGE = 0.2;
const double SONAR_MAX_RANGE = 16.0;
const SensingMethod SONAR_SENSING_METHOD = CONE;
const std::string SONAR_MODEL = "SN-001";

const std::vector<unsigned int> ACCEPTABLE_ANGLE_RESOLUTION = {20};
const std::vector<unsigned int> ACCEPTABLE_FIELD_OF_VIEW = {20}; 
};

#endif // SONAR_H
