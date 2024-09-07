#ifndef LASER_H
#define LASER_H

#include "ranger.h"

class Laser: public Ranger
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Laser();
  std::vector<double> generateData();
  bool setAngularResolution(unsigned int anglular_resolution);
  bool setFieldOfView(unsigned int field_of_view);

private:
const unsigned int DEFAULT_ANGLE_RESOLUTION = 10;
const int DEFAULT_OFFSET = 0;
const int DEFAULT_FIELD_OF_VIEW = 180;
const double LASER_MIN_RANGE = 0.2;
const double LASER_MAX_RANGE = 8.0;
const SensingMethod LASER_SENSING_METHOD = POINT;
const std::string LASER_MODEL = "SICK-XL";

const std::vector<unsigned int> ACCEPTABLE_ANGLE_RESOLUTION = {30, 10};
const std::vector<unsigned int> ACCEPTABLE_FIELD_OF_VIEW = {180}; 
};

#endif // LASER_H
