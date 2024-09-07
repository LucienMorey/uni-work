#ifndef LASER_H
#define LASER_H

#include "ranger.h"

class Radar: public Ranger
{
public:
  Radar();

  void setAngularResolution(unsigned int ares);
  void setOffset(unsigned int degrees);
};

#endif // LASER_H
