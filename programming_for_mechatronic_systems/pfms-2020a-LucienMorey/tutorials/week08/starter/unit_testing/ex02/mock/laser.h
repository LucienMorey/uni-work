#ifndef LASER_H
#define LASER_H

#include "ranger.h"

class Laser: public Ranger
{
public:
  Laser();

  void setAngularResolution(unsigned int ares);
  void setOffset(unsigned int degrees);
};

#endif // LASER_H
