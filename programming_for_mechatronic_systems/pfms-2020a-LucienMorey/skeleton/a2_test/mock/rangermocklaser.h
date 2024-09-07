#ifndef RANGERMOCKLASER_H
#define RANGERMOCKLASER_H

#include "rangerinterface.h"
#include "laser.h"
#include <vector>

class RangerMockLaser: public Laser
{
public:
  RangerMockLaser();
  //First three are for ares, offset, fov - last is for mock data
  RangerMockLaser(unsigned int fov, unsigned int ares, int offset, std::vector<double> mockData);

  unsigned int getAngularResolution(void);
  int getOffset(void);
  unsigned int getFieldOfView(void);

  bool setAngularResolution(unsigned int);
  bool setOffset(int);
  bool setFieldOfView(unsigned int);

  SensingMethod getSensingMethod(void);

  std::vector<double> generateData();

protected:
  unsigned int fieldOfView_;
  unsigned int angularResolution_;
  int offset_;

  std::vector<double> mockData_;
};

#endif // RANGERMOCKLASER_H
