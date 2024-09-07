#ifndef RANGER_H
#define RANGER_H

#include "../../rangerinterface.h"
#include <vector>

class Ranger: public RangerInterface
{
public:
  Ranger();
  //First three are for ares, offset, fov - last is for mock data
  Ranger(unsigned int, unsigned int, int, std::vector<double>);

  unsigned int getAngularResolution(void);
  int getOffset(void);
  unsigned int getFieldOfView(void);

  void setAngularResolution(unsigned int);
  void setOffset(int);
  void setFieldOfView(unsigned int);

  std::vector<double> generateData();

protected:
  unsigned int fieldOfView_;
  unsigned int angularResolution_;
  int offset_;

  std::vector<double> mockData_;
};

#endif // RANGER_H
