#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"

class RangerFusion: public RangerFusionInterface
{
public:
  //Default constructor should set all RangerFusion attributes to a default value
  RangerFusion();

  //See rangerfusioninterface.h for more information


private:
  //This is to cater for getRawRangeData (which generates the raw data))
  std::vector<std::vector<double>> data_;
};

#endif // RANGERFUSION_H
