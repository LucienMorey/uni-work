#include "circle.h"
#include <math.h>

Circle::Circle() : radius_(0.0)
{
  description_ = "cicrle";
}

void Circle::setRadius(double radius)
{
  radius_ = radius;
}

double Circle::getArea(void)
{
  return M_PI * pow(radius_, 2);
}

bool Circle::pointInterception(double x, double y){
  return sqrt(pow((x-centreX_), 2) + pow((y-centreY_), 2)) < radius_;
}