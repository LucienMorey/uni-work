#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape.h"

class Circle : public Shape
{
public:
  Circle();

  void setRadius(double radius);
  double getArea();
  bool pointInterception(double x , double y);
  double getRadius();

private:
  double radius_;
};

#endif