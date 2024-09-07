#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape.h"

class Circle : public Shape
{
public:
  Circle();
  ~Circle();

  void setRadius(double radius);
  double getArea();
  bool pointInerception(double x , double y);

private:
  double radius_;
};

#endif