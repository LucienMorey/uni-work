#include "circle.h"
#include <cmath>

Circle::Circle(double radius):
    Shape(0.0, 0.0),radius_(radius)
{
    description_ = "circle";
}

void Circle::setRadius(double radius)
{
    radius_ = radius;
}

double Circle::getRadius()
{
    return radius_;
}


double Circle::getArea()
{
    return radius_ * radius_ * M_PI;
}

bool Circle::checkPoint(double x, double y)
{

  double dist = std::pow((std::pow(x - centreX_, 2) + std::pow(y - centreY_, 2)),0.5);
  return dist < radius_;
}
