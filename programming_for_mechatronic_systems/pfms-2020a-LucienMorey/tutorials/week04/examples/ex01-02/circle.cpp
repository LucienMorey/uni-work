#include "circle.h"
#include <math.h>

Circle::Circle(double radius):
    radius_(radius)
{
    description_ = "circle";
}

void Circle::setRadius(double radius)
{
    radius_ = radius;
}

double Circle::getArea()
{
    return radius_ * radius_ * M_PI;
}
