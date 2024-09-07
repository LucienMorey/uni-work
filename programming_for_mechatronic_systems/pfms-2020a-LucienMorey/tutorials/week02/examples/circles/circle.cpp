#include "circle.h"
#include <cmath>

Circle::Circle(): // This constructor can call a constructor that requires a parameter and supply a default parameter
    Circle(0.0)
{
}

Circle::Circle(double radius):
    radius_(radius) /// This is the best form of initialisation for a constructor, via an initialiser list
{
}

void Circle::setRadius(double radius)
{
    radius_ = radius;
}

double Circle::area()
{
    return radius_ * radius_ * M_PI;
}

double Circle::perimeter()
{
    return 2 * M_PI *radius_;
}


