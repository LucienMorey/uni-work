#include "circle.h"
#include <cmath>

/* Constructors. */

Circle::Circle()
: radius_(0.0)
{
    description_ = "circle";
}

Circle::Circle(double radius)
: radius_(radius)
{
    description_ = "circle";
}

/* Also sets the shape's X and Y by using a specific parent class constructor. */
Circle::Circle(double x, double y, double radius)
: radius_(radius)
{
    description_ = "circle";
    setCentre(x, y);
}

/* Methods. */
void Circle::setRadius(double radius)
{
    radius_ = radius;
}

double Circle::getRadius(void)
{
    return radius_;
}

double Circle::getArea(void)
{
    return M_PI * radius_ * radius_;
}


// bool Circle::checkIntercept(double x, double y)
// {
//     double dist = pow(pow(2, x - centreX_) + pow(2, y-centreY_), 0.5);
//     if (dist <= radius_) {
//         return true;
//     } else {
//         return false;
//     }
// }