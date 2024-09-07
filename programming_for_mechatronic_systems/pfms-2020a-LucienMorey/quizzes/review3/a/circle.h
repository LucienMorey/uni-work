#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape.h"

class Circle : public Shape
{
public:
    Circle(double radius);
    void setRadius(double radius);
    double getRadius();
    double getArea ();
    bool checkPoint(double x, double y);
private:
    double radius_; //!< width of triangle
};

#endif // CIRCLE_H
