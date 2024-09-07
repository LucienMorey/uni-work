/*!
@file circle.h
@author Daniel Selmes
@date 2020-03-28
*/

#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape.h"

class Circle : public Shape
{
public:
    Circle();
    Circle(double radius);
    Circle(double x, double y, double radius);
    void setRadius(double raduis);
    double getRadius(void);
    double getArea(void);
    bool checkIntercept(double x, double y);
private:
    double radius_;
};

#endif