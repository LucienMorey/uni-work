#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "shape.h"

class Triangle : public Shape
{
public:
    Triangle(double width, double height);
    void setHeightWidth(double width, double height);
    bool pointInterception(double x, double y);
    double getArea ();
private:
    double width_; //!< width of triangle
    double height_;//!< height of triangle
};


#endif // TRIANGLE_H
