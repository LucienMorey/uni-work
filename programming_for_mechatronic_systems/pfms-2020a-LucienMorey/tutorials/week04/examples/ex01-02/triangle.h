#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "shape.h"

class Triangle : public Shape
{
public:
    Triangle(double width, double height);
    //~Triangle();
    void setHeightWidth(double width, double height);
    double getArea ();
private:
    double width_; //!< width of triangle
    double height_;//!< height of triangle
};


#endif // TRIANGLE_H
