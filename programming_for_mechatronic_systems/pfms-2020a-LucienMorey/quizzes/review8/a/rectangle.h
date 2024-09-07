#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape
{
public:
    Rectangle();
    Rectangle(double width,double height);
    void setHeightWidth(double width, double height);
    double getWidth();
    double getHeight();

    double getArea (void);
    bool checkIntercept (double x, double y);
private:

    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // RECTANGLE_H
