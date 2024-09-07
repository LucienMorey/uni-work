#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "shape.h"

#include "line.h"

class Triangle : public Shape
{
public:
    Triangle(double width, double height);
    //~Triangle();
    void setHeightWidth(double width, double height);
    double getArea ();
    bool checkIntercept(double x, double y);//! return true if the point is within the circle
    void offset(double x, double y);

private:
    void updateEdges();

private:
    double width_; //!< width of triangle
    double height_;//!< height of triangle
    Line left_edge_;
    Line right_edge_;
    Line base_edge_;
};


#endif // TRIANGLE_H
