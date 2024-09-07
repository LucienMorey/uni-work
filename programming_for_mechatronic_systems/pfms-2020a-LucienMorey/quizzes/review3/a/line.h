#ifndef LINE_H
#define LINE_H
#include "math.h"

class Line
{
public:
    Line();
    Line(double gradient, double y_intercept);
    Line(double ax, double ay, double bx, double by);
    void fromPoints(double ax, double ay, double bx, double by);
    void setGradient(double gradient);
    void setYIntercept(double y_intercept);
    bool pointAboveLine(double x, double y);
    double getValD();
    double getValDr();
private:
    double gradient_;
    double y_intercept_;
    double ax_,bx_,ay_,by_;
};

#endif // LINE_H
