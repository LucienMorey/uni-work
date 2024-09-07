#include "triangle.h"

Triangle::Triangle(double width, double height):
    width_(width), height_(height)
{
    description_ = "isoc triangle";
}

void Triangle::setHeightWidth(double width, double height)
{
    width_ = width;
    height_ = height;
}

double Triangle::getArea()
{
    return width_ * height_ * 0.5;
}

bool Triangle::pointInterception(double x, double y) {
    if(std::abs(y - centreY_ <= height_ / 2)) {
        double b = height_ / 2 - centreY_ + y;
        double a = (width_ * b) / (2 * height_);
        if ((width_ / 2.0 - std::abs(centreX_- x)) >= a) {
            return true;
        }
    }
    return false;
}
