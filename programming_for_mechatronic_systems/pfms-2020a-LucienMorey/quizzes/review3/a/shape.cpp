#include "shape.h"

Shape::Shape(double centreX, double centreY):
    centreX_(centreX),
    centreY_(centreY),
    description_("unknown shape")
{
}

Shape::~Shape()
{

}

void Shape::setCentre(double x, double y)
{
    centreX_=x;
    centreY_=y;
}


std::string Shape::getDescription()
{
    return description_;
}

void Shape::offset(double x, double y)
{
    centreX_ += x;
    centreY_ += y;
}

