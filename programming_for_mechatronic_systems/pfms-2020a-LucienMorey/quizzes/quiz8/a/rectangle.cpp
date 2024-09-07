#include "rectangle.h"

Rectangle::Rectangle()
{
  setHeightWidth(0, 0);
}

Rectangle::Rectangle(double width, double height)
{
  setHeightWidth(width, height);
}

void Rectangle::setHeightWidth(double width, double height)
{
  width_ = width;
  height_ = height;
  if (width_ == height_)
  {
    description_ = "square";
  }
  else
  {
    description_ = "rectangle";
  }
}

double Rectangle::getHeight()
{
  return height_;
}

double Rectangle::getWidth()
{
  return width_;
}

double Rectangle::getArea(void)
{
  return width_ * height_;
}

bool Rectangle::checkIntercept(double x, double y)
{
  return ((x >= centreX_ - (width_ / 2.0)) && (x <= centreX_ + (width_ / 2.0)) && (y <= centreY_ + (height_ / 2.0)) &&
          (y >= centreY_ - (height_ / 2.0)));
}
