#include "rectanglehelper.h"
#include <iostream>

RectangleHelper::RectangleHelper()
{
}

void RectangleHelper::printArea()
{
  std::cout << rectangle_.getDescription() << " ";
  std::cout << "area=[" << rectangle_.getArea() << "]" << std::endl;
}

void RectangleHelper::setRectangle(Rectangle rectangle)
{
  rectangle_ = rectangle;
}
