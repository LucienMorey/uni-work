#ifndef RECTANGLEHELPER_H
#define RECTANGLEHELPER_H

#include "rectangle.h"
/**
@class RectangleHelper
@brief RectangleHelper the intent of the class was to assist in printing properties of the Rectangle Class to terminal
*/
class RectangleHelper : public Rectangle
{
public:
  RectangleHelper();
  /**
  Member function to print to standard out the area of rectangle
  */
  void printArea(void);
};

#endif // RECTANGLEHELPER_H
