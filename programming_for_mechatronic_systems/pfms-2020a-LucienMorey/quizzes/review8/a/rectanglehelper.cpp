#include "rectanglehelper.h"
#include <iostream>

RectangleHelper::RectangleHelper()
{

}

void RectangleHelper::printArea(){

  std::cout << getDescription() <<  " " ;
  std::cout << "area=[" <<  getArea() << "]" << std::endl;

}
