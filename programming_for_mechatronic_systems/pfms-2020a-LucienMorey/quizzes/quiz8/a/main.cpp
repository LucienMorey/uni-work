#include <iostream>
#include <vector>

#include "rectangle.h"
#include "rectanglehelper.h"

using std::cout;
using std::endl;
using std::vector;

void printInterceptCheck(Rectangle rect, double x, double y)
{
  bool intercepts = rect.checkIntercept(x, y);
  std::cout << rect.getDescription() << " ";

  if (!intercepts)
  {
    std::cout << "does not intercept ";
  }
  else
  {
    std::cout << "intercepts ";
  }
  std::cout << "point [x,y]=[" << x << "," << y << "]" << std::endl;
}

int main()
{
  Rectangle rect(1.0, 1.0);

  printInterceptCheck(rect, 0.0, 0.0);

  double x = 3.0, y = 3.0;
  rect.offset(x, y);

  printInterceptCheck(rect, 0.0, 0.0);

  // Attempt to use a helper for the Rectangle
  RectangleHelper rectangleHelper;
  rectangleHelper.setRectangle(rect);
  rectangleHelper.printArea();
}
