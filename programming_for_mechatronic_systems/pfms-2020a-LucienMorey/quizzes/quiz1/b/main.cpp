#include <iostream>
#include "rectangle.h"

int main()
{
  //! TODO: Create a rectangle
  Rectangle rectangle;
  rectangle.setWidthHeight(5.0, 3.5);

  // Print some info about it
  std::cout << "The area of box is " << rectangle.getArea() << std::endl;
  std::cout << "It is a " << rectangle.getDescription() << std::endl;

  //! ADDITIONAL QUESTIONS TO CONSIDER
  // - Would it make sense to create an instance of shape?
  // - A design to prohibit this is covered when we introduce polymorphism
}
