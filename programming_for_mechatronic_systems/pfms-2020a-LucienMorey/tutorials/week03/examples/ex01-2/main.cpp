#include <iostream>
#include <vector>

#include "rectangle.h"
#include "circle.h"

int main () {

    Rectangle rectangle;
    rectangle.setHeightWidth(5.0, 3.5);
    std::cout << "The area of rectangle is " << rectangle.getArea() << std::endl;
    std::cout << "It is a " << rectangle.getDescription() << std::endl;

    Circle circle(3.0);
    std::cout << "The area of circle is " << circle.getArea() << std::endl;
    std::cout << "It is a " << circle.getDescription() << std::endl;

}
