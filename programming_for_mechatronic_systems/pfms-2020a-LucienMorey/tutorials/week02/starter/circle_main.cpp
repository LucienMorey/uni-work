#include <iostream>

#include "circle.h"

int main(){
    
    Circle circle1(1.0);
    Circle circle2(2.0);
    Circle circle3(5.0);

    double area1 = circle1.get_area();
    double area2 = circle2.get_area();
    double area3 = circle3.get_area();

    double perimeter1 = circle1.get_perimeter();
    double perimeter2 = circle2.get_perimeter();
    double perimeter3 = circle3.get_perimeter();

    std::cout << "the area of circle 1 is " << area1 << " the perimeter is " << perimeter1 << std::endl;
    std::cout << "the area of circle 2 is " << area2 << " the perimeter is " << perimeter2 << std::endl;
    std::cout << "the area of circle 3 is " << area3 << " the perimeter is " << perimeter3 << std::endl;

    return 0;
}