#include <math.h>
#include "circle.h"

Circle::Circle(double radius){
    this->radius = radius;
}

void Circle::set_radius(double radius){
    this->radius = radius;
}

double Circle::get_area(){
    double area = M_PI*pow(radius,2.0);
    return area; 
}

double Circle::get_perimeter(){
    double perimeter = 2 * M_PI * radius;
    return perimeter;
}