#include <iostream>
#include <vector>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"
#include "shape.h"

using std::cout;
using std::endl;
using std::vector;

double getTotalArea(vector<Shape*> shapes);

int main () {

    Rectangle rectangle1;
    rectangle1.setHeightWidth(5.0, 3.5);
    Rectangle rectangle2;
    rectangle2.setHeightWidth(5.0, 3.5);
//    std::cout << "The area of box is " << rectangle.getArea() << std::endl;
//    std::cout << "It is a " << rectangle.getDescription() << std::endl;

    Triangle triangle1(3.0, 4.0);
    Triangle triangle2(4.0, 3.0);
//    std::cout << "The area of our triangle is " << triangle.getArea() << std::endl;
//    std::cout << "It is a " << triangle.getDescription() << std::endl;

    Circle circle1;
    circle1.setRadius(2.0);
    Circle circle2;
    circle2.setRadius(5.0);

    vector<Shape*> shapes;
    shapes.push_back(&rectangle1);
    shapes.push_back(&rectangle2);
    shapes.push_back(&triangle1);
    shapes.push_back(&triangle2);
    shapes.push_back(&circle1);
    shapes.push_back(&circle2);

    cout << "total area of shapes is " <<getTotalArea(shapes) << endl;

    for (auto s : shapes) {
        cout << s->getDescription() << " has area " <<  (*s).getArea() << endl;
    }
}

double getTotalArea(vector<Shape*> shapes){
    double totalArea = 0.0;
    for(auto x :  shapes){
        totalArea += x->getArea();
    }

    return totalArea;
}
