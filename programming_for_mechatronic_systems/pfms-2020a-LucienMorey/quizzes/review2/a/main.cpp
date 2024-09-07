#include <iostream>
#include <vector>
#include <random>
#include <chrono>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"
#include "shape.h"

using std::cout;
using std::endl;
using std::vector;

//TASK 4: Printing area - Completing this / think about auto
void printShapeArea(std::vector<Shape *> shapes)
{
    for (auto shape : shapes) {
        std::cout << "Area of " << shape->getDescription() << " = "
            << shape->getArea() << std::endl;
    }
}


int main () {

    //TASK 2: Create a Square and Traingle, and store both of them in a vector of type `Shape`
    vector<Shape*> shapes;
    shapes.push_back(new Rectangle());
    dynamic_cast<Rectangle *>(shapes.back())->setHeightWidth(2.0, 2.0);
    shapes.push_back(new Triangle(2.0, 2.0));
    shapes.push_back(new Circle(2.0));

    printShapeArea(shapes);

    //!TODO - TASK 5: Write a program that allows the user to specify number of circles and `max_radius`.
    //! Create the circles with random lengths to be capped to `max_length`.
    std::cout << "Enter the number of circles to generate: > ";
    int circleCount;
    std::cin >> circleCount;
    std::cout << "Enter the maximum raduis of the randomly generated circles: > ";
    double max_radius;
    std::cin >> max_radius;
    std::cout << "Generating " << circleCount << " circles up to " << max_radius
        << " in size." << std::endl;
    std::default_random_engine rngEngine(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> rngDist(0.0, max_radius);
    for (int i = 0; i < circleCount; i++) {
        shapes.push_back(new Circle(rngDist(rngEngine)));
    }

    for (auto shape : shapes) {
        std::cout << "Area of " << shape->getDescription() << " = " << shape->getArea() << std::endl;
    }
}
