#include <iostream>
#include <vector>
#include <random>
#include <chrono>

#include "circle.h"
#include "rectangle.h"
#include "triangle.h"
#include "shape.h"

using std::cout;
using std::endl;
using std::vector;

//!TODO - TASK 4: Printing description - Completing this / think about auto
void printAreaOfShapes(vector<Shape*> shapes);

double generateRandomNumber(double min, double max);

void generateCircles();


int main () {

    //!TODO - TASK 2: Create a Square and Traingle, and store both of them in a vector of type `Shape`
    Rectangle rectangle;
    rectangle.setHeightWidth(2.0, 2.0);
    Triangle triangle(1.0, 2.0);

    vector<Shape*> shapes;
    shapes.push_back(&rectangle);
    shapes.push_back(&triangle);

    printAreaOfShapes(shapes);

    //!TODO - TASK 5: Write a program that allows the user to specify number of circles and `max_radius`.
    //! Create the circles with random lengths to be capped to `max_length`.
    generateCircles();
    

}

void printAreaOfShapes(vector<Shape*> shapes){
    for(auto x : shapes){
        cout << x->getDescription() << " has area "<< x->getArea() << endl;
    }
}

double generateRandomNumber(double min, double max)
{
  unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> randomiser(min, max);

  return randomiser(generator);
}

void generateCircles()
{
    int num_of_circles;
    cout << "how many circles would you like to generate?" << endl;
    while(!(std::cin >> num_of_circles)){
        std::cin.clear();
        std::cin.ignore();
    }

    num_of_circles = abs(num_of_circles);

    double max_radius;
    cout << "what should the maximum radius be? (minimum will be 0.1)" << endl;
    while(!(std::cin >> max_radius)){
        std::cin.clear();
        std::cin.ignore();
    }
    vector<Circle*> circles;

    for(int i = 0; i < num_of_circles; i++){
        circles.push_back(new Circle);
        circles.at(i)->setRadius(generateRandomNumber(0.1, max_radius));
    }

    for (int i = 0; i< circles.size(); i++){
        cout << "circle " << i+1 <<" radius is: " << circles.at(i)->getRadius() << endl;
    }

    //cleanup pointers after use
    for(auto circle : circles){
        delete circle;
    }
    circles.clear();
}