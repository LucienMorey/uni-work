#include <iostream>
#include <vector>

#include <random>   // Includes the random number generator
#include <iomanip>  // For formatting of data output
#include <ctime>    // Used to print timestamp along with data
#include <memory>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"

using std::cout;
using std::endl;
using std::vector;

int main () {

    vector<Shape* > shapes;

    //Let's use a random number generator for the legths
    std::random_device generator;
    std::uniform_real_distribution<double> distribution(0.0001,5.0);

    int numTriangles=2,numRectangles=2,numCircles=3;


    for(long int num=0;num<numTriangles;num++){
      shapes.push_back(new Triangle(distribution(generator), distribution(generator)));
      std::cout << shapes.at(shapes.size()-1)->getDescription() << " #" << num << std::endl;
    }

    for(long int num=0;num<numRectangles;num++){
      shapes.push_back(new Rectangle(distribution(generator), distribution(generator)));
      std::cout << shapes.at(shapes.size()-1)->getDescription() << " #" << num << std::endl;
    }

    for(long int num=0;num<numCircles;num++){
      shapes.push_back(new Circle(distribution(generator)));
      std::cout << shapes.at(shapes.size()-1)->getDescription() << " #" << num << std::endl;
    }

    double totalArea=0;
    for (auto s : shapes) {
        cout << s->getDescription() << " has area " <<  (*s).getArea() << endl;
        totalArea+=s->getArea();
    }

    cout << "Total area is :" << totalArea << endl;

}

