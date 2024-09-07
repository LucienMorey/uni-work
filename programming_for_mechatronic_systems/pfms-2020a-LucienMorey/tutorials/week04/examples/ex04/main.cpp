#include <iostream>
#include <vector>

#include <random>   // Includes the random number generator
#include <sstream>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"

using std::cout;
using std::endl;
using std::vector;

int main () {

//    Modify the main (which uses the library) and

//    * Createa an vector of 4 Shapes and use it to store 2 Triangles and 2 Rectangles
//    * Accepets user input of an x,y location and computes an area of all shapes that intersect that point

    vector<Shape* > shapes;

    //Let's use a random number generator for the legths
    std::random_device generator;
    std::uniform_real_distribution<double> distribution(0.0001,5.0);

    int numTriangles=2,numRectangles=2;


    for(long int num=0;num<numTriangles;num++){
      std::cout << "triangle #" << num << std::endl;
      shapes.push_back(new Triangle(distribution(generator), distribution(generator)));
    }

    for(long int num=0;num<numRectangles;num++){
      std::cout << "rectangle #" << num << std::endl;
      shapes.push_back(new Rectangle(distribution(generator), distribution(generator)));
    }


    // * Write a program that accepets an x,y location and computes an area of all circles that intersect that point
    std::string mystr;
    double x,y;
    std::cout << std::flush;
    std::cout << "Supply x location? ";
    std::getline (std::cin,mystr);
    std::stringstream(mystr) >> x;
    std::cout << "Supply y location? ";
    std::getline (std::cin,mystr);
    std::stringstream(mystr) >> y;

    double totalArea=0;
    for (auto s : shapes) {
      if(s->checkIntercept(x,y)){
        totalArea+= s->getArea();
      }
    }
    std::cout << "Total Area: " <<  totalArea << std::endl;

    return 0;

}
















//      vector<std::shared_ptr<Shape> > shapes;

//      shapes.emplace_back(new Circle(distribution(generator)));
