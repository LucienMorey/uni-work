// We need to include the declaration of our your shapes library
//#include ???

#include <iostream>
#include <limits>
#include <vector>
#include <shape.h>
#include <rectangle.h>
#include <circle.h>
#include <triangle.h>

void pointShapeIntercept(std::vector<Shape*> shapes, double x, double y);

int main () {

    // * Createa an vector of 4 Shapes and use it to store 2 Triangles and 2 Rectangles
    // * Accepets user input of an x,y location and computes an area of all shapes that intersect that point
    std::vector<Shape*> shapes;

    shapes.push_back(new Rectangle);
    dynamic_cast<Rectangle*>(shapes.back())->setHeightWidth(2.0, 4.0);
    shapes.push_back(new Rectangle);
    dynamic_cast<Rectangle*>(shapes.back())->setHeightWidth(3.0, 5.0);
    shapes.push_back(new Triangle(2.0, 4.0));
    shapes.push_back(new Triangle(5.0, 2.0));

    for(auto shape : shapes){
        std::cout << "the shape is a " << shape->getDescription() << std::endl;
    }

    double x;
  std::cout << "please enter a x point" << std::endl;
    while (!(std::cin >> x))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter a decimal value and try again: " << std::endl;
  }  

  double y;
  std::cout << "please enter a y point" << std::endl;
    while (!(std::cin >> x))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter a decimal value and try again: " << std::endl;
  }  

    pointShapeIntercept(shapes, x, y);
    return 0;
}

void pointShapeIntercept(std::vector<Shape*> shapes, double x, double y)
{
    for(auto shape :shapes){
        if(shape->pointInterception(x,y)){
            std::cout << "point intercepts shape" << std::endl;
        } else {
            std::cout << "point doesnt intercept shape" << std::endl;
        }
    }
}