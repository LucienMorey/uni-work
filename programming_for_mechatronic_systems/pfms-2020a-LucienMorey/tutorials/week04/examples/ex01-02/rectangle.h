#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape
{
public:
    //! We also change the contrsuctor, as we are unable to acccess
    //! the setHeightWidth method if we have a vector of Shape pointers
    //! At that stage the only methods we can acess are those in
    //! base class of Shape, so we need to set width and height
    //! I the constructor
    Rectangle(double width, double height);
    void setHeightWidth(double width, double height);
    
    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Consider if getArea() is a method that should exist in Rectangle?
    // Should all shapes be able to computer Area? Do all shapes have this attribute?
    // A design to enable this is covered in when we introduce polymorphism
    double getArea (void);
private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?

    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
