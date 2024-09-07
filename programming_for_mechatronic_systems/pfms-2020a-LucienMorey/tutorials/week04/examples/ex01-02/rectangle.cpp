#include "rectangle.h"
#include <cmath>
#include <limits>

Rectangle::Rectangle(double width, double height):
width_(width), height_(height)
{
    description_="rectangle";
}

void Rectangle::setHeightWidth(double width, double height)
{
    //!NOTES 
    // This is a example of why you should not allow direct access to member variables (why they are private)
    // Given we have a function to set the member varaibles, we also can leverage this function to set any
    // other member variables required, of perform any other operations that are needed to be executed
    // (such as invoking other methods)

    width_ = width;
    height_ = height;
    if (std::fabs(width_- height_) < (std::numeric_limits<double>::min())) {
        description_ = "square";
    } else {
        description_ = "rectangle";
    }
}

double Rectangle::getArea(void)
{
    return width_ * height_;
}

