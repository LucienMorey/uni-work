#include "rectangle.h"

Rectangle::Rectangle(){

}

void Rectangle::setWidthHeight(double width, double height){
    width_ = width;
    height_ = height;
}

double Rectangle::area(){
    auto area = width_ * height_;
    return area;
}

double Rectangle::perimeter(){
    auto perimeter = 2*width_ +2*height_;
    return perimeter;
}
