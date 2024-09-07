// We need to include the declaration of our new rectangle class in order to use it.
#include "sample.h"

#include <iostream>

int main () {

    // Create a rectangle object
    Sample sample(5.0);

    // Get the area and print it to screen
    double result = sample.readvalue();
    std::cout << "the result value in the object is "<< result << std::endl;

    return 0;
}
