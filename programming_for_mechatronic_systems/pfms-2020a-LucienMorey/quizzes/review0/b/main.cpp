#include "sample.h"
#include <iostream>

int main () {

    // Create an object
    Sample sample(2);

    // Set the values
    sample.setvalue(6);

    //Print it to screen
    std::cout << sample.readvalue() << std::endl;

    return 0;
}
