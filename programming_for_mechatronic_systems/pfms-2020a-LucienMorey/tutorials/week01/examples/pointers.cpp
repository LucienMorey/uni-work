#include <iostream>

int main () {

    double x = 0.0;
    x = 41012;
    std::cout << "x = " << x << std::endl;

    double* ip = &x;
    std::cout << "value at ip = " << *ip << std::endl;
    std::cout << "value of ip = " << ip << std::endl;

    double &y = x;
    std::cout << "y = " << y << std::endl;

    double z = 1.0;
    std::cout << "z = " << z << std::endl;

    ip = &z;
    std::cout << "value at ip = " << *ip << std::endl;

    y = z; // Only changing the value not the reference
    z = 100;

    std::cout << "value at ip = " << *ip << std::endl;
    std::cout << "x = " << x << std::endl;
    std::cout << "y = " << y << std::endl;


    return 0;
}
