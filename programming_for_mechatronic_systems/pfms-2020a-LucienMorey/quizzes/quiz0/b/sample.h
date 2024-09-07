#include <iostream> // Includes std::cout and friends so we can output to console
#include <cmath>   // Includes the math library

class Sample {
public: 
    Sample(double value);
    void setvalue (double value); 
    double readvalue (void);
private:
    int value_; 
};
