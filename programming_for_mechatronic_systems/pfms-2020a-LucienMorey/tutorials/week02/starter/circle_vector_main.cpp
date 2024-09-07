#include <iostream>
#include <vector>
#include <ctime>
#include <random>

#include "circle.h"

double sum_area(std::vector<Circle> circles);

int main(){
    int num_circles;
    std::cout << "how many circles would you like to generate?" << std::endl;
    while(!(std::cin>>num_circles)){
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input, Try again: ";
    }

    unsigned long seed = time(NULL);
    std::default_random_engine generator(seed);

    std::uniform_real_distribution<> value_distribution(0,10.0);
    std::vector<Circle> circles;
    for(auto i = 0; i<num_circles; i++){
        circles.push_back(Circle(value_distribution(generator)));
    }

    double circle_area = sum_area(circles);
    std::cout << "total circle area is " << circle_area << std::endl;

    return 0;


}

double sum_area(std::vector<Circle> circles){
    double total_area = 0.0;
    
    for(auto i : circles){
        total_area = total_area + i.get_area();
    }

    return total_area;
}