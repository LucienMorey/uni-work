#include <iostream>
#include <stdlib.h>
#include <tuple>
#include <math.h>
#include <vector>

void point_and_reference();
void c_arrays();
void loops_typecasting();

//exercise 5
std::pair<bool, double> ex5_square(double x);

//exercise 6
std::tuple<bool, double, double, double> ex6_square(double x);

//Exercise 7
struct Sensor{
    uint16_t num_samples;
    double data[5];

};
void ex7_print(Sensor sensor);
void ex7();

//exercise 8
void ex8();

//exercise 9
void ex9();

int main () {
    
    point_and_reference();
    
    c_arrays();
    
    loops_typecasting();

    ex5_square(2.2);

    ex6_square(5.0);

    ex7();

    ex8();

    ex9();
    
    std::cout << "Hello Class" << std::endl;
    return 0;
}

void point_and_reference(){
    double x;
    x = 41012.0;
    
    double* ip = &x;

    std::cout << "the value ip is pointing to " << *ip << std::endl;

    double &y = x;
    
    std::cout << "the value y is referencing is " << y << std::endl;

    double z = 1.0;
    
    ip = &z;

    y = z;

    std::cout << "ip is referencing " << *ip << " y is referencing " << y << std::endl;
}

void c_arrays(){
    double x[10] = {0,1,2,3,4,5,6,7,8,9};

    for(int i = 0; i<(sizeof(x)/sizeof(double)); i++){
        x[i] = i;
        std::cout << "the value of the array at index " << i << " is " << x[i] << std::endl;
    }

    for(int i = 0; i<(sizeof(x)/sizeof(double)); i++){
        *(x + i) = i;
        std::cout << "the value of the array at index " << i << " is " << *(x+i) << std::endl;
    }
}

void loops_typecasting(){
    char x[] = "41012";
    int i = 0;
    int sum = 0;

    while(i <  ((sizeof(x)/sizeof(char)))-1){
        sum = sum + ((int)x[i] - '0');
        i++;
    }
    std::cout << "the sum of elements in x is " << sum << std::endl;

    i = 0;
    sum = 0;

    while(i < 2){
        sum = sum + ((int)x[i]-'0');
        i++;
    }
    std::cout << "the sum of elements less than 2 in x is " << sum << std::endl;
}

std::pair<bool, double> ex5_square(double x){
    if(x > 0.0){
        std::cout << "square is " << pow(x,2) << std::endl;
        return std::make_pair(true, pow(x,2));
    }
}

std::tuple<bool, double, double, double> ex6_square(double x){
    if(x > 0.0){
        std::cout << "cube is " << pow(x,3) << std::endl;
        return std::make_tuple(true, pow(x,2), pow(x, 3), (x + 1));
    }
}

void ex7_print(Sensor sensor){
    for(int i = 0; i < (sizeof(sensor.data)/sizeof(double)); i++){
        std::cout << "sensor data at index " << i << " is " << sensor.data[i] << std::endl;
    }
}

void ex7(){
    Sensor sensor1;

    for(int i = 0; i<5; i++){
        sensor1.data[i] = i;
    }

    ex7_print(sensor1);

    Sensor sensor2{ 0,  {1.0, 2.0, 5.0, 7.0, 1.0}};
}

void ex8(){
    std::vector<int> data;
    for(auto i = 0; i < 10; i++){
        data.push_back(i);
    }

    for(auto i : data){
        std::cout << "vector data at position " << i << " is " << data.at(i) << std::endl;
    }

    std::vector<float> data2(10);
    std::cout << "data2 length is " << data2.size() << std::endl;
    for(auto i = 0; i < 10;i++){
        data2.at(i) = i;
        std::cout << "vector data2 at position " << i << " is " << data2.at(i) << std::endl;
    }
}

void ex9(){
    std::vector<std::vector<int>> data(4 , std::vector<int>(4,0));
    for(int i = 0; i < 4; i++){
        for(int j =0; j < 4; j++){
            data[i][j] = i;       
        }
    }

    for(int i = 0; i < 4; i++){
        for(int j =0; j < 4; j++){
            std::cout << data[i][j] << " ";       
        }
        std::cout << std::endl;
    }
    
}