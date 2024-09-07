#include <chrono>

#include "ranger.h"

Ranger::Ranger(){
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = new std::default_random_engine(seed);
    distribution = new std::normal_distribution<double>(4.0,5.0);
}

Ranger::~Ranger()
{
    delete generator;
    delete distribution;
}

unsigned int Ranger::getAngularResolution(void){
    return angle_resolution_;
}

int Ranger::getOffset(void){
    return offset_;
}

unsigned int Ranger::getFieldOfView(void){
    return field_of_view_;
}

double Ranger::getMinRange(void){
    return min_range_;
}

double Ranger::getMaxRange(void){
    return max_range_;
}

SensingMethod Ranger::getSensingMethod(void){
    return sensing_method_;
}

std::string Ranger::getSensorModel()
{
    return sensor_model_;
}

bool Ranger::setOffset(int offset)
{
    //restrict offset to a range of pi,-pi
    offset=(180/M_PI)*atan2(sin((M_PI/180)*offset),cos((M_PI/180)*offset));
    return true;
}