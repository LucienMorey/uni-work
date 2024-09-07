#include "sample.h"

Sample::Sample(double value){
    value_ = value;
}

void Sample::setvalue(double value){
    value_ = value;
}

double Sample::readvalue(void){
    return value_;
}