#include "radar.h"
#include <random>

//defult constructor
Radar::Radar()
{
    angular_resolution_ = 20;
    field_of_view_ = 60;
    off_set_ = 0;
    max_=16.0;
    min_=0.2;
    model_ = "RAD-00";

}

std::vector<double> Radar::generateData()
{
    std::vector<double> data(field_of_view_/angular_resolution_);
    double num;
   // std::cout<<"radar generated data"<<std::endl;

    for(int i = 0; i<field_of_view_/angular_resolution_; i++)
    {
        num = distribution_(generator_);
        if(num >= max_){
           data.at(i) = max_;
        }
        else if (num <= min_){
            data.at(i) = min_;
            }
        else{
            data.at(i) = num;
    }


}
    samples_++;
    return data;
}

int Radar::getSample(void)
{
    return samples_;
}

std::string Radar::getModel(void)
{
    return model_;
}


bool Radar::setAngularResolution(unsigned int angular_res)
{
    return false;       //can't change angular resolution for radar.
}
