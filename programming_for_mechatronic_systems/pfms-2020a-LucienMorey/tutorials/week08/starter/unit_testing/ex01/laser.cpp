#include "laser.h"
#include <random>

Laser::Laser()
{
    angular_resolution_ = 10;
    field_of_view_ = 180;
    off_set_ = 0;
    max_=8.0;
    min_=0.2;
    model_ = "UTM-XXL";
}

std::vector<double> Laser::generateData()
{

        double num;
        std::vector<double> data(field_of_view_/angular_resolution_+1);
        for(int i = 0; i<=field_of_view_/angular_resolution_; i++)
        {
            num = distribution_(generator_);
            //dont use push back here need to define vector size
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

int Laser::getSample(void)
{
    return samples_;
}

std::string Laser::getModel(void)
{
    return model_;
}

bool Laser::setAngularResolution(unsigned int angular_res)
{
  angular_resolution_=angular_res;
  return true;       //can't change angular resolution for radar.
}

