#include "ranger.h"


Ranger::Ranger(): laser_field_(180), radar_field_(60), generator_(seed_), distribution_(4.0,5.0), samples_(0)  //defult values
{
    seed_ = std::chrono::system_clock::now().time_since_epoch().count();
}

//Essential getters for obtaining internal private variables


unsigned int Ranger::getAngularResolution(void)
{
    return angular_resolution_;
}

int Ranger::getOffset(void)
{
    return off_set_;
}
unsigned int Ranger::getFieldOfView(void)
{
   return field_of_view_;
}

double Ranger::getMax(void)
{
    return max_;
}

double Ranger::getMin(void)
{
    return min_;
}

std::string Ranger::getModel()
{
    return model_;
}

bool Ranger::setOffset(int off_set)
{
    if(getFieldOfView() == radar_field_){
              return true;
    }
    else{
            return false;       //no off set for laser
   }
}

bool Ranger::setFieldOfView(unsigned int fov)
{
    //can't be changed
    return false;

}
