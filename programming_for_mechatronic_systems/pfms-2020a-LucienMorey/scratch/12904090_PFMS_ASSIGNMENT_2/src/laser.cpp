#include "laser.h"
#include <algorithm>

Laser::Laser(){
    angle_resolution_ = DEFAULT_ANGLE_RESOLUTION;
    offset_ = DEFAULT_OFFSET;
    field_of_view_ = DEFAULT_FIELD_OF_VIEW;
    min_range_ = LASER_MIN_RANGE;
    max_range_ = LASER_MAX_RANGE; 
    sensing_method_= LASER_SENSING_METHOD;
    sensor_model_ = LASER_MODEL; 
}

bool Laser::setAngularResolution(unsigned int angle_resolution)
{
    //check for valid angle resolution
    //return if the sensor member has been overwritten
    if(std::find(ACCEPTABLE_ANGLE_RESOLUTION.begin(), ACCEPTABLE_ANGLE_RESOLUTION.end(), angle_resolution) != ACCEPTABLE_ANGLE_RESOLUTION.end()){
        angle_resolution_ = angle_resolution;
        return true;
    } else {
        angle_resolution_ = DEFAULT_ANGLE_RESOLUTION;
        return false;
    }    
}

bool Laser::setFieldOfView(unsigned int field_of_view)
{
    //check for valid field of view
    //return the sensor member has been overwritten
    if(std::find(ACCEPTABLE_FIELD_OF_VIEW.begin(), ACCEPTABLE_FIELD_OF_VIEW.end(), field_of_view) != ACCEPTABLE_FIELD_OF_VIEW.end()){
        field_of_view_=field_of_view;
        return true;
    } else {
        field_of_view_ = DEFAULT_FIELD_OF_VIEW;
        return false;
    }  
}

std::vector<double> Laser::generateData()
{
    std::vector<double> data((field_of_view_/angle_resolution_)+1);
    for (auto &data_point : data)
    {
        data_point = (*distribution)(*generator);
        data_point = std::max(data_point,min_range_);
        data_point = std::min(data_point,max_range_);
    }

    return data;
}