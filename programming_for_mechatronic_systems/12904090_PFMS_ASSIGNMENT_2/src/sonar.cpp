#include "sonar.h"

#include <algorithm>

/**
 * @brief Construct a new Sonar:: Sonar object
 *
 */
Sonar::Sonar()
{
  angle_resolution_ = DEFAULT_ANGLE_RESOLUTION;
  offset_ = DEFAULT_OFFSET;
  field_of_view_ = DEFAULT_FIELD_OF_VIEW;
  min_range_ = SONAR_MIN_RANGE;
  max_range_ = SONAR_MAX_RANGE;
  sensing_method_ = SONAR_SENSING_METHOD;
  sensor_model_ = SONAR_MODEL;
}

/**
 * @brief  angle resolution setter
 *
 * @param angle_resolution unsigned int
 * @return true if param is set sucessfully
 * @return false if param fails to be set
 */
bool Sonar::setAngularResolution(unsigned int angle_resolution)
{
  // check for valid angle resolution
  // return if the sensor member has been overwritten
  if (std::find(ACCEPTABLE_ANGLE_RESOLUTION.begin(), ACCEPTABLE_ANGLE_RESOLUTION.end(), angle_resolution) !=
      ACCEPTABLE_ANGLE_RESOLUTION.end())
  {
    angle_resolution_ = angle_resolution;
    field_of_view_ = angle_resolution;
    return true;
  }
  else
  {
    angle_resolution_ = DEFAULT_ANGLE_RESOLUTION;
    field_of_view_ = DEFAULT_FIELD_OF_VIEW;
    return false;
  }
}

/**
 * @brief Field of view setter
 *
 * @param field_of_view unsigned int
 * @return true if param is set successfully
 * @return false if param fails to be set
 */
bool Sonar::setFieldOfView(unsigned int field_of_view)
{
  // check for valid field of view
  // return the sensor member has been overwritten
  if (std::find(ACCEPTABLE_FIELD_OF_VIEW.begin(), ACCEPTABLE_FIELD_OF_VIEW.end(), field_of_view) !=
      ACCEPTABLE_FIELD_OF_VIEW.end())
  {
    field_of_view_ = field_of_view;
    angle_resolution_ = field_of_view;
    return true;
  }
  else
  {
    field_of_view_ = DEFAULT_FIELD_OF_VIEW;
    angle_resolution_ = DEFAULT_FIELD_OF_VIEW;
    return false;
  }
}

/**
 * @brief generate sonar data sample.  capped at min and max sensor range.
 *
 * @return std::vector<double>  data sample
 */
std::vector<double> Sonar::generateData()
{
  std::vector<double> data(field_of_view_ / angle_resolution_);
  for (auto& data_point : data)
  {
    data_point = (*distribution)(*generator);
    data_point = std::max(data_point, min_range_);
    data_point = std::min(data_point, max_range_);
  }
  return data;
}