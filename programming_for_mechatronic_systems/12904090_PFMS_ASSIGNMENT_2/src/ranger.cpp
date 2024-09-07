#include <chrono>

#include "ranger.h"

/**
 * @brief Construct a new Ranger:: Ranger object. creates random generator for seeding data
 *
 */
Ranger::Ranger()
{
  unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
  generator = new std::default_random_engine(seed);
  distribution = new std::normal_distribution<double>(DATA_MEAN, DATA_STD_DEV);
}

/**
 * @brief Destroy the Ranger:: Ranger object deletes random generator.
 *
 */
Ranger::~Ranger()
{
  delete generator;
  delete distribution;
}

/**
 * @brief angle resolution getter
 *
 * @return unsigned int angle resolution
 */
unsigned int Ranger::getAngularResolution(void)
{
  return angle_resolution_;
}

/**
 * @brief offset getter
 *
 * @return int offset
 */
int Ranger::getOffset(void)
{
  return offset_;
}

/**
 * @brief field of view getter
 *
 * @return unsigned int field of view
 */
unsigned int Ranger::getFieldOfView(void)
{
  return field_of_view_;
}

/**
 * @brief minimum range getter
 *
 * @return double min range
 */
double Ranger::getMinRange(void)
{
  return min_range_;
}

/**
 * @brief maximum range getter.
 *
 * @return double max range
 */
double Ranger::getMaxRange(void)
{
  return max_range_;
}

/**
 * @brief sensingmethod getter. returns an enumeration of sensing types
 *
 * @return SensingMethod based on sensor type
 */
SensingMethod Ranger::getSensingMethod(void)
{
  return sensing_method_;
}

/**
 * @brief Snsor model getter
 *
 * @return std::string Model
 */
std::string Ranger::getSensorModel()
{
  return sensor_model_;
}

/**
 * @brief sensor offset setter. returns true after offset has been set
 *
 * @param offset signed integer offset to be set
 * @return true after setting paramater
 */
bool Ranger::setOffset(int offset)
{
  offset_ = offset;
  return true;
}