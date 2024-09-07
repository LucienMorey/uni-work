#include "laser.h"

/**
 * @brief Construct a new Laser:: Laser object. If a user fails to parse an appropriate pair of scan time and angle
 * resolution then the sensor will default to mode 1.
 *
 * @param scanning_time inital scanning time for sensor
 * @param angle_resolution initial angle resolution for sensor
 */
Laser::Laser(double scanning_time, int angle_resolution)
  : scanning_time_(scanning_time), angle_resolution_(angle_resolution), first_scan_(true), sample_counter_(1)
{
  if (!(((scanning_time_ == SCAN_TIME_MODE_1) && (angle_resolution_ == ANGLE_RESOLUTION_MODE_1)) ||
        ((scanning_time_ == SCAN_TIME_MODE_2) && (angle_resolution_ == ANGLE_RESOLUTION_MODE_2))))
  {
    scanning_time_ = SCAN_TIME_MODE_1;
    angle_resolution_ = ANGLE_RESOLUTION_MODE_1;
  }
}

/**
 * @brief Destroy the Laser:: Laser object
 *
 */
Laser::~Laser()
{
}

/**
 * @brief Get new scan of data
 *
 * @return vector<double> latest scan data
 */
std::vector<double> Laser::getData()
{
  // if this is the first scan document the timestamp
  if (first_scan_)
  {
    first_scan_time_ = timer.now();
    first_scan_ = false;
  }

  // update last scan time after finished blocking
  scan_timestamp_ = timer.now();

  // obtain random data for mock
  double dummy_data_ = generateRandomData();
  
  //clear old data from container
  laser_data_.clear();

  for (int i = MIN_HEADING; i <= MAX_HEADING; i += angle_resolution_)
  {
    // read data into container
    laser_data_.push_back(dummy_data_);

    // reduce dummy data value as per assignment
    dummy_data_ *= 0.95;

    // gate out of range values
    dummy_data_ = std::max(dummy_data_, MIN_RANGE);
  }

  // determine time since last scan. block if less than required scan time
  std::chrono::duration<double> scan_time = timer.now() - scan_timestamp_;
  while (scan_time.count() < scanning_time_)
  {
    scan_time = timer.now() - scan_timestamp_;
  }

  // increment sample counter
  sample_counter_++;

  return laser_data_;
}

/**
 * @brief get configurable scanning time paramater value
 *
 * @return double scanning time in seconds
 */
double Laser::getScanningTime()
{
  return scanning_time_;
}

/**
 * @brief get configurable anglular resolution parameter value
 *
 * @return int angular resolution in degrees
 */
int Laser::getAngleResolution()
{
  return angle_resolution_;
}

/**
 * @brief set scanning time parameter and coupled angular resolution. successful config change will clear previous data
 *
 * @param scanning_time
 * @return true if successful in setting scanning time
 * @return false if failure to set scanning time
 */
bool Laser::setScanningTime(double scanning_time)
{
  // check if calid config value was parsed
  if ((scanning_time == SCAN_TIME_MODE_1) || (scanning_time == SCAN_TIME_MODE_2))
  {
    // check if changing values or remaining the same
    if (scanning_time_ != scanning_time)
    {
      // clear data from previous configuration
      laser_data_.clear();

      // update last configuration change
      last_config_change_ = timer.now();
    }

    // set valid parameter and coupled angle resolution
    scanning_time_ = scanning_time;
    if (scanning_time == SCAN_TIME_MODE_1)
    {
      angle_resolution_ = ANGLE_RESOLUTION_MODE_1;
    }
    else
    {
      angle_resolution_ = ANGLE_RESOLUTION_MODE_2;
    }
  }

  return (scanning_time_ == scanning_time);
}
/**
 * @brief Set angle resolution and coupled scanning time. Successful config change will clear previously obtained data.
 *
 * @param angle_resolution
 * @return true if successful in setting angle resolution
 * @return false if failure to set angle resolution
 */
bool Laser::setAngleResolution(int angle_resolution)
{
  // check if valid parameter was parsed
  if ((angle_resolution == ANGLE_RESOLUTION_MODE_1) || (angle_resolution == ANGLE_RESOLUTION_MODE_2))
  {
    // check if parameter is actually changing
    if (angle_resolution_ != angle_resolution)
    {
      // clear data from previous configuration
      laser_data_.clear();

      // update last configuration change time
      last_config_change_ = timer.now();
    }

    // set valid parameter and coupled scanning time
    angle_resolution_ = angle_resolution;
    if (angle_resolution == ANGLE_RESOLUTION_MODE_1)
    {
      scanning_time_ = SCAN_TIME_MODE_1;
    }
    else
    {
      scanning_time_ = SCAN_TIME_MODE_2;
    }
  }

  return (angle_resolution_ == angle_resolution);
}

/**
 * @brief get time since last confguration change
 *
 * @return double time since configuration change in s
 */
double Laser::getTimeSinceConfigChange()
{
  std::chrono::duration<double> time = timer.now() - last_config_change_;
  return time.count();
}

/**
 * @brief get time since first data query
 *
 * @return double time since first dat query in s
 */
double Laser::getTimeSinceFirstQuery()
{
  std::chrono::duration<double> time = timer.now() - first_scan_time_;
  return time.count();
}

/**
 * @brief get total samples taken
 *
 * @return int total samples taken since construction of object
 */
int Laser::getSampleCount()
{
  return sample_counter_;
}

/**
 * @brief get intrinsic laser model no.
 *
 * @return std::string laser model no.
 */
std::string Laser::getModel()
{
  return MODEL;
}

/**
 * @brief get intrinisic laser output type
 *
 * @return std::string laser output type
 */
std::string Laser::getOutputType()
{
  return OUTPUT_TYPE;
}

/**
 * @brief get intrinsic field of view
 *
 * @return int laser field of view in degrees
 */
int Laser::getFieldOfView()
{
  return FIELD_OF_VIEW;
}

/**
 * @brief get intrinisc minimum range
 *
 * @return double minimum range in m
 */
double Laser::getMinRange()
{
  return MIN_RANGE;
}

/**
 * @brief get intrinsic maximum range
 *
 * @return double maximum range in m
 */
double Laser::getMaxRange()
{
  return MAX_RANGE;
}

/**
 * @brief generate random data point between min and max range of sensor
 * 
 * @return double random data point
 */
double Laser::generateRandomData()
{
  unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> randomiser(MIN_RANGE, MAX_RANGE);

  return randomiser(generator);
}