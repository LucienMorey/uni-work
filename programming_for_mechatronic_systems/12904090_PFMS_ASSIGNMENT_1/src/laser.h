#ifndef LASER_H
#define LASER_H

#include <string>
#include <chrono>
#include <vector>
#include <random>
#include <algorithm>

class Laser
{
public:
  Laser(double scanning_time = 0.02, int angle_resolution = 30);
  ~Laser();

  // configurable param getters
  double getScanningTime();
  int getAngleResolution();

  // timing getters
  double getTimeSinceConfigChange();
  double getTimeSinceFirstQuery();

  // sample counter getter
  int getSampleCount();

  // fixed param getters
  std::string getModel();
  std::string getOutputType();
  int getFieldOfView();
  double getMinRange();
  double getMaxRange();

  // configurable parm setters
  bool setScanningTime(double scanning_time);
  bool setAngleResolution(int angle_resolution);

  // data output interface
  std::vector<double> getData();

private:
  double generateRandomData();

  typedef std::chrono::steady_clock Time;

  // timing variables
  Time timer;
  Time::time_point last_config_change_;
  Time::time_point scan_timestamp_;
  Time::time_point first_scan_time_;

  // state member variables
  bool first_scan_;
  int sample_counter_;

  // configurable paramaters
  double scanning_time_;
  int angle_resolution_;

  // data storage container
  std::vector<double> laser_data_;

  // class constants
  const int MAX_HEADING = 270;
  const int MIN_HEADING = 0;

  const double SCAN_TIME_MODE_1 = 0.02;
  const int ANGLE_RESOLUTION_MODE_1 = 30;

  const double SCAN_TIME_MODE_2 = 0.06;
  const int ANGLE_RESOLUTION_MODE_2 = 10;

  // fixed parameters
  const double MAX_RANGE = 4.0;
  const double MIN_RANGE = 0.1;
  const int FIELD_OF_VIEW = 270;
  const std::string MODEL = "UTM-XL";
  const std::string OUTPUT_TYPE = "Laser Scan";
};

#endif