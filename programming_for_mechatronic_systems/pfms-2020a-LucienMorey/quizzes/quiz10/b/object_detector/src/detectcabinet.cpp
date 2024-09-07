#include "detectcabinet.h"

DetectCabinet::DetectCabinet()
{
}

bool DetectCabinet::detectCabinetPresence(sensor_msgs::LaserScan::ConstPtr laserScan)
{
  /**
   * @todo - Ex 2 : Change teh code to return true if a hugh intensity reading is available
   *
   * - Loop goes through all elements
   * - Currently retruns false
   *
   */

  unsigned int highIntensityScans = 0;
  for (int elem = 0; elem < laserScan->ranges.size(); elem++)
  {
    if (laserScan->intensities.at(elem) > 0.5)
    {
      highIntensityScans++;
    }
  }

  return (highIntensityScans > 0);
}
