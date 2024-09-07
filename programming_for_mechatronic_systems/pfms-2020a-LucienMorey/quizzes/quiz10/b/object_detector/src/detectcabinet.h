#ifndef DETECTCABINET_H
#define DETECTCABINET_H

#include <sensor_msgs/LaserScan.h>

class DetectCabinet
{
public:
  DetectCabinet();
  bool detectCabinetPresence(sensor_msgs::LaserScan::ConstPtr laserScan);
};

#endif // DETECTCABINET_H
