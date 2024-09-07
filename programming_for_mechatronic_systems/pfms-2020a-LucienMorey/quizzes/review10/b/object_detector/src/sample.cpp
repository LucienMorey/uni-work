
#include "sample.h"
#include "detectcabinet.h"

/**
 * This sample code is provided to illustrate
 * - Subscribing to topics (Laser)
 * - Looking at high intensity readings
 */


ObjectDetector::ObjectDetector(ros::NodeHandle nh)
    : nh_(nh)
{
    sub1_ = nh_.subscribe("/robot_0/base_scan", 10, &ObjectDetector::laserCallback,this);


}

ObjectDetector::~ObjectDetector()
{

}


void ObjectDetector::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

  /**
   * @todo - Ex 4 : Make this callback use the detectCabinet member function of DetectCabinet
   *
   * - Where do we create an instance of this object?
   * - Use ROS_INFO_STREAM to display if a cabinet is detected
   *
   * - CONSIDER: If this process took a significant amount of time, shoudl it be in callback?
   */
//  std::shared_ptr<DetectCabinet> cab(new DetectCabinet);

  DetectCabinet dc;
  ROS_INFO(dc.detectCabinetPresence(msg) ? "Cabinet Detected" : "Cabinet Not Detected");

}


