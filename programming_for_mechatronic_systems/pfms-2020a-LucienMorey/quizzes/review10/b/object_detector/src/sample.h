#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"


/**
 * This node shows some connections and publishing images
 */


class ObjectDetector{

public:
  /*! @brief ObjectDetector constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    ObjectDetector(ros::NodeHandle nh);

  /*! @brief ObjectDetector destructor.
   *
   *  Will tear down the object
   */
    ~ObjectDetector();


 /*! @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);


public:
    ros::NodeHandle nh_;

private:
    ros::Subscriber sub1_;
};

