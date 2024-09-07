#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "a4_setup/RequestGoal.h"

namespace enc = sensor_msgs::image_encodings;

/**
 * This node shows some connections and publishing images
 */


class PfmsSample{

public:
  /*! @brief PfmsSample constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    PfmsSample(ros::NodeHandle nh);

  /*! @brief PfmsSample destructor.
   *
   *  Will tear down the object
   */
    ~PfmsSample();


  /*! @brief Request Goal service callback
   *
   *  @param req The requested goal.
   *  @param res The responce
   *
   *  @return bool - Will return true to indicate the request has sucsedded
   */
    bool requestGoal(a4_setup::RequestGoal::Request  &req,
             a4_setup::RequestGoal::Response &res);


  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

 /*! @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

 /*! @brief ImageConst Callback
   *
   *  @param sensor_msgs::ImageConstPtr - The imageconst message
   *  @note This function and the declaration are ROS specific
   */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /*! @brief seperate thread.
   *
   *  The main processing thread that will run continously and utilise the data
   *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
   */

    void seperateThread(); 

public:
    ros::NodeHandle nh_;

private:
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    image_transport::Subscriber sub3_;
    cv_bridge::CvImagePtr cvPtr_;
    ros::ServiceServer service_;

    int count_;//! A counter to allow executing items on N iterations
    double resolution_;//! size of OgMap in pixels

    //! Question: Is theer a better way to store data, instead of a structure?
    //! refer to Tutorial 7 exercise 3
    struct PoseDataBuffer
    {
      //! Question: Given these elements come in two's (pose and time)
      //! Is there a better type of STL container rather than two seperate deques?
        std::deque<geometry_msgs::Pose> poseDeq;
        std::deque<ros::Time> timeStampDeq;
        std::mutex mtx;
    };
    PoseDataBuffer poseDataBuffer_;//! Container for pose data

    struct ImageDataBuffer
    {
        //! Question: Given these elements come in two's (image and time)
        //! Is there a better type of STL container rather than two seperate deques?
        std::deque<cv::Mat> imageDeq;
        std::deque<ros::Time> timeStampDeq;
        std::mutex mtx;
    };

    ImageDataBuffer imageDataBuffer_;//! Container for image data


};

