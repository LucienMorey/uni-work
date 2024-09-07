
#include "sample.h"
#include <tf/transform_datatypes.h>
#include <iostream>

/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry and Laser)
 * - Subscribing to images (which require a image transport interpreter)
 * - Publishing images
 * - The need to exchange data betweena seperate thread of execution and callbacks
 */

PfmsSample::PfmsSample(ros::NodeHandle nh) : nh_(nh), it_(nh)
{
  // Subscribing to odometry
  sub1_ = nh_.subscribe("robot_0/odom", 1000, &PfmsSample::odomCallback, this);
  // Subscribing to laser
  sub2_ = nh_.subscribe("base_scan_1", 10, &PfmsSample::laserCallback, this);

  // Subscribing to image
  // unlike other topics this requires a image transport rather than simply a node handle
  image_transport::ImageTransport it(nh);
  sub3_ = it.subscribe("map_image/full", 1, &PfmsSample::imageCallback, this);

  // Publishing an image ... just to show how
  image_pub_ = it_.advertise("test/image", 1);

  // Below is how to get parameters from command line, on command line they need to be _param:=value
  // For example _resolution:=0.1
  ros::NodeHandle pn("~");
  pn.param<double>("resolution", resolution_, 0.1);
}

PfmsSample::~PfmsSample()
{
  //    cv::destroyWindow("view");
}

// A callback for odometry
void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  /**
   * @todo - Ex 1 : Obtain a pose (x,y yaw) from nav_msgs/Odometry
   *
   * - On command line type 'rosmsg show nav_msgs/Odometry'
   * - The position and orientation are in two seperate parts of the message
   * - The orinetation is provided as a quaternion
   * - Which angle to we need?
   * - Ros has a 'tf' library with a helper function to get yaw from the quaternion
   * - http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
   * - Consider: Do we have nav_msgs::Odometry or q pointer to nav_msgs::Odometry ?
   * - Where is time of this message stored
   */

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  double yaw = tf::getYaw(msg->pose.pose.orientation);
  // printing values with standard out. comment has been commented in favour of ROS_INFO_STREAM
  // std::cout << "x: " << x << " y: " << y << " yaw " << yaw << std::endl;

  // Ros equivalent to std::cout is a ROS_INFO_STREAM
  ROS_INFO_STREAM("x: " << x << " y: " << y << " yaw " << yaw);
}

void PfmsSample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  /**
   * @todo - Ex 2 : Find the closest point [x,y] to the robot using sensor_msgs::LaserScan
   *
   * - On command line type 'rosmsg show sensor_msgs/LaserScan'
   * - What are we provided in this message?
   * - Do we have the inforamtion in this message to find the closest point?
   * - What part of the message do we need to iterate over?
   * - How do we convert from range data to [x,y] (this is known as polar to cartesian)
   * - https://www.mathsisfun.com/polar-cartesian-coordinates.html
   * - Where is time of this message stored?
   * - Is the closest point identified the same as the one you see as closest on the stage simulator?
   * - Why is this the case?
   */
}

void PfmsSample::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //! Below code converts from sensor message to a pointer to an opencv image and time to a deque, to share across
  //! threads
  try
  {
    if (enc::isColor(msg->encoding))
      cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
    else
      cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // To obtain image we use below
  cv::Mat image = cvPtr_->image;

  /**
   * @todo Ex 3 : Find the closest point [x,y] to the robot using the cv::Mat image
   *
   * - Where is time of this message stored?
   * - What information do we convert from the OgMap pixels to [x,y]
   * - Where is 0,0 on the image?
   *  o------> j
   *  |
   *  |
   *  v i
   * - Where is the robot on the OgMap?
   * - Is the closest point identified the same as the one you see as closest on the stage simulator?
   * - Why is this the case?
   */

  // An example of a loop examining row and col values
  //    for (int i = 0; i<image.rows; i++) {
  //        for (int j = 0; j <image.cols; j++) {
  //            unsigned char &pixel = image.at<unsigned char>(i, j);
  //            if (pixel > thresh) {
  //
  //            }
  //        }
  //    }
}

void PfmsSample::seperateThread()
{
  /**
   * The below loop runs until ros is shutdown, to ensure this thread does not remain
   * a zombie thread
   *
   */

  /// The below gets the current Ros Time
  ros::Time timeSample = ros::Time::now();

  //! What does this rate limiter do?
  ros::Rate rate_limiter(1.0);
  while (ros::ok())
  {
    /**
     * @todo Ex 4 : Find the closest point [x,y] to the robot using the robot pose and laser data
     *
     * - If we need to combine the information and use it in this seperate thread what do we need to consider
     * - Is the closest point identified the same as the one you see as closest on the stage simulator?
     * - Why is this the case?
     */

    /**
     * @todo Ex 5 : Mark the closest point on a RGB version of the OgMap image
     *
     * - If we need to combine the information and use it in this seperate thread what do we need to consider
     * - Is the closest point identified the same as the one you see as closest on the stage simulator?
     * - Why is this the case?
     */

    /*
     * The section is commented out initialy and can be used in Ex 5
      //If the image is not empty (which means it has data)
      if(!image.empty()){
        //Below code takes the cv::Mat image which is single channel (grayscale) and converts it into a RGB image
        cv_bridge::CvImage cv_image;
        cv::cvtColor(image,cv_image.image, CV_GRAY2RGB);
        // create a cv::Point at the centre of the image
        cv::Point pt1(image.cols/2,image.rows/2);
        // draw a circle using the point of size 3 pixels and in green color
        cv::circle(cv_image.image, pt1, 3, CV_RGB(0,255, 0) , 1);

        // To publish - send the image we need to specify the encoding and make a header
        cv_image.encoding = "bgr8";
        cv_image.header = std_msgs::Header();
        // We now publish the image and use the inbuilt ros function to convert cv_image to a image message
        image_pub_.publish(cv_image.toImageMsg());
      }
    */
    rate_limiter.sleep();
  }
}
