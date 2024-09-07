#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <algorithm>
#include <deque>
#include <mutex>
#include <queue>
#include <string>

#include "ros/ros.h"

//! All the messages we need are here
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "image_processing.h"
#include "path_tracker.h"
#include "point_shoot.h"
#include "pure_pursuit.h"

class Navigator
{
private:
  cv_bridge::CvImagePtr cvPtr_;

  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber laser_sub_;
  image_transport::Subscriber map_sub_;

  image_transport::Publisher map_pub_;

  ros::Publisher cmd_vel_pub_;

  struct OdomDataBuffer
  {
    std::deque<nav_msgs::Odometry> odomDeq;
    std::mutex mtx;
  };

  //! Is theer a better way to store data, instead of a structure?
  //! refer to Tutorial 7 exercise 3
  struct PoseArrayBuffer
  {
    std::mutex mtx;                             //! mutex to lock data
    std::deque<geometry_msgs::Pose> poseDeque;  //! series of poses as PoseArray
    std::vector<geometry_msgs::Pose> completed_poses_;
    geometry_msgs::Pose vehPose;  //! the vehicle pose
  };

  struct ImageDataBuffer
  {
    //! Question: Given these elements come in two's (image and time)
    //! Is there a better type of STL container rather than two seperate deques?
    std::deque<cv::Mat> imageDeq;
    std::deque<cv::Mat> configSpaceDeque;
    std::deque<ros::Time> timeStampDeq;
    std::mutex mtx;
  };

  struct LaserDataBuffer
  {
    std::deque<sensor_msgs::LaserScan> laserDeque;
    std::deque<std::vector<geometry_msgs::Pose>> interception_poses_deque;
    std::mutex mtx;
  };

  ImageDataBuffer imageDataBuffer_;  //! Container for image data

  PoseArrayBuffer pathArrayBuffer_;  //! Path received via callback

  OdomDataBuffer robotOdomDataBuffer_;  //! Container for pose data

  LaserDataBuffer laserDataBuffer_;

  std::shared_ptr<PathTracker> tracker;

  double resolution_;
  double robot_diameter_;

  const double DEFAULT_MAP_RESOLUTION = 0.1;
  const double DEFAULT_ROBOT_DIAMETER = 0.4;

  const int CIRCLE_RADIUS = 1;
  const cv::Scalar COLOUR_BLUE = { 255, 0, 0 };
  const cv::Scalar COLOUR_GREEN = { 0, 255, 0 };
  const cv::Scalar COLOUR_RED = { 0, 0, 255 };

public:
  Navigator();
  ~Navigator();

  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  /*! @brief Path Callback
   *
   *  @param geometry_msgs::PoseArrayConstPtr - The path message
   *  @note This function and the declaration are ROS specific
   */
  void pathCallback(const geometry_msgs::PoseArrayConstPtr& msg);

  /*! @brief Image Callback
   *
   *  @param sensor_msgs::imageConstPtr - The image message
   *  @note This function and the declaration are ROS specific
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /*! @brief Laser Callback
   *
   *  @param sensor_msgs::LaserConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /**
   * @brief Control thread for navigator. handles tracking path to reach desired
   * Pose.
   * @note This function is designed to run with ROS and obtain data that is
   * processes in callbacks
   *
   */
  void controlThread();

  /**
   * @brief determine all potential interception poses in a global space based
   * on laser scan data
   *
   * @param laserscan [in] sensor_msgs::LaserScan - laserscan for calculating
   * collisions
   * @param robot_pose [in] geometry_msgs::Pose - current robot pose
   * @return std::vector<geometry_msgs::Pose> [out] vector containing all
   * possible interception poses
   */
  std::vector<geometry_msgs::Pose> calculateStoppingPoses(sensor_msgs::LaserScan laserscan,
                                                          geometry_msgs::Pose robot_pose);

  /**
   * @brief
   *
   * @param current_map [in] cv::Mat currrent og map as determined by incoming
   * laser data
   * @param current_pose [in] geometry_msgs::Pose current robot pose
   * @param remaining_poses [in] std::deque<geometry_msgs::Pose> deque
   * containing all remaining path poses
   * @param completed_poses [in] std::vector<geometry_msgs::Pose> vector
   * containing all completed dpath poses
   * @return cv::Mat RGB image with annotated poses
   */
  cv::Mat annotateMapProgress(const cv::Mat& current_map, const geometry_msgs::Pose& current_pose,
                              const std::deque<geometry_msgs::Pose>& remaining_poses,
                              const std::vector<geometry_msgs::Pose>& completed_poses);

  /**
   * @brief determine if current pose is within stopping distance of any
   * computed stopping pose
   *
   * @param robot_pose geometry_msgs::Pose current robot pose in a global space
   * @param stopping_poses std::vector<geometry_msgs::Poses> all interception
   * poses in a global space
   * @return bool whether the pose is within angularand linear stopping
   * tolerance
   */
  bool laserStop(const geometry_msgs::Pose& robot_pose, const std::vector<geometry_msgs::Pose>& stopping_poses);

  /**
   * @brief check path continuity for the immediate segment. if the segment
   * fails the test then the bad checkpoint is removed.this repeats until a
   * valid checkpoint is cheked or until there are no poses left for checking
   *
   * @param current_pose [in] geometry_msgs::Pose  current robot Pose in global
   * space
   * @param path [in/out] std::deque<geometry_msgs::Pose> deque of poses that
   * make up a robot path. all poses are in global space
   * @param config_space [in] c::Mat geycale image containing current robot
   * configuration space
   */
  void pathValidater(const geometry_msgs::Pose& current_pose, std::deque<geometry_msgs::Pose>& path,
                     const cv::Mat& config_space);

private:
  /**
   * @brief annotate the remaining path points to the map. THis includes the
   * current goal pose and any poses after.
   *
   * @param current_map [in] cv::Mat current local OG map around the robot
   * @param annotated_map [out] cv::Mat local ogmap around the robot that has
   * been rannotated top have sny remaining path points
   * @param current_pose [in] geometry_msgs::Pose Pose of robot in global space
   * @param remaining_poses[in] std::deque<geometry_msgs::Pose> deque of
   * poses that make up a robot path. all poses are in global space
   */
  void annotateRemainingPath(const cv::Mat& current_map, cv::Mat& annotated_map,
                             const geometry_msgs::Pose& current_pose,
                             const std::deque<geometry_msgs::Pose>& remaining_poses);

  /**
   * @brief annotate the completed path points onto an rgb map of the robot
   * space
   *
   * @param current_map [in] cv::Mat current local OG map around the robot
   * @param annotated_map [out] cv::Mat local ogmap around the robot that has
   * been rannotated top have sny remaining path points
   * @param current_pose [in] geometry_msgs::Pose Pose of robot in global space
   * @param completed_poses [in] std::vector<geometry_msgs::Pose> vector of
   * poses that have already been vistited sucessfully
   */
  void annotateCompletedPath(const cv::Mat& current_map, cv::Mat& annotated_map,
                             const geometry_msgs::Pose& current_pose,
                             const std::vector<geometry_msgs::Pose>& completed_poses);
};

#endif
