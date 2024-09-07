#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include "geometry_msgs/Pose.h"
#include <opencv2/opencv.hpp>

class ImageProcessing
{
private:
  // value of gray in an unsigned 8bit image type
  static const uchar COLOUR_GREY = 128;

  // value of white in an unsigned 8bit image
  static const uchar COLOUR_WHITE = 255;  //! value of white in an unsigned 8bit image type

public:
  /**
   * @brief Checks whether the origin and destination can be connected with a
   * line, such that line only goes over free space
   *
   * @param image [in] cv::Mat image - image array to perform operations on
   * @param origin [in] cv::Point origin - beginning of line segment to be
   * tested
   * @param destination [in] cv::Point destination 0 end of line segment to be
   * tested
   * @return bool [out] boolean of whether the lines are connected without
   * blockage
   */
  static bool checkConnectivity(cv::Mat image, cv::Point origin, cv::Point destination);

  /**
   * @brief transform of a global pose into a local pixelspace surrounding the
   * robot
   *
   * @param image [in] cv::Mat - image array for operations to be performed on
   * @param resolution [in] double - resolution of distance per pixel in the
   * image
   * @param robotPose [in] geometry_msgs::Pose - current robot pose in global
   * space
   * @param PoseforTransform [in] geometry_msgs::Pose - global Pose to be
   * transformed into pixel space
   * @return cv::Point [out] point transformed into local pixel space
   */
  static cv::Point tfGlobalToImage(cv::Mat image, double resolution, geometry_msgs::Pose robotPose,
                                   geometry_msgs::Pose PoseforTransform);

  /**
   * @brief transform of a local pixel into global space
   *
   * @param image [in] cv::Mat - image array for operations to be performed on
   * @param resolution [in] double - resolution of distance per pixel in the
   * image
   * @param robotPose [in] geometry_msgs::Pose - current robot pose in global
   * space
   * @param PointforTransform [in] cv::Point - local point to be transformed
   * into a global space
   * @return geometry_msgs::Pose [out] global pose of pixel after transform
   */
  static geometry_msgs::Pose tfImageToGlobal(cv::Mat image, double resolution, geometry_msgs::Pose robotPose,
                                             cv::Point PointforTransform);
  /**
   * @brief transform og map into config space for a circular robot
   *
   * @param og_map [in] cv::Mat - image array containing occupancy grid map for
   * transform
   * @param robot_radius [in] double - robot radius
   * @param map_resolution [in] double - resolution of distance per pixel in the
   * image
   * @return cv::Mat [out] transormed image with wlls artificially grown by the
   * radius of the robot
   */
  static cv::Mat configTransfrom(cv::Mat og_map, double robot_radius, double map_resolution);
};

#endif  // IMAGE_PROCESSING_H
