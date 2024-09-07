#include "image_processing.h"
#include <image_transport/image_transport.h>

bool ImageProcessing::checkConnectivity(cv::Mat image, cv::Point origin, cv::Point destination)
{
  // check if image is empty before trying test
  if (image.empty())
  {
    return false;
  }

  // assume connected until proven otherwise
  bool connectivity = true;

  //! Create a Line Iterator  between origib abd destination
  cv::LineIterator lineIterator(image, origin, destination);

  // iterate through the line. if any pixel is not white then a collision has
  // occured
  for (int i = 0; i < lineIterator.count; i++, lineIterator++)
  {
    uchar* pixel = (uchar*)*lineIterator;
    if (*pixel != COLOUR_WHITE)
    {
      connectivity = false;
    }
  }

  // return whether the line is connected or not
  return connectivity;
}

cv::Point ImageProcessing::tfGlobalToImage(cv::Mat image, double resolution, geometry_msgs::Pose robot_pose,
                                           geometry_msgs::Pose pose_for_transform)
{
  // determine the x-pixel value in the image
  int p_x = ((pose_for_transform.position.x - robot_pose.position.x) / resolution) + image.cols / 2;

  // determine the y pixel alue in the image
  int p_y = ((pose_for_transform.position.y - robot_pose.position.y) / -resolution) + image.rows / 2;

  // return a point object from the two pixel co ordiantes
  return cv::Point(p_x, p_y);
}

geometry_msgs::Pose ImageProcessing::tfImageToGlobal(cv::Mat image, double resolution, geometry_msgs::Pose robot_pose,
                                                     cv::Point point_for_transform)
{
  geometry_msgs::Pose transformed_pose;
  // global pose x co-ordinate
  transformed_pose.position.x = ((point_for_transform.x - image.cols / 2) * resolution) + robot_pose.position.x;
  // global pose y co-ordinate
  transformed_pose.position.y = ((point_for_transform.y - image.rows / 2) * -resolution) + robot_pose.position.y;

  // return global pose object
  return transformed_pose;
}

cv::Mat ImageProcessing::configTransfrom(cv::Mat og_map, double robot_radius, double map_resolution)
{
  // convert image to 8bit data dype
  og_map.convertTo(og_map, CV_8UC1);

  // threshold calues so that grey pixels become obstacles as well
  cv::threshold(og_map, og_map, COLOUR_GREY, COLOUR_WHITE, cv::THRESH_BINARY);
  cv::Mat distance_image;
  // transform pixel space into distanc espace
  cv::distanceTransform(og_map, distance_image, cv::DIST_L1, cv::DIST_MASK_3);
  cv::Mat config_space;
  // threshold distance space to obtain robot config space
  cv::threshold(distance_image, config_space, int(robot_radius / map_resolution), COLOUR_WHITE, cv::THRESH_BINARY);
  // convert back to 8 bit pixel space from floating point distance space
  config_space.convertTo(config_space, CV_8UC1);

  // draw circle around robot because it exists in space
  cv::circle(config_space, cv::Point(og_map.cols / 2, og_map.rows / 2), int(robot_radius / map_resolution),
             COLOUR_WHITE, CV_FILLED);

  return config_space;
}