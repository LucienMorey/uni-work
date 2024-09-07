#ifndef GEOMETRYUTILITY_H
#define GEOMETRYUTILITY_H

#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"

class GeometryUtility
{
private:
  /* data */
public:
  /**
   * @brief utility function to calculate the angle error btween the current
   * robot heading and the heading to the desired goal position. the angle is
   * constrained and wrapped bwtween -pi and pi
   *
   * @param target_pose [in] current destination pose
   * @param current_pose [in] current robot pose
   * @return double [out] returns the heading error between the heading to the
   * goal position and the current heading
   */
  static double getHeadingToTargetError(geometry_msgs::Pose target_pose, geometry_msgs::Pose current_pose);
  /**
   * @brief utility function to calculate the position error between the current
   * pose and goal pose
   *
   * @param target_pose [in] global pose of target
   * @param current_pose [in] global pose of robot
   * @return double [out] distance bwtween current and target pose based on
   * pythagorean methods
   */
  static double getPositionError(geometry_msgs::Pose target_pose, geometry_msgs::Pose current_pose);
  /**
   * @brief utility function to calculate the angle error between the current
   * heading and desired goal heading. the angle is constrained and wrapped
   * between -pi and pi
   *
   * @param target_pose [in] global pose of target
   * @param current_pose [in] global pose of robot
   * @return double [out] the heading error between the goal heading and the
   * current heading
   */
  static double getHeadingToGoalError(geometry_msgs::Pose target_pose, geometry_msgs::Pose current_pose);
};

#endif