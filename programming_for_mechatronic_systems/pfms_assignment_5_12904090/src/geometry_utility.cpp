
#include "geometry_utility.h"

double GeometryUtility::getHeadingToTargetError(geometry_msgs::Pose target_pose, geometry_msgs::Pose current_pose)
{
  // calculate heading to target error
  double heading_to_target_error =
      atan2(target_pose.position.y - current_pose.position.y, target_pose.position.x - current_pose.position.x) -
      tf::getYaw(current_pose.orientation);

  // return constrained error
  return atan2(sin(heading_to_target_error), cos(heading_to_target_error));
}

double GeometryUtility::getPositionError(geometry_msgs::Pose target_pose, geometry_msgs::Pose current_pose)
{
  // return pythagorean distance between the two positions
  return pow((pow(current_pose.position.x - target_pose.position.x, 2) +
              pow(current_pose.position.y - target_pose.position.y, 2)),
             0.5);
}

double GeometryUtility::getHeadingToGoalError(geometry_msgs::Pose target_pose, geometry_msgs::Pose current_pose)
{
  // calculate heading to goal error
  double heading_to_goal_error = tf::getYaw(target_pose.orientation) - tf::getYaw(current_pose.orientation);
  // return constrained error
  return atan2(sin(heading_to_goal_error), cos(heading_to_goal_error));
}