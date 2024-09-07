#include "point_shoot.h"

PointShoot::PointShoot()
{
}

PointShoot::~PointShoot()
{
}

geometry_msgs::Twist PointShoot::track(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& initial_pose,
                                       const geometry_msgs::Pose& target_pose)
{
  // create command velocity message
  geometry_msgs::Twist cmd_vel;

  // calculate position error
  double pos_error = GeometryUtility::getPositionError(target_pose, current_pose);

  // if position error less than tolerance then rotate to goal orientation
  // else get to the appropriate position
  if (pos_error < POSE_LINEAR_TOLERANCE)
  {
    // rotate to goal pose
    cmd_vel.angular.z = GeometryUtility::getHeadingToGoalError(target_pose, current_pose) * ANG_KP;
    return cmd_vel;
  }
  else
  {
    // calculate heading to target error
    double heading_to_target_error = GeometryUtility::getHeadingToTargetError(target_pose, current_pose);

    // if not at position then heading error to target needs to be reduced then
    // drive forward to goal pos
    if (fabs(heading_to_target_error) < POSE_ANGULAR_TOLERANCE)
    {
      cmd_vel.linear.x = LIN_KP * pos_error;
      return cmd_vel;
    }
    else
    {
      cmd_vel.angular.z = ANG_KP * heading_to_target_error;
      return cmd_vel;
    }
  }
}