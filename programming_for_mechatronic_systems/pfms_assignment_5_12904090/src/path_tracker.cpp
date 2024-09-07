#include "path_tracker.h"

bool PathTracker::completedPathSegment(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose)
{
  // return if crrent pose is ithin angular and linear tolerance of desire pose
  return ((GeometryUtility::getPositionError(goal_pose, current_pose) <= POSE_LINEAR_TOLERANCE) &&
          (fabs(GeometryUtility::getHeadingToGoalError(goal_pose, current_pose)) <= POSE_ANGULAR_TOLERANCE));
}