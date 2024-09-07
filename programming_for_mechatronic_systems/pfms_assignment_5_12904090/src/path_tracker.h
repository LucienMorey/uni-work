#ifndef PATHTRACKER_H
#define PATHTRACKER_H

#include "geometry_utility.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

class PathTracker
{
protected:
  const double POSE_LINEAR_TOLERANCE = 0.1;
  const double POSE_ANGULAR_TOLERANCE = 5.0 * M_PI / 180.0;
  const double MIN_LINEAR_VELOCITY = 0;
  const double MAX_LINEAR_VELOCITY = 0.5;
  const double MAX_ANGLE_VELOCITY = M_PI_2;

public:
  /**
   * @brief Construct a new Path Tracker object
   *
   */
  PathTracker(){};

  /**
   * @brief virtual declaration of track method
   *
   * @param current_pose [in] geometry_msgs::Pose current robot pose
   * @param initial_pose [in] geometry_msgs::Pose pose of the segment beginning
   * @param target_pose [in] geometry_msgs::Pose segment destination pose
   * @return geometry_msgs::Twist
   */
  virtual geometry_msgs::Twist track(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& initial_pose,
                                     const geometry_msgs::Pose& target_pose) = 0;

  /**
   * @brief use geometry utility function s to determine if current pose is
   * whein angular and linear tolerance of the desired pose
   *
   * @param current_pose [in] geometry_msgs::Pose current robot pose
   * @param goal_pose [in] geometry_msgs::Pose desired target pose
   * @return bool whether the curent pose is withen angular and linear tolerance
   * of the goal pose
   */
  bool completedPathSegment(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose);
};

#endif
