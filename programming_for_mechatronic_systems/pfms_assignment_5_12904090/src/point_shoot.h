#ifndef POINTSHOOT_H
#define POINTSHOOT_H

#include "geometry_utility.h"
#include "path_tracker.h"

class PointShoot : public PathTracker
{
private:
  /* data */
  const double ANG_KP = 1.0;
  const double LIN_KP = 1.0;

public:
  /**
   * @brief Construct a new Point Shoot object
   *
   */
  PointShoot();
  /**
   * @brief Destroy the Point Shoot object
   *
   */
  ~PointShoot();
  /**
   * @brief Track goal pose parsed to function. The point shoot metod will
   * rotate until in angular tolerance to nnext target then move at ful speed
   * until approaching the target and slow down. once the desired position has
   * been reached it will rotate to the specified heading
   *
   * @param current_pose [in] - Pose, current pose of robot
   * @param initial_pose [in] - Pose, initial pose of robot along the path
   * segment
   * @param target_pose [in] - Pose, final Desired Pose.
   * @return Twist - twist obejct containing next velocity command
   */
  geometry_msgs::Twist track(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& initial_pose,
                             const geometry_msgs::Pose& target_pose);
};

#endif