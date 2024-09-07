#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "geometry_msgs/Point.h"
#include "geometry_utility.h"
#include "path_tracker.h"
#include <algorithm>

class PurePursuit : public PathTracker
{
private:
  struct GlobalOrd
  {
    double x; /*!< x coordinate within global map (m) */
    double y; /*!< y coordinate within global map (m) */
  };

  const double PURSUIT_ANGULAR_TOLERANCE = 45.0 * M_PI / 180.0;
  const double G = 9.81;
  const double MAX_G = MAX_LINEAR_VELOCITY * MAX_ANGLE_VELOCITY / G;
  const double LOOK_AHEAD_DISTANCE = 1.0;

  /**
   * @brief return the sign of a double precision floating point number
   *
   * @param num double - number for testing
   * @return double - will return 1.0 if the number tested is greater than or
   * equal to 0 else it returns -1.0
   */
  double sign(double num);

  /**
   * @brief calculate the point of intersection between a circle and line
   * segment. If there are two points of intersection the point furthest along
   * will be returned. If there are no points of interception an empty vector
   * will be returned.
   *
   * @param segment_begin [in] - geometry_msgs::Point, Point beginning of line
   * segment
   * @param segment_end [in] - geometry_msgs::Point, Point end of line segment
   * @param circle_origin [in] - geometry_msgs::Point, Point centre of circle
   * @param circle_radius [in] - double, radius of circle
   * @return [out] std::vector<geometry_msgs::Point> - vector containing point
   * of interception
   */
  std::vector<geometry_msgs::Point> lineCircleIntercept(geometry_msgs::Point segment_begin,
                                                        geometry_msgs::Point segment_end,
                                                        geometry_msgs::Point point_checking, double circle_radius);

  /**
   * @brief Determine the point along a line segment that is closest to a test
   * position. It will either return a point creating a perpendicular line from
   * the segment to the test point or will return either the beginning or end of
   * segment point.
   *
   * @param segment_begin [in] - geometry_msgs::Point, point beginning of
   * segment
   * @param segment_end [in] - geometry_msgs::Point, point end of segment
   * @param point_checking [in] - geometry_msgs::Point, point for checking
   * @return [out] geometry_msgs::Point, closest point along the line to the
   * point being cheked
   */
  geometry_msgs::Point closestPointAlongSegmentFromPoint(geometry_msgs::Point segment_begin,
                                                         geometry_msgs::Point segment_end,
                                                         geometry_msgs::Point point_checking);

public:
  /**
   * @brief Construct a new Pure Pursuit object
   *
   */
  PurePursuit(){};

  /**
   * @brief Destroy the Pure Pursuit object
   *
   */
  ~PurePursuit(){};

  /**
   * @brief Track goal pose parsed to function. The pure pursuit will rotate
   * until it meets a specified angle the nwill calculate and pursue a small
   * curvature to maximise linear velocity and minimise angular velocity to a
   * target. it requries an initial and goal pose to create a path. the current
   * pose and current velocity will assist and tracking a point along the path
   * until the end
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