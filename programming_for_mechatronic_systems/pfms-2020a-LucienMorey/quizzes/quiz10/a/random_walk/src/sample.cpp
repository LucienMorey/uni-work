
#include "sample.h"

/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry and Laser)
 * - Publishing velocity commands
 */

RandomWalk::RandomWalk(ros::NodeHandle nh) : nh_(nh)
{
  sub2_ = nh_.subscribe("/scan", 10, &RandomWalk::laserCallback, this);

  // Publish a velocity ... to control the robot
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

RandomWalk::~RandomWalk()
{
}

void RandomWalk::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  //! Let's find closest on left and closest on right of laser and then steer based on this

  // We start of with min right and min left at max range (so we can find the
  // minima easier - is should be below this.
  float minR = msg->range_max;  // minimum distance on Right
  float minL = msg->range_max;  // minimum distance on Left

  for (unsigned int i = 0; i <= msg->ranges.size() / 2; i++)
  {
    /**
     * @todo - TASK 4 : Find the closest point range reading on the right side of the robot and assign it to minR;
     *
     * - This loop iterates over the laser readings (from 0 to half the range readings)
     * - Determine the smallest range reading (which is the closest) and assign it to minR
     */
    if (msg->ranges.at(i) < minR)
    {
      minR = msg->ranges.at(i);
    }
  }

  for (unsigned int i = (msg->ranges.size() / 2); i < msg->ranges.size(); i++)
  {
    /**
     * @todo - TASK 4 : Find the closest point range reading on the left side of the robot and assign it to minL;
     *
     * - This loop iterates over the laser readings (from half the range readings to the full readings)
     * - Determine the smallest range reading (which is the closest) and assign it to minl
     */

    if (msg->ranges.at(i) < minL)
    {
      minL = msg->ranges.at(i);
    }
  }

  //! Now we use some logic to determine a velocity and turn rate
  //! Borrowed from
  //! https://github.com/Arkapravo/Player-3.0.2/blob/master/examples/libplayerc%2B%2B/laserobstacleavoid.cc This will
  //! work unless we have objects of equal distance on both sides (we head to a deadlock)
  double l = (1e5 * minR) / 500 - 100;
  double r = (1e5 * minL) / 500 - 100;

  if (l > 100)
    l = 100;
  if (r > 100)
    r = 100;

  //! Speed is proportional to distance to closest object (so it slows down when getting closer to objects)
  double newspeed = (r + l) / 1e3;

  //! Angular velocity (turn rate) proportional to difference between left/righ closest objet
  double newturnrate = (r - l);

  //! Let's clip turn rate to -40 to 40 deg/s
  if (newturnrate > 40.0)
  {
    newturnrate = 40.0;
  }
  else if (newturnrate < -40.0)
  {
    newturnrate = -40.0;
  }

  newturnrate = newturnrate * M_PI / 180.0;  // turn into radians

  // write commands to robot
  geometry_msgs::Twist control;     // container for the control system
  control.linear.x = newspeed;      // linear velocity of the control system
  control.angular.z = newturnrate;  // angular velocity of the control system
  cmd_vel_pub_.publish(control);    // publishing the control system
}
