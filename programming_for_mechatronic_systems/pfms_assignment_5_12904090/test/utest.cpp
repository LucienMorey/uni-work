#include <climits>
#include <gtest/gtest.h>

#include <ros/package.h>  //This tool allows to identify the path of the package on your system

#include "ros/ros.h"

#include <thread>

#include "nav_msgs/Odometry.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf/transform_datatypes.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "../src/geometry_utility.h"
#include "../src/image_processing.h"
#include "../src/navigator.h"
#include "../src/point_shoot.h"
#include "../src/pure_pursuit.h"

/**
 * @brief Test the transfrom from local pixel space to global co-ordinates.
 * Imports test image and tests to see if a specific pixel returns an expected
 * pose output
 *
 */
TEST(TransformTests, pixelToGlobal)
{
  std::string path = ros::package::getPath("pfms_assignment_5_12904090");

  path += "/test/samples/";
  // The file is called smiley_og.png
  std::string file = path + "tf_test_02.png";

  // robot is at (0,0)
  cv::Mat tf_test_image = cv::imread(file);

  // there is aa green pixel at 130,130 pixel this is equal to 0,0;
  cv::Point greenPixel = cv::Point(120, 130);
  cv::Vec3b pixel_colour = tf_test_image.at<cv::Vec3b>(greenPixel);
  ASSERT_EQ(pixel_colour, cv::Vec3b(0, 255, 0));

  geometry_msgs::Pose robot_pose, pixel_pose;
  robot_pose.position.x = -3.0;
  robot_pose.position.y = 3.0;

  // expected pose iss at -1.0 0.0;
  pixel_pose.position.x = -1.0;
  pixel_pose.position.y = 0.0;

  // green 120,130 is expected to be at -1.0, 0.0 given aa map resolution of
  // 0.1m
  geometry_msgs::Pose pose_point = ImageProcessing::tfImageToGlobal(tf_test_image, 0.1, robot_pose, greenPixel);

  ASSERT_EQ(pose_point, pixel_pose);
}

/**
 * @brief Test transform from globl to local pixel space. Imports an image and
 * asserts that a speicifc pixel value is overwirtten based on a robotpose and
 * global pose
 *
 */
TEST(TransformTests, globalToPixel)
{
  std::string path = ros::package::getPath("pfms_assignment_5_12904090");

  path += "/test/samples/";
  // The file is called smiley_og.png
  std::string file = path + "tf_test_01.png";

  // robot is at (0,0)
  cv::Mat tf_test_image = cv::imread(file);

  geometry_msgs::Pose robot_pose, pixel_pose;
  robot_pose.position.x = 0.0;
  robot_pose.position.y = 0.0;

  // expected pose is at 4.0 3.0;
  pixel_pose.position.x = 4.0;
  pixel_pose.position.y = 3.0;

  // for 4.0,3.0 the expected pixel is 140,70 for a map with resolution of 0.1m
  cv::Point pose_point = ImageProcessing::tfGlobalToImage(tf_test_image, 0.1, robot_pose, pixel_pose);

  // check that pixel is green
  ASSERT_EQ(tf_test_image.at<cv::Vec3b>(pose_point), cv::Vec3b(0, 255, 0));
}

/**
 * @brief Tests for position error between two points. Uses pythagoras' theorm
 * to determine distance then tests against it
 *
 */
TEST(GeometryUtilityTests, PositionError)
{
  geometry_msgs::Pose origin;
  origin.position.x = 0.0;
  origin.position.y = 0.0;

  geometry_msgs::Pose goal;
  goal.position.x = 4.0;
  goal.position.y = 3.0;

  // caculate distance between points
  double distance = GeometryUtility::getPositionError(goal, origin);

  // Assert that distance is returned
  EXPECT_DOUBLE_EQ(5.0, distance);

  origin.position.x = 0.0;
  origin.position.y = 0.0;

  goal.position.x = -4.0;
  goal.position.y = -3.0;

  // calculate distance betwen points
  distance = GeometryUtility::getPositionError(goal, origin);

  // Assert that absolute distance is returned
  EXPECT_DOUBLE_EQ(5.0, distance);
}

/**
 * @brief Test correct tranform occurs to config space of the robot. Reads test
 * image performs distance transorm to grow walls and draws circle around robot
 * location that is equal to robot size. reads second image and assets the two
 * are equal
 *
 */
TEST(ImageProcessingTests, ConfigTransform)
{
  //! The below code tests line connectivity
  //! On a loaded image
  std::string path = ros::package::getPath("pfms_assignment_5_12904090");

  path += "/test/samples/";
  // The file is called smiley_og.png
  std::string file = path + "line.png";

  cv::Mat image = cv::imread(file, 0);  // The zero forces it to a grayscale
                                        // image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols);  // Test if aquare image
  ASSERT_EQ(image.channels(), 1);     // Test if aingle channel

  cv::Mat test_img = ImageProcessing::configTransfrom(image, 0.5, 0.1);

  file = path + "expected_line.png";
  cv::Mat expected_image = cv::imread(file, 0);
  // annotate the robot onto the image
  cv::circle(expected_image, cv::Point(100, 100), int(0.5 / 0.1), 255, -1);

  test_img.convertTo(test_img, 0);

  // Subtract all pixel values to obtain final image
  cv::Mat dif = test_img - expected_image;

  // final image should contain entirely black pixels
  bool all0 = (cv::countNonZero(dif) == 0);

  ASSERT_TRUE(all0);
}

/**
 * @brief Coonfirm correct path progress annotation occurs. Asserts That the
 * correct number of coloured points are generated
 *
 */
TEST(CheckpointTests, PointsAnnotation)
{
  Navigator navigator;
  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = 0.0;
  robot_pose.position.y = 0.0;

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("pfms_assignment_5_12904090");
  // Now we have the path, the images for our testing are stored in a subfolder
  // /test/samples
  path += "/test/samples/";
  // The file is called smiley_og.png
  std::string file = path + "line.png";

  cv::Mat ogmap = cv::imread(file, 0);  // The zero forces it to a grayscale
                                        // image (single channel, such as OgMap)

  std::deque<geometry_msgs::Pose> remaining_checkpoints;
  // pushabck robot path
  geometry_msgs::Pose path_point;
  path_point.position.x = 2.0;
  path_point.position.y = 0.0;
  remaining_checkpoints.push_back(robot_pose);
  remaining_checkpoints.push_back(path_point);

  path_point.position.x = 5.0;
  path_point.position.y = 5.0;
  remaining_checkpoints.push_back(path_point);
  path_point.position.x = 4.0;
  path_point.position.y = 4.0;
  remaining_checkpoints.push_back(path_point);

  std::vector<geometry_msgs::Pose> completed_checkpoints;
  path_point.position.x = -4.0;
  path_point.position.y = 4.0;
  completed_checkpoints.push_back(path_point);
  path_point.position.x = -4.0;
  path_point.position.y = -4.0;
  completed_checkpoints.push_back(path_point);
  path_point.position.x = -4.0;
  path_point.position.y = 0.0;
  completed_checkpoints.push_back(path_point);

  // annotate progress onto map mage
  cv::Mat image;
  image = navigator.annotateMapProgress(ogmap, robot_pose, remaining_checkpoints, completed_checkpoints);

  // thereshold against green pixels and determine that the correct number of
  // currently tracked points is obtained
  cv::Mat green_thresholded_image;
  cv::inRange(image, cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 0), green_thresholded_image);

  cv::Mat green_expected[3];
  cv::split(green_thresholded_image, green_expected);
  bool green_all0 = (cv::countNonZero(green_expected[0]) == 5);
  ASSERT_TRUE(green_all0);

  // threshold aginst all blue pixels and determine that the correct number of
  // completed path points is obtained
  cv::Mat blue_thresholded_image;
  cv::inRange(image, cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 0), blue_thresholded_image);

  cv::Mat blue_expected[3];
  cv::split(blue_thresholded_image, blue_expected);
  bool blue_all0 = (cv::countNonZero(blue_expected[0]) == 15);
  ASSERT_TRUE(blue_all0);

  // threshold against all red pixels and determine that the correct number of
  // to be attempted path points is obtained
  cv::Mat red_thresholded_image;
  cv::inRange(image, cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 255), red_thresholded_image);

  cv::Mat red_expected[3];
  cv::split(red_thresholded_image, red_expected);
  bool red_all0 = (cv::countNonZero(red_expected[0]) == 10);
  ASSERT_TRUE(red_all0);
}

/**
 * @brief Tests Path prgress annotation. Test to make sure pno points are
 * annotated when conditions are not met. Imports an image and Asserts that the
 * image remains the same after point annotation
 *
 */

TEST(CheckpointTests, NoAnnotation)
{
  Navigator navigator;
  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = 0.0;
  robot_pose.position.y = 0.0;

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("pfms_assignment_5_12904090");
  // Now we have the path, the images for our testing are stored in a subfolder
  // /test/samples
  path += "/test/samples/";
  // The file is called smiley_og.png
  std::string file = path + "line.png";

  cv::Mat ogmap = cv::imread(file, 0);  // The zero forces it to a grayscale
  // image (single channel, such as OgMap)

  std::deque<geometry_msgs::Pose> remaining_checkpoints;
  std::vector<geometry_msgs::Pose> completed_checkpoints;

  // annotate map progress onto image
  cv::Mat output;
  output = navigator.annotateMapProgress(ogmap, robot_pose, remaining_checkpoints, completed_checkpoints);

  cv::cvtColor(ogmap, ogmap, CV_GRAY2BGR);

  // check image hasnt changed because there are no checkpoints to annotate
  cv::Mat expected = ogmap - output;
  cv::Mat channels[3];
  cv::split(expected, channels);
  for (int i = 0; i < 3; i++)
  {
    bool all0 = (cv::countNonZero(channels[i]) == 0);
    ASSERT_TRUE(all0);
  }
}

/**
 * @brief Path validation test. Asserts that a non broken path segment doesnt
 * get removed. Checks sie of path and that the goal pose remains the same
 *
 */
TEST(MapTests, completePath)
{
  std::string image_path = ros::package::getPath("pfms_assignment_5_12904090");

  cv::Mat og_map = cv::imread(image_path += "/test/samples/og_clear.png", 0);

  cv::Mat config_space = ImageProcessing::configTransfrom(og_map, 0.2, 0.1);

  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = 0.0;
  robot_pose.position.y = 0.0;

  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = 4.0;
  goal_pose.position.y = 0.0;

  // create path segment
  std::deque<geometry_msgs::Pose> path;
  path.push_back(robot_pose);
  path.push_back(goal_pose);

  // validate path segment
  Navigator navigator;
  navigator.pathValidater(robot_pose, path, config_space);

  // assert path hasnt changed
  ASSERT_EQ(path.size(), 2);

  ASSERT_EQ(goal_pose.position.x, path.at(1).position.x);
  ASSERT_EQ(goal_pose.position.y, path.at(1).position.y);
}

/**
 * @brief Path validation test. function is provided a broken path and is
 * expected to remove the bad checkpoint. Asserts that the size is altered and
 * that the next good checkpoint is the first goal
 *
 */
TEST(MapTests, pathWithBadCheckpoint)
{
  std::string image_path = ros::package::getPath("pfms_assignment_5_12904090");

  cv::Mat og_map = cv::imread(image_path += "/test/samples/og_blocked.png", 0);

  cv::Mat config_space = ImageProcessing::configTransfrom(og_map, 0.2, 0.1);

  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = 0.0;
  robot_pose.position.y = 0.0;

  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = 4.0;
  goal_pose.position.y = 0.0;

  // create pah with bad checkpoint
  std::deque<geometry_msgs::Pose> path;
  path.push_back(robot_pose);
  path.push_back(goal_pose);
  goal_pose.position.x = 0.0;
  goal_pose.position.y = 1.0;
  path.push_back(goal_pose);

  // validate path
  Navigator navigator;
  navigator.pathValidater(robot_pose, path, config_space);

  // assert path size after removeing bad checkpoint
  ASSERT_EQ(path.size(), 2);

  // assert correct path point after validation
  ASSERT_EQ(goal_pose.position.x, path.at(1).position.x);
  ASSERT_EQ(goal_pose.position.y, path.at(1).position.y);
}

/**
 * @brief Laser blockage path test. The laser reading records no adjacetn
 * objects. It is asserted that there is no blockage
 *
 */
TEST(LaserMapTests, laserMapFree)
{
  rosbag::Bag bag;

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("pfms_assignment_5_12904090");
  // Now we have the path, the images for our testing are stored in a subfolder
  // /test/samples

  // get sensor messages from ros bag
  bag.open(path + "/test/samples/og_clear_laser_clear.bag");
  sensor_msgs::LaserScanConstPtr laser_scan;
  nav_msgs::OdometryConstPtr robot_odom;
  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    laser_scan = m.instantiate<sensor_msgs::LaserScan>();

    if ((laser_scan != nullptr))
    {
      break;
    }
  }

  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    robot_odom = m.instantiate<nav_msgs::Odometry>();

    if ((robot_odom != nullptr))
    {
      break;
    }
  }

  bag.close();

  ASSERT_NE(laser_scan, nullptr);
  ASSERT_NE(robot_odom, nullptr);

  Navigator navigator;

  // calcualte all laser stopping poses based on laserscan
  std::vector<geometry_msgs::Pose> stopping_poses =
      navigator.calculateStoppingPoses(*laser_scan, (*robot_odom).pose.pose);
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 4.0;
  target_pose.position.y = 0.0;
  // determine if stopping is required
  bool stop_robot = navigator.laserStop((*robot_odom).pose.pose, stopping_poses);

  // assert stopping not required
  ASSERT_FALSE(stop_robot);
}

/**
 * @brief Laser path blockage test. robot path with object blocking checkpoint.
 * Assert that no stopage should occur because the robot is out of range of
 * object
 *
 */
TEST(LaserMapTests, mapFreeLaserBlocked)
{
  rosbag::Bag bag;

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("pfms_assignment_5_12904090");
  // Now we have the path, the images for our testing are stored in a subfolder
  // /test/samples

  // get sensor data from rosbag
  bag.open(path + "/test/samples/og_clear_laser_blocked.bag");
  sensor_msgs::LaserScanConstPtr laser_scan;
  nav_msgs::OdometryConstPtr robot_odom;
  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    laser_scan = m.instantiate<sensor_msgs::LaserScan>();

    if ((laser_scan != nullptr))
    {
      break;
    }
  }

  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    robot_odom = m.instantiate<nav_msgs::Odometry>();

    if ((robot_odom != nullptr))
    {
      break;
    }
  }

  bag.close();

  ASSERT_NE(laser_scan, nullptr);
  ASSERT_NE(robot_odom, nullptr);

  Navigator navigator;
  // calculate all stopping poses based on laser scan
  std::vector<geometry_msgs::Pose> stopping_poses =
      navigator.calculateStoppingPoses(*laser_scan, (*robot_odom).pose.pose);
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 4.0;
  target_pose.position.y = 0.0;
  // check if robot should be stopping
  bool stop_robot = navigator.laserStop((*robot_odom).pose.pose, stopping_poses);

  ASSERT_FALSE(stop_robot);
}

/**
 * @brief Laser path blockage test. robot path with object not blocking point
 * bu tinterfering with config space. Assert that no stopage should occur
 * because the robot is out of range of object
 *
 */
TEST(LaserMapTests, mapFreeLaserBlockedPartial)
{
  rosbag::Bag bag;

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("pfms_assignment_5_12904090");
  // Now we have the path, the images for our testing are stored in a subfolder
  // /test/samples

  // get sensor data from ros bag
  bag.open(path + "/test/samples/og_clear_laser_blocked_partial.bag");
  sensor_msgs::LaserScanConstPtr laser_scan;
  nav_msgs::OdometryConstPtr robot_odom;
  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    laser_scan = m.instantiate<sensor_msgs::LaserScan>();

    if ((laser_scan != nullptr))
    {
      break;
    }
  }

  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    robot_odom = m.instantiate<nav_msgs::Odometry>();

    if ((robot_odom != nullptr))
    {
      break;
    }
  }

  bag.close();

  ASSERT_NE(laser_scan, nullptr);
  ASSERT_NE(robot_odom, nullptr);

  Navigator navigator;

  // calculate all stopping poses for robot
  std::vector<geometry_msgs::Pose> stopping_poses =
      navigator.calculateStoppingPoses(*laser_scan, (*robot_odom).pose.pose);
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 4.0;
  target_pose.position.y = 0.0;
  // check if robot should be stopping or not
  bool stop_robot = navigator.laserStop((*robot_odom).pose.pose, stopping_poses);

  ASSERT_FALSE(stop_robot);
}

/**
 * @brief Laser path blockage test. robot path with object blocking checkpoint.
 * Assert that the robot should stop because object is within crictical distance
 * of the robot
 *
 */
TEST(LaserMapTests, mapFreeLaserCritical)
{
  rosbag::Bag bag;

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("pfms_assignment_5_12904090");
  // Now we have the path, the images for our testing are stored in a subfolder
  // /test/samples

  // get sensor data from bag file
  bag.open(path + "/test/samples/og_clear_laser_critical.bag");
  sensor_msgs::LaserScanConstPtr laser_scan;
  nav_msgs::OdometryConstPtr robot_odom;
  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    laser_scan = m.instantiate<sensor_msgs::LaserScan>();

    if ((laser_scan != nullptr))
    {
      break;
    }
  }

  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    robot_odom = m.instantiate<nav_msgs::Odometry>();

    if ((robot_odom != nullptr))
    {
      break;
    }
  }

  bag.close();

  ASSERT_NE(laser_scan, nullptr);
  ASSERT_NE(robot_odom, nullptr);

  Navigator navigator;

  // calculate all current stopping poses
  std::vector<geometry_msgs::Pose> stopping_poses =
      navigator.calculateStoppingPoses(*laser_scan, (*robot_odom).pose.pose);
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 4.0;
  target_pose.position.y = 0.0;

  // check if robot stop is required
  bool stop_robot = navigator.laserStop((*robot_odom).pose.pose, stopping_poses);

  ASSERT_TRUE(stop_robot);
}

/**
 * @brief Laser path blockage test. robot path with object obstructing config
 * space. Assert that the robot should stop because object is within crictical
 * distance of the robot
 *
 */
TEST(LaserMapTests, mapFreeLaserCriticalPartial)
{
  rosbag::Bag bag;

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("pfms_assignment_5_12904090");
  // Now we have the path, the images for our testing are stored in a subfolder
  // /test/samples

  // get sensor data from robot
  bag.open(path + "/test/samples/og_clear_laser_critical_partial.bag");
  sensor_msgs::LaserScanConstPtr laser_scan;
  nav_msgs::OdometryConstPtr robot_odom;
  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    laser_scan = m.instantiate<sensor_msgs::LaserScan>();

    if ((laser_scan != nullptr))
    {
      break;
    }
  }

  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    robot_odom = m.instantiate<nav_msgs::Odometry>();

    if ((robot_odom != nullptr))
    {
      break;
    }
  }

  bag.close();

  ASSERT_NE(laser_scan, nullptr);
  ASSERT_NE(robot_odom, nullptr);

  Navigator navigator;

  // calculate all stopping poses fromlaser scan
  std::vector<geometry_msgs::Pose> stopping_poses =
      navigator.calculateStoppingPoses(*laser_scan, (*robot_odom).pose.pose);
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 4.0;
  target_pose.position.y = 0.0;

  // check if laser stop is required
  bool stop_robot = navigator.laserStop((*robot_odom).pose.pose, stopping_poses);

  ASSERT_TRUE(stop_robot);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "NavigatorTestNode");
  testing::InitGoogleTest(&argc, argv);

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto result = RUN_ALL_TESTS();

  ros::shutdown();

  t.join();
  return result;
}
