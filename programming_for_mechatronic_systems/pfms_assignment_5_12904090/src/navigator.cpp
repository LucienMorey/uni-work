#include "navigator.h"

Navigator::Navigator()
{
  ros::NodeHandle nodehandle("~");
  // create odom subscriber
  odom_sub_ = nodehandle.subscribe("/robot_0/odom", 1000, &Navigator::odomCallback, this);

  // create path subscriber
  path_sub_ = nodehandle.subscribe("/path", 10, &Navigator::pathCallback, this);

  // create image subscriber
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nodehandle);
  map_sub_ = it.subscribe("/map_image/full", 10, &Navigator::imageCallback, this);

  // create laser subscriber
  laser_sub_ = nodehandle.subscribe("/robot_0/base_scan", 10, &Navigator::laserCallback, this);

  // get preferred pursuit algorithm param
  std::string pursuit_method;
  nodehandle.param<std::string>("pursuit_method", pursuit_method, "pointShoot");

  // transform param to uppercase
  std::transform(pursuit_method.begin(), pursuit_method.end(), pursuit_method.begin(), ::toupper);

  // create path tracker based on user input
  if (pursuit_method == "PUREPURSUIT")
  {
    ROS_INFO_STREAM("PurePursuit tracking selected");
    tracker = std::make_shared<PurePursuit>();
  }
  else
  {
    ROS_INFO_STREAM("PointShoot tracking selected");
    tracker = std::make_shared<PointShoot>();
  }

  // create velocity publisher
  cmd_vel_pub_ = nodehandle.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);
  // create annotated map publisher
  map_pub_ = it.advertise("/map_image/path_following", 1);

  // The below get's configurations from parameters server
  nodehandle.param("/local_map/map_resolution", resolution_, DEFAULT_MAP_RESOLUTION);

  // get the robot diameter from the parameter server
  nodehandle.param("robot_diameter", robot_diameter_, DEFAULT_ROBOT_DIAMETER);
}

Navigator::~Navigator()
{
}

void Navigator::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  geometry_msgs::Pose pose = msg->pose.pose;

  // store latest odometry value
  robotOdomDataBuffer_.mtx.lock();
  robotOdomDataBuffer_.odomDeq.push_back(*msg);

  // if the odom container grows beyond 2 values then pop the oldest data
  if (robotOdomDataBuffer_.odomDeq.size() > 2)
  {
    robotOdomDataBuffer_.odomDeq.pop_front();
  }
  robotOdomDataBuffer_.mtx.unlock();
}

void Navigator::pathCallback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  // alert user that the new path has been recieved
  ROS_INFO("New path received");
  ROS_INFO_STREAM("Received path has:" << msg->poses.size() << "elements");

  // clear existing path points for new data
  pathArrayBuffer_.mtx.lock();
  pathArrayBuffer_.poseDeque.clear();
  pathArrayBuffer_.completed_poses_.clear();

  // stop robot while processing path
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(cmd_vel);

  imageDataBuffer_.mtx.lock();
  // if there is no image to compare against then this doesn't work
  if ((imageDataBuffer_.imageDeq.size() > 0) && (msg->poses.size() > 0))
  {
    // check path size is greater than 0

    robotOdomDataBuffer_.mtx.lock();

    // insert the current robot position as the first checkpoint in the path to
    // join all links
    pathArrayBuffer_.poseDeque.push_back(robotOdomDataBuffer_.odomDeq.back().pose.pose);
    // pushback path points
    for (auto& elem : msg->poses)
    {
      pathArrayBuffer_.poseDeque.push_back(elem);
    }

    // reord vehicle pose at time of path
    pathArrayBuffer_.vehPose = robotOdomDataBuffer_.odomDeq.back().pose.pose;

    robotOdomDataBuffer_.mtx.unlock();

    pathValidater(pathArrayBuffer_.vehPose, pathArrayBuffer_.poseDeque, imageDataBuffer_.configSpaceDeque.back());
  }

  imageDataBuffer_.mtx.unlock();

  // alert user to the number of valid checkpoints
  if (pathArrayBuffer_.poseDeque.size() <= 1)
  {
    ROS_INFO_STREAM("Cannot Resolve Path");
  }
  else
  {
    ROS_INFO_STREAM("processed path has " << pathArrayBuffer_.poseDeque.size() - 1 << " elements");
  }

  pathArrayBuffer_.mtx.unlock();
}

void Navigator::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //! Below code pushes the image and time to a deque, to share across threads
  try
  {
    if (sensor_msgs::image_encodings::isColor(msg->encoding))
      cvPtr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    else
      cvPtr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // lock image data
  imageDataBuffer_.mtx.lock();

  // generate config space
  cv::Mat config_space = ImageProcessing::configTransfrom(cvPtr_->image, robot_diameter_ / 2, resolution_);

  // pushback new data and tiemstamp
  imageDataBuffer_.imageDeq.push_back(cvPtr_->image);
  imageDataBuffer_.timeStampDeq.push_back(msg->header.stamp);
  imageDataBuffer_.configSpaceDeque.push_back(config_space);

  // limit buffere size to only keep the latest data
  if (imageDataBuffer_.imageDeq.size() > 2)
  {
    imageDataBuffer_.imageDeq.pop_front();
    imageDataBuffer_.timeStampDeq.pop_front();
    imageDataBuffer_.configSpaceDeque.pop_front();
  }

  // unlock the image data
  imageDataBuffer_.mtx.unlock();
}

void Navigator::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  robotOdomDataBuffer_.mtx.lock();
  if (robotOdomDataBuffer_.odomDeq.size() > 0)
  {
    laserDataBuffer_.mtx.lock();

    // store lastest laser scan data
    laserDataBuffer_.laserDeque.push_back(*msg);
    laserDataBuffer_.interception_poses_deque.push_back(
        calculateStoppingPoses(laserDataBuffer_.laserDeque.back(), robotOdomDataBuffer_.odomDeq.back().pose.pose));

    // if more than 2 samples are stored then pop the oldest data
    if (laserDataBuffer_.laserDeque.size() > 2)
    {
      laserDataBuffer_.laserDeque.pop_front();
      laserDataBuffer_.interception_poses_deque.pop_front();
    }

    laserDataBuffer_.mtx.unlock();
  }
  robotOdomDataBuffer_.mtx.unlock();
}

void Navigator::controlThread()
{
  ros::Rate rate_limiter(100.0);
  geometry_msgs::Twist cmd_vel;

  while (ros::ok())
  {
    pathArrayBuffer_.mtx.lock();

    geometry_msgs::Pose segment_beginning;
    geometry_msgs::Pose segment_end;
    geometry_msgs::Pose current_pose;
    geometry_msgs::Twist current_twist;
    bool have_odom = false;

    // get current odom data
    robotOdomDataBuffer_.mtx.lock();
    if (robotOdomDataBuffer_.odomDeq.size() > 0)
    {
      current_pose = robotOdomDataBuffer_.odomDeq.back().pose.pose;
      current_twist = robotOdomDataBuffer_.odomDeq.back().twist.twist;
      have_odom = true;
    }
    robotOdomDataBuffer_.mtx.unlock();

    // if a path and odom data are available then it is possible to track points
    if ((pathArrayBuffer_.poseDeque.size() > 1) && (have_odom))
    {
      // get current segment beginning and segment goal
      segment_beginning = *(pathArrayBuffer_.poseDeque.begin());
      segment_end = *(pathArrayBuffer_.poseDeque.begin() + 1);

      //   track current goal pose
      cmd_vel = tracker->track(current_pose, segment_beginning, segment_end);

      laserDataBuffer_.mtx.lock();

      if (laserDataBuffer_.interception_poses_deque.size() > 0)
      {
        if (laserStop(current_pose, laserDataBuffer_.interception_poses_deque.back()))
        {
          cmd_vel.linear.x = 0.0;
        }
      }

      laserDataBuffer_.mtx.unlock();

      // publish velocity command
      cmd_vel_pub_.publish(cmd_vel);

      // if within tolerance of current goal pose
      if (tracker->completedPathSegment(current_pose, segment_end))
      {
        // pushback completed pose
        pathArrayBuffer_.completed_poses_.push_back(segment_end);
        // pop front pose so that we can begin tracking the next point
        pathArrayBuffer_.poseDeque.pop_front();
        ROS_INFO_STREAM("Completed Path Segment");
        imageDataBuffer_.mtx.lock();
        pathValidater(current_pose, pathArrayBuffer_.poseDeque, imageDataBuffer_.configSpaceDeque.back());
        imageDataBuffer_.mtx.unlock();
      }
    }
    else
    {
      // if a path and odom can't be obtained then wait for data arrival
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;

      // publish velocity
      cmd_vel_pub_.publish(cmd_vel);
    }

    imageDataBuffer_.mtx.lock();
    if (imageDataBuffer_.imageDeq.size() > 0)
    {
      // annotate progress to map
      cv::Mat map_with_progress = annotateMapProgress(imageDataBuffer_.imageDeq.back(), current_pose,
                                                      pathArrayBuffer_.poseDeque, pathArrayBuffer_.completed_poses_);

      // create image message and publish
      cv_bridge::CvImage image;
      image.image = map_with_progress;
      image.encoding = "bgr8";
      image.header = std_msgs::Header();
      map_pub_.publish(image.toImageMsg());
    }
    imageDataBuffer_.mtx.unlock();

    pathArrayBuffer_.mtx.unlock();
  }
}

void Navigator::annotateRemainingPath(const cv::Mat& current_map, cv::Mat& annotated_map,
                                      const geometry_msgs::Pose& current_pose,
                                      const std::deque<geometry_msgs::Pose>& remaining_poses)
{
  // the robot will be tracking a point as long as there are at least 2 elements
  // to make a path link
  if (remaining_poses.size() >= 2)
  {
    // draw a green cirle for the pose currently being tracked
    cv::circle(annotated_map,
               ImageProcessing::tfGlobalToImage(current_map, resolution_, current_pose, *(remaining_poses.begin() + 1)),
               CIRCLE_RADIUS, COLOUR_GREEN, CV_FILLED);
  }

  // There are points to be tracked as long as there is at least one other point
  // after the link being tracked
  if (remaining_poses.size() >= 3)
  {
    // for each point remaining draw a red circle
    for (auto deque_it = remaining_poses.begin() + 2; deque_it != remaining_poses.end(); ++deque_it)
    {
      cv::circle(annotated_map, ImageProcessing::tfGlobalToImage(current_map, resolution_, current_pose, *(deque_it)),
                 CIRCLE_RADIUS, COLOUR_RED, CV_FILLED);
    }
  }
}

void Navigator::annotateCompletedPath(const cv::Mat& current_map, cv::Mat& annotated_map,
                                      const geometry_msgs::Pose& current_pose,
                                      const std::vector<geometry_msgs::Pose>& completed_poses)
{
  // once a path point has been completed then it needs to be annotated on the
  // image
  if (completed_poses.size() > 0)
  {
    // for each completed pose draw a blue circle
    for (auto pose : completed_poses)
    {
      cv::circle(annotated_map, ImageProcessing::tfGlobalToImage(current_map, resolution_, current_pose, pose),
                 CIRCLE_RADIUS, COLOUR_BLUE, CV_FILLED);
    }
  }
}

std::vector<geometry_msgs::Pose> Navigator::calculateStoppingPoses(sensor_msgs::LaserScan laserscan,
                                                                   geometry_msgs::Pose robot_pose)
{
  std::vector<geometry_msgs::Pose> predicted_interception_positions;

  double reading_angle = tf::getYaw(robot_pose.orientation) - M_PI / 2;
  for (auto scan_data : laserscan.ranges)
  {
    geometry_msgs::Pose predicted_position;

    // determine x co-ordinate for predicted pose
    predicted_position.position.x = robot_pose.position.x + scan_data * cos(reading_angle);

    // determine y co-ordinate for predicted pose
    predicted_position.position.y = robot_pose.position.y + scan_data * sin(reading_angle);
    // pushback predicted pose
    predicted_interception_positions.push_back(predicted_position);

    // increment scan angle for next reading
    reading_angle += laserscan.angle_increment;
  }

  return predicted_interception_positions;
}

cv::Mat Navigator::annotateMapProgress(const cv::Mat& current_map, const geometry_msgs::Pose& current_pose,
                                       const std::deque<geometry_msgs::Pose>& remaining_poses,
                                       const std::vector<geometry_msgs::Pose>& completed_poses)
{
  // if map in expected format then annotate else return black image
  if (current_map.type() == CV_8UC1)
  {
    cv::Mat map_with_progress;
    // copy map and convert to colour from greyscale
    cv::cvtColor(current_map, map_with_progress, CV_GRAY2BGR);

    annotateCompletedPath(current_map, map_with_progress, current_pose, completed_poses);
    annotateRemainingPath(current_map, map_with_progress, current_pose, remaining_poses);

    return map_with_progress;
  }
  else
  {
    return cv::Mat::zeros(current_map.size(), CV_8UC3);
  }
}

bool Navigator::laserStop(const geometry_msgs::Pose& current_pose,
                          const std::vector<geometry_msgs::Pose>& stopping_poses)
{
  // if the error between the laser pose and current pose is less than the
  // robot radius then collision has occured and we should stop else
  // collision is iminent without blockage moving
  for (auto pose_it = stopping_poses.begin() + 1; pose_it != stopping_poses.end() - 1; ++pose_it)
  {
    if (GeometryUtility::getPositionError(*pose_it, current_pose) <= robot_diameter_ / 2)
    {
      ROS_INFO_STREAM("COLLISION IMINENT");
      return true;
    }
  }

  return false;
}

void Navigator::pathValidater(const geometry_msgs::Pose& current_pose, std::deque<geometry_msgs::Pose>& path,
                              const cv::Mat& config_space)
{
  // set path_connection state to false until it is proved to be complete
  bool path_connection = false;
  // while pose in position 1 is invalid remove it
  while ((!path_connection) && (path.size() > 1))
  {
    // create an iterator for the path
    auto current_checkpoint = *(path.begin());
    auto next_checkpoint = *(path.begin() + 1);

    // create point on image of current checkpoint
    cv::Point current_checkpoint_pixel =
        ImageProcessing::tfGlobalToImage(config_space, resolution_, current_pose, current_checkpoint);

    // create point on image of next checkpoint
    cv::Point next_checkpoint_pixel =
        ImageProcessing::tfGlobalToImage(config_space, resolution_, current_pose, next_checkpoint);

    // if they they do not connect then set bad checkpoint to be true and
    // break
    path_connection = ImageProcessing::checkConnectivity(config_space, current_checkpoint_pixel, next_checkpoint_pixel);

    // erase the pose at the bad checkpoint
    if (!path_connection)
    {
      path.erase(path.begin() + 1);
      ROS_INFO_STREAM("Erasing bad path checkpoint");
    }
  }
}