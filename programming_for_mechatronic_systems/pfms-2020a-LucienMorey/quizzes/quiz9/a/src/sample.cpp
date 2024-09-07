
#include "sample.h"

/**
 * This node shows some connections and publishing images
 */

PfmsSample::PfmsSample(ros::NodeHandle nh) : nh_(nh), it_(nh)
{
  // Subscribing to odometry
  sub1_ = nh_.subscribe("robot_0/odom", 1000, &PfmsSample::odomCallback, this);

  image_transport::ImageTransport it(nh);
  // Subscribing to image
  sub3_ = it.subscribe("map_image/full", 1, &PfmsSample::imageCallback, this);

  // Publishing an image ... just to show how
  image_pub_ = it_.advertise("test/image", 1);

  // Allowing an incoming service on /check_goal
  service_ = nh_.advertiseService("check_goal", &PfmsSample::requestGoal, this);

  // Below ge't parameters from parameter server
  nh_.getParam("/local_map/map_resolution", resolution_);

  count_ = 0;
}

PfmsSample::~PfmsSample()
{
}

bool PfmsSample::requestGoal(a4_setup::RequestGoal::Request& req, a4_setup::RequestGoal::Response& res)
{
  // The incoming request is a Global coordinate.
  // To find information about the service execute "rossrv info a4_setup/RequestGoal"
  ROS_INFO_STREAM("requested goal [x,y,yaw]=[" << req.pose.x << "," << req.pose.x << "]");

  //! @todo : 4 - return true if the global coordinate supplied in the requst is within current OgMap
  //! (just needs to be within the map - either free / occupied or unknown)
  //!
  // Return in the res.ack
  // true : if the global coordinate is in the current OgMap
  // false : global coordinate outside of current OgMap

  // Informion you have now:
  // resolution_          - each pixel in OgMap corresponds to this many [m]
  // poseDataBuffer_      - position of Robot (refer code in seperateThread
  //                        on how to get x,y,yaw of robot
  // imageDataBuffer_     - ogMap, size can be accessed via image.rows and image.cols
  //                        (refer code in seperateThread on how to get ogMap

  imageDataBuffer_.mtx.lock();
  poseDataBuffer_.mtx.lock();

  if ((req.pose.y <
       (imageDataBuffer_.imageDeq.front().rows / 2) * resolution_ + poseDataBuffer_.poseDeq.front().position.y) &&
      (req.pose.y >
       -(imageDataBuffer_.imageDeq.front().rows / 2) * resolution_ + poseDataBuffer_.poseDeq.front().position.y) &&
      (req.pose.x <
       (imageDataBuffer_.imageDeq.front().cols / 2) * resolution_ + poseDataBuffer_.poseDeq.front().position.x) &&
      (req.pose.x >
       -(imageDataBuffer_.imageDeq.front().cols / 2) * resolution_ + poseDataBuffer_.poseDeq.front().position.x))
  {
    res.ack = true;
  }
  else
  {
    res.ack = false;
  }

  imageDataBuffer_.mtx.unlock();
  poseDataBuffer_.mtx.unlock();

  // res.ack = is the global coordinate in the current OgMap;
  ROS_INFO_STREAM("sending back response:" << res.ack);

  return true;
}

void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  //! Let's get the pose out from odometry message
  //! REMEBER: on command line you can view entier msg as
  //! rosmsg show nav_msgs/Odometry
  poseDataBuffer_.mtx.lock();
  poseDataBuffer_.poseDeq.push_back(msg->pose.pose);
  poseDataBuffer_.timeStampDeq.push_back(msg->header.stamp);
  if ((poseDataBuffer_.poseDeq.size() > 2))
  {
    poseDataBuffer_.poseDeq.pop_front();
    poseDataBuffer_.timeStampDeq.pop_front();
  }
  poseDataBuffer_.mtx.unlock();

  //! Additional question, what is managing this deque size? Should you?
}

void PfmsSample::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //! Below code pushes the image and time to a deque, to share across threads
  try
  {
    if (enc::isColor(msg->encoding))
      cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
    else
      cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  imageDataBuffer_.mtx.lock();
  imageDataBuffer_.imageDeq.push_back(cvPtr_->image);
  imageDataBuffer_.timeStampDeq.push_back(msg->header.stamp);
  // If the deque size is growing beyond 2 elements - remove the first element.
  if (imageDataBuffer_.imageDeq.size() > 2)
  {
    imageDataBuffer_.imageDeq.pop_front();
    imageDataBuffer_.timeStampDeq.pop_front();
  }
  imageDataBuffer_.mtx.unlock();
}

void PfmsSample::seperateThread()
{
  /**
   * The below loop runs until ros is shutdown, to ensure this thread does not remain
   * a zombie thread
   *
   * The loop locks the buffer, checks the size
   * And then pulls items: the pose and timer_t
   * You can contemplate weather these could have been combined into one ...
   */

  double yaw, x, y;
  /// The below gets the current Ros Time
  ros::Time timeOdom = ros::Time::now();
  ;
  ros::Time timeImage = ros::Time::now();
  ;
  cv::Mat image;

  //! @todo : 2 - Adjust rate limiter to run code every 5 seconds
  //!
  //! Additional Question
  //! What is rate_limiter?
  //!

  ros::Rate rate_limiter(5.0);
  while (ros::ok())
  {
    poseDataBuffer_.mtx.lock();
    if (poseDataBuffer_.poseDeq.size() > 0)
    {
      std::cout << "this ran " << std::endl;
      geometry_msgs::Pose pose = poseDataBuffer_.poseDeq.front();
      yaw = tf::getYaw(pose.orientation);
      x = pose.position.x;
      y = pose.position.y;
      timeOdom = poseDataBuffer_.timeStampDeq.front();
      poseDataBuffer_.poseDeq.pop_front();
      poseDataBuffer_.timeStampDeq.pop_front();
    }
    poseDataBuffer_.mtx.unlock();

    //! Lock image buffer, take one message, remove from deque and unlock it
    imageDataBuffer_.mtx.lock();
    if (imageDataBuffer_.imageDeq.size() > 0)
    {
      image = imageDataBuffer_.imageDeq.front();
      timeImage = imageDataBuffer_.timeStampDeq.front();
      imageDataBuffer_.imageDeq.pop_front();
      imageDataBuffer_.timeStampDeq.pop_front();
    }
    imageDataBuffer_.mtx.unlock();

    if (!image.empty())
    {
      cv_bridge::CvImage cv_image;
      // To draw a circle in colour we first convert the image (to )which in gray given it's an OgMap to colour)
      cv::cvtColor(image, cv_image.image, CV_GRAY2RGB);

      // o------> j
      // |
      // |
      // v i
      //

      //! @todo : 5 - Draw a circle at global location P(x,y)=(8.0,8.0) on the OgMap
      //!
      //! To do So:
      //! - Convert the global coordinate to pixel value in OgMap
      //! What is each pixel value in terms of [m]
      //! Where is (0,0) of the OgMap
      //! What pixel value is this?
      //! USE
      //! (image.cols, image.rows and resolution_ would be needed)

      int pixel_x = ((8.0 - x) / resolution_) + image.cols / 2;
      int pixel_y = ((8.0 - y) / -resolution_) + image.rows / 2;

      cv::Point pt(pixel_x, pixel_y);
      // cv::Point pt(image.cols / 2, image.rows / 2);
      cv::circle(cv_image.image, pt, 3, CV_RGB(0, 255, 0), 1);

      std::cout << "point x " << pixel_x << " robot x " << x << " point y " << pixel_y << " robot y " << y << std::endl;

      cv_image.encoding = "bgr8";
      cv_image.header = std_msgs::Header();
      image_pub_.publish(cv_image.toImageMsg());
    }
    rate_limiter.sleep();
  }
}
