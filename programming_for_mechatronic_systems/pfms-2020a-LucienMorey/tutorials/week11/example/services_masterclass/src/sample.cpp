
#include "sample.h"


/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry and Laser)
 * - Subscribing to images (which require a image transport interpreter)
 * - Publishing images
 *
 * The code is not intended to provide a
 */


PfmsSample::PfmsSample(ros::NodeHandle nh)
    : nh_(nh), it_(nh), imageProcessing_()
{
  //Subscribing to odometry
  sub1_ = nh_.subscribe("/robot_0/odom", 1000, &PfmsSample::odomCallback,this);

  //Subscribing to image
  // unlike other topics this requires a image transport rather than simply a node handle
  image_transport::ImageTransport it(nh);
  sub2_ = it.subscribe("map_image/full", 1, &PfmsSample::imageCallback,this);

  //Publish a velocity ... to control the robot
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",1);

  //Allowing an incoming service on /face_goal, this invokes the faceGoal function
  service_ = nh_.advertiseService("face_goal", &PfmsSample::faceGoal,this);

  //The below get's configurations from parameters server
  nh_.getParam("/local_map/map_resolution", resolution_);

  //Below is how to get parameters from command line, on command line they need to be _param:=value
  //For example _example:=0.1
  //Below will either get the configuration from command line or assign a default value 0.1
  ros::NodeHandle pn("~");
  double example;
  pn.param<double>("example", example, 0.1);

}

PfmsSample::~PfmsSample()
{
//    cv::destroyWindow("view");
}


bool PfmsSample::faceGoal(a4_setup::RequestGoal::Request  &req,
             a4_setup::RequestGoal::Response &res)
{
  //When an incoming call arrives, we can respond to it here
  ROS_INFO_STREAM("request: [x,y]=[" << req.pose.x << "," << req.pose.y);

  res.ack=false;

  //! @todo Ex04 : Adjust the code so it returns a `true` in the acknowledgment of the service call if the point
  //! can be reached in a straight line from current robot pose, only traversing free space.

  //! Below code will be useful
  //! Lock image buffer, take one message from deque and unlock it
  cv::Mat image;
  imageDataBuffer_.mtx.lock();
  if(imageDataBuffer_.imageDeq.size()>0){
      image = imageDataBuffer_.imageDeq.front();
      imageDataBuffer_.timeStampDeq.front();//We don't need time here, we could also obtain it
      imageDataBuffer_.imageDeq.pop_front();
      imageDataBuffer_.timeStampDeq.pop_front();
  }
  imageDataBuffer_.mtx.unlock();

  double x,y;
  poseDataBuffer_.mtx.lock();
  if (poseDataBuffer_.poseDeq.size() > 0) {
      geometry_msgs::Pose pose=poseDataBuffer_.poseDeq.front();
      x = pose.position.x;
      y = pose.position.y;
      poseDataBuffer_.poseDeq.pop_front();
      poseDataBuffer_.timeStampDeq.pop_front();
  }
  poseDataBuffer_.mtx.unlock();

  if(!image.empty()){
    double offset = (image.cols)/2;

    //First we transform into local coordinate frame (subtract robot coordinates)
    req.pose.x -= x;
    req.pose.y -= y;
    ROS_INFO_STREAM("local position (" << req.pose.x << "," << req.pose.y  << ")");

    //To convert into image coordinates, we need to invert the y axis (as y is down in an image)
    //We also need to devide by the resolution and then offset (as 0,0) is middle of image
    double xImage = (req.pose.x / resolution_)+offset;
    double yImage = (-req.pose.y / resolution_)+offset;

    cv::Point org(100,100);
    cv::Point dest(static_cast<int>(xImage),static_cast<int>(yImage));

    imageProcessing_.setImage(image);
    res.ack = imageProcessing_.checkConnectivity(org,dest);

  }


  ROS_INFO_STREAM(__func__ << " sending back response");//The example has __func__ , which get's the function name, so it assists debugging
  return true; //We HAVE to retrun true to indicate the service call sucseeded

}

void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    //Let's get the pose out from odometry message
    //rosmsg show nav_msgs/Odometry
    geometry_msgs::Pose pose=msg->pose.pose;

    //We will be using a structire to store odometry with a Deq for time and poses
    //In case we want to search through them, if your only using last reading then you
    //might not need a deque (this is an example different to topics_masterclass)
    poseDataBuffer_.mtx.lock();
    poseDataBuffer_.poseDeq.push_back(pose);
    poseDataBuffer_.timeStampDeq.push_back(msg->header.stamp);
    if(poseDataBuffer_.poseDeq.size()>2){
        poseDataBuffer_.poseDeq.pop_front();
        poseDataBuffer_.timeStampDeq.pop_front();
    }
    poseDataBuffer_.mtx.unlock();
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
    if(imageDataBuffer_.imageDeq.size()>2){
        imageDataBuffer_.imageDeq.pop_front();
        imageDataBuffer_.timeStampDeq.pop_front();
    }
    imageDataBuffer_.mtx.unlock();
}


void PfmsSample::seperateThread() {
   /**
    * The below loop runs until ros is shutdown
    */

    //! What does this rate limiter do?
    ros::Rate rate_limiter(1.0);
    while (ros::ok()) {

      //! @todo One of the assignments requires control of the robot so it turns to face the point requested (and does not move forward).
      //! Yor node should also allow for any new requests to overwrite the current actioned goal.
      //! What data do we need?
        geometry_msgs::Twist cmdvel;
        cmdvel.linear.x = 0.1; //Linear Velocity - V
        cmdvel.angular.z = 0;  //Angular Velocity - Omega
        ROS_INFO_STREAM("sending...[" <<  cmdvel.linear.x << "," <<  cmdvel.angular.z << "]");
        cmd_vel_pub_.publish(cmdvel);

        rate_limiter.sleep();

    }
}

