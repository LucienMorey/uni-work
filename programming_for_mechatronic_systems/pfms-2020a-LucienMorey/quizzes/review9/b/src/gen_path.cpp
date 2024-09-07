
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

//ROS Message Types
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"

//ROS-OpenCV Tools for Image Manipulation
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//ROS Service
#include "a4_setup/RequestGoal.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <atomic>

namespace enc = sensor_msgs::image_encodings;

//! Why do we have the struct defined here?
//! WHo needs to access it?
struct PoseArrayBuffer
{
    std::mutex mtx;        // mutex to lock data
    std::atomic<bool> send;         // bool to indicate data to be sent
    geometry_msgs::PoseArray poses; // series of poses as PoseArray
    geometry_msgs::Pose vehPose;    // the vehicle pose
    std::atomic<double> mapSize;    // mapSize
    std::atomic<double> resolution; // resolution
};

//! The Calcback when clicking on image
//! Will report x,y on image as well as which event (right or left click)
//!
//! If left click - Appends the point cicked onto the set of poses in PoseArrayBuffer
//! When right click - Indicates poses will be sent out as path
void CallBackFunc(int event, int x, int y, int flags, void* param)
{

    // Some OpenCV magic to get the pixel location by selecting point on image and left/click
    PoseArrayBuffer* poseArrayBuffer = ((PoseArrayBuffer*)(param)); // 1st cast it back, then deref

     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;


          //!
          //! Convert from Image to Coordinates
          //!


          // OgMap as an image
          //*------------> j(x)
          //|
          //|
          //v i (-y)


          // Coordinate system
          //^ (y)
          //|
          //|
          //*------------> (x)

          // Therefore
          // y axis on image and global are inverted
          // as the image starts from top left corner going down


          // Create one pose
          geometry_msgs::Pose pose; // pose will contain  location of the cell selected in [m]

          // The centre of the OgMap is (0,0) in [m]
          double centre = (poseArrayBuffer->mapSize/poseArrayBuffer->resolution)/2;

          // Convert the pixel value to [m] in two steps below
          pose.position.x=(double(x)-(centre))*poseArrayBuffer->resolution;
          pose.position.y=-(double(y)-(centre))*poseArrayBuffer->resolution;

          // Let's push this pose onto a stack of poses in poseArray Buffer
          poseArrayBuffer->mtx.lock();


          poseArrayBuffer->poses.poses.push_back(pose);
          poseArrayBuffer->mtx.unlock();

          std::cout << "pixel  position (" << x << ", " << y << ")" << std::endl;
          std::cout << "global position (" << pose.position.x << ", " << pose.position.y << ")" << std::endl;


     }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
          poseArrayBuffer->mtx.lock();
          if(poseArrayBuffer->poses.poses.size()>0){
            poseArrayBuffer->send=true;
            std::cout << "Right button of the mouse is clicked, will send poses soon!" << std::endl;
          }
          else{
            std::cout << "No poses selected, can not be sent" << std::endl;
          }
          poseArrayBuffer->mtx.unlock();
     }
}


class PfmsSample{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber sub1_;
    image_transport::Subscriber sub2_;

    ros::Publisher posePublisher_;

    cv_bridge::CvImagePtr cvPtr_;

    int count_;//! A counter to allow executing items on N iterations
    double resolution_;//! size of OgMap in pixels
    double mapSize_;//! size of OgMap in pixels

    struct ImageDataBuffer
    {
        std::deque<cv::Mat> imageDeq;
        std::deque<ros::Time> timeStampDeq;
        std::mutex mtx;
    };
    ImageDataBuffer imageDataBuffer_;//! And now we have our container

    PoseArrayBuffer poseArrayBuffer;


public:
    PfmsSample(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
    {

        // Create subcsribers for odometry and image
        sub1_ = nh_.subscribe("odom", 1000, &PfmsSample::odomCallback,this);
        image_transport::ImageTransport it(nh);
        sub2_ = it.subscribe("map_image/full", 1, &PfmsSample::imageCallback,this);

        // Craete a publisher for path
        posePublisher_ = nh_.advertise<geometry_msgs::PoseArray>("path", 100);

        //Below ge't parameters from parameter server
        nh_.getParam("/local_map/map_resolution", resolution_);
        nh_.getParam("/local_map/map_width", mapSize_);

        poseArrayBuffer.send=false;// Let's make sending not possible initially
        poseArrayBuffer.mapSize=mapSize_;
        poseArrayBuffer.resolution=resolution_;

    }

    ~PfmsSample()
    {
        cv::destroyWindow("view");
    }


    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        //Let's get the pose out from odometry message
        // REMEBER: on command line you can view entier msg as
        //rosmsg show nav_msgs/Odometry
        geometry_msgs::Pose pose=msg->pose.pose;
        poseArrayBuffer.mtx.lock();
        poseArrayBuffer.vehPose= pose;
        poseArrayBuffer.mtx.unlock();

    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

        if((cvPtr_->image.cols !=200) || (cvPtr_->image.rows !=200)){
                ROS_WARN_THROTTLE(60, "The image is not 200 x 200 what has gone wrong!");
        }

        imageDataBuffer_.timeStampDeq.push_back(msg->header.stamp);


        if(imageDataBuffer_.imageDeq.size()>3){
            imageDataBuffer_.imageDeq.pop_front();
            imageDataBuffer_.timeStampDeq.pop_front();
        }

        imageDataBuffer_.mtx.unlock();

    }


    void seperateThread() {
       /**
        * The below loop runs until ros is shutdown, to ensure this thread does not remain
        * a zombie thread
        *
        * The loop locks the buffer, checks the size
        * And then pulls items: the pose and timer_t
        * You can think if these could have been combined into one ...
        */

        /// The below gets the current Ros Time
        ros::Time timeImage = ros::Time::now();
        cv::Mat image;
        count_ =0;
        cv::namedWindow("view",CV_WINDOW_NORMAL);
        cv::startWindowThread();
        cv::waitKey(50);

        // OpenCV callback when using mouse, window view will invole CallBackFunc with poseArrayBuffer being passed
        // as an extra vaiarble (by ref passed)
        cv::setMouseCallback("view", CallBackFunc, &poseArrayBuffer);


        while (ros::ok()) {

            bool imageOK=false;

            //! Lock image buffer, take one message from deque and unlock it
            imageDataBuffer_.mtx.lock();
            if(imageDataBuffer_.imageDeq.size()>0){
                image = imageDataBuffer_.imageDeq.front();
                timeImage = imageDataBuffer_.timeStampDeq.front();
                imageDataBuffer_.imageDeq.pop_front();
                imageDataBuffer_.timeStampDeq.pop_front();
                imageOK=true;
            }
            imageDataBuffer_.mtx.unlock();

            // Update GUI Window
            if(imageOK){

                cv::Mat rgbImage;
                cv::cvtColor(image,rgbImage,CV_GRAY2RGB);
                cv::circle(rgbImage, cv::Point((image.rows/2), (image.cols/2)), 3, CV_RGB(0,0,255),-1);

                if(poseArrayBuffer.send){

                   poseArrayBuffer.mtx.lock();

                   //Let's grab a timestamp and put it on the poses
                   poseArrayBuffer.poses.header.stamp = ros::Time::now();

                   //Push out the pose array
                   posePublisher_.publish(poseArrayBuffer.poses);

                   poseArrayBuffer.poses.poses.clear();
                   poseArrayBuffer.send=false;
                   poseArrayBuffer.mtx.unlock();

                   cv::putText(rgbImage, "Sent Path.", cv::Point(30,30),
                       cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(200,200,250), 1, CV_AA);
                   cv::imshow("view", image);
                   cv::waitKey(3000); // waits for 3s so we can visualise this
                }
                else
                {
                  cv::imshow("view", image);
                  cv::waitKey(5);
                }
            }
            else{
                 // This delay slows the loop down for the sake of readability
                std::this_thread::sleep_for (std::chrono::milliseconds(50));
            }
        }
        std::cout << __func__ << " thread terminated" << std::endl;
    }

};


int main(int argc, char **argv)
{


  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "gen_path");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  std::cout << "STARTING" << std::endl;

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on teh function desired
   */
  std::shared_ptr<PfmsSample> gc(new PfmsSample(nh));
  std::thread t(&PfmsSample::seperateThread,gc);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();
  t.join();

  return 0;
}

