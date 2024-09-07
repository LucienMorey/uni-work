
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

//ROS Message Types
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"

//ROS-OpenCV Tools for Image Manipulation
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

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



/**
 * This node attempts to draw the received path on an image
 */


class PfmsSample{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    image_transport::Subscriber sub3_;
    cv_bridge::CvImagePtr cvPtr_;
    ros::ServiceClient client_;

    int count_;//! A counter to allow executing items on N iterations
    double resolution_;//! size of OgMap in pixels
    double mapSize_;//! size of OgMap in pixels

    //! Is theer a better way to store data, instead of a structure?
    //! refer to Tutorial 7 exercise 3
    struct PoseArrayBuffer
    {
        std::mutex mtx;                 //! mutex to lock data
        std::atomic<bool> received;     //! bool to indicate data to be sent
        geometry_msgs::PoseArray poses; //! series of poses as PoseArray
        geometry_msgs::Pose vehPose;    //! the vehicle pose
        std::atomic<double> mapSize;    //! mapSize
        std::atomic<double> resolution; //! resolution
    };

    PoseArrayBuffer pathArrayBuffer_;//! Path received via callback


    struct ImageDataBuffer
    {
        //! Given these elements come in two's (image and time)
        //! Is there a better type of STL container rather than two seperate deques?
        std::deque<cv::Mat> imageDeq;
        std::deque<ros::Time> timeStampDeq;
        std::mutex mtx;
    };

    ImageDataBuffer imageDataBuffer_;//! Container for image data


public:
    PfmsSample(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
    {
        sub1_ = nh_.subscribe("odom", 1000, &PfmsSample::odomCallback,this);
        image_transport::ImageTransport it(nh);
        sub3_ = it.subscribe("map_image/full", 1, &PfmsSample::imageCallback,this);

        sub2_ = nh_.subscribe("path", 10, &PfmsSample::pathCallback,this);

        //Below get's parameters from parameter server
        nh_.getParam("/local_map/map_resolution", resolution_);
        nh_.getParam("/local_map/map_width", mapSize_);

        // We store information allowing to convert from global to local
        pathArrayBuffer_.received=false;
        pathArrayBuffer_.mapSize=mapSize_;
        pathArrayBuffer_.resolution=resolution_;

    }

    ~PfmsSample()
    {
        cv::destroyWindow("view");
    }


    void pathCallback(const geometry_msgs::PoseArrayConstPtr & msg)
    {
        //Let's get the pose out from odometry message
        // REMEBER: on command line you can view entier msg as
        //rosmsg show nav_msgs/Odometry
        ROS_INFO("New path received");
        ROS_INFO_STREAM("Received path has:" << msg->poses.size() << "elements");

        //! @todo - Q3 : Add the series of poses received from msg onto the pathArrayBuffer_.poses
        //! What do we need to do to access the pathArrayBuffer_

        pathArrayBuffer_.mtx.lock();

        pathArrayBuffer_.poses.poses.insert(pathArrayBuffer_.poses.poses.end(), msg->poses.begin(), msg->poses.end());

        pathArrayBuffer_.received = true;

        pathArrayBuffer_.mtx.unlock();
    }


    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        //Let's get the pose out from odometry message
        // REMEBER: on command line you can view entier msg as
        //rosmsg show nav_msgs/Odometry
        geometry_msgs::Pose pose=msg->pose.pose;
        pathArrayBuffer_.mtx.lock();
        pathArrayBuffer_.vehPose= pose;
        pathArrayBuffer_.mtx.unlock();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // Below code pushes the image and time to a deque, to share across threads
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

        // Additional Question: What does code below do?
        if((cvPtr_->image.cols !=200) || (cvPtr_->image.rows !=200)){
                ROS_WARN_THROTTLE(60, "The image is not 200 x 200 what has gone wrong!");
        }

        imageDataBuffer_.timeStampDeq.push_back(msg->header.stamp);

        if(imageDataBuffer_.imageDeq.size()>2){
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
        cv::namedWindow("path",CV_WINDOW_NORMAL);
        cv::startWindowThread();
        cv::waitKey(50);

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

            // Update GUI Window on new OgMap received
            if(imageOK){

                if(pathArrayBuffer_.received){

                  ROS_INFO("!!! Got new path");
                  cv::Mat rgbImage;
                  cv::cvtColor(image,rgbImage,CV_GRAY2RGB);


                  pathArrayBuffer_.mtx.lock();

                  //! @todo - Q4
                  //!
                  //! Convert the global path to pixel coordinates
                  //! This will be reverse of the code supplied in gen_path CallBackFunc

                  auto path = pathArrayBuffer_.poses.poses;
                  for (auto p:path)
                  {
                      int px = p.position.x/resolution_ + (image.cols/resolution_)/2;
                    int py = p.position.y/resolution_ + (image.rows/resolution_)/2;

                 //! @todo - Q5
                  //! Draw the checkpoints on the image
                  //!  Use the circle to draw the pose
                   cv::circle(rgbImage, cv::Point(px, py), 3, CV_RGB(255,0,0), 1);
                  }




                   pathArrayBuffer_.poses.poses.clear();
                   pathArrayBuffer_.received=false;
                   pathArrayBuffer_.mtx.unlock();

                   cv::putText(rgbImage, "Received Path.", cv::Point(30,30),
                       cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(200,200,250), 1, CV_AA);

                   cv::imshow("path", rgbImage);


                }
                else{
                    //std::cout << "No new path" << std::endl;
                }

                cv::waitKey(5);
            }
            // This delay slows the loop down for the sake of readability
            std::this_thread::sleep_for (std::chrono::milliseconds(50));
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
  ros::init(argc, argv, "draw_path");

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

