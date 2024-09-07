#include <gtest/gtest.h>
#include <climits>
#include <vector>

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/LaserScan.h>

#include "../src/detectcabinet.h"

TEST(LoadedBag,DetectCabinet){

    //! The below code tests the cabinet detection class
    //! The data has been saved in a bag, that is opened and used.

    //! Create an object of DetectCabinet class as we will use the public function of that object to run tests against
    DetectCabinet detectCabinet;

    //! Below command allows to find the folder belonging to a package
    std::string path = ros::package::getPath("random_walk");
    // Now we have the path, the images for our testing are stored in a subfolder /test/samples
    path += "/test/bag/";
    // The file is called smiley_og.png
    std::string file = path + "sample.bag";

    //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    sensor_msgs::LaserScan::ConstPtr laserScan;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
      laserScan = m.instantiate<sensor_msgs::LaserScan>();
      if (laserScan != nullptr){
        // Now we have a laserScan so we can proceed
        break;
      }
    }

    bag.close();

    ASSERT_NE(laserScan, nullptr);//Check that we have a scan from the bag

    bool detected = detectCabinet.detectCabinetPresence(laserScan);

    /**
     * @todo - TASK 4 : Use Google Test to ASSERT if the value of detected is true
     *
     */
    ASSERT_TRUE(detected);

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
