Week 10 Tutorial Questions
=========================

Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

The tutorial requires the a5_support packages to be part of your ROS system as decried in prework information). Modify the provided starter package as per the prework to complete the following excersizes.

Ex02: Find the closest point [x,y] to the robot in local cordinates (robots own coordinates) using sensor_msgs::LaserScan
-----------------------------------------------
See the `laserCallback` function of the `PfmsSample` class to complete this exercise.

**You task is to print the location of the nearest obstacle in the laser scan. The location should be given as the x, y position of the obstacle *relative* to the robot's coordinate frame.**

* For more information about the structure of the laser scan message type `rosmsg show sensor_msgs/LaserScan`
* Check if the topic you have subscribed to by your component is corect? Can we change this without recompiling your code (mapping current_topic:=new_topic)
* You will first need to figure out which range value in the scan is the closest
* Then you will need to convert this range and bearing to an x, y point

### Questions ###

* What part of the message do we need to iterate over?
* How do we convert from range data to x,y (this is known as [Polar to Cartesian])
* Where is time of this message stored?
* Is the closest point identified the same as the one you see as closest on the stage simulator in global cordinates?
* How do we find the intensity of the laser scaner return?

Ex03 : Find the closest point [x,y] to the robot using the OgMap (encoded as cv::Mat image)
-----------------------------------------------

Find the code for Ex03

* Where is time of this message stored?
* Given the OgMap is in pixels, what information do we need to convert to [x,y]? 
* Where is 0,0 on the Image?
* Where is the robot on the OgMap?

Use all the information above to find the closest point in global coordinates [x,y] to the robot

### Questions ###


* Is the closest point identified the same as the one you see as closest on the stage simulator?
* Why is this the case?

Ex04: Find the closest point [x,y] to the robot in global coordiantes using the robot pose and laser data
---------------------------

### Questions ###

* If we need to combine the information and use it in this separate thread what do we need to consider?
* Should the closest point identified the same as the one you see as closest on the stage simulator?


Ex05: Mark the closest point on a RGB version of the OgMap image
---------------------------

### Questions ###

* If we need to combine the information and use it in this seperate thread what do we need to consider
* Is the closest point identified the same as the one you see as closest on the stage simulator?

[ROS Installation Instructions]: http://wiki.ros.org/ROS/Installation
[ROS Tutorials]: http://wiki.ros.org/ROS/Tutorials
[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
[Polar to Cartesian]: https://www.mathsisfun.com/polar-cartesian-coordinates.html
