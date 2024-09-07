Quiz 8
======

Part A
------
You have been provided piece of code your colleauges have developed for Assignment 1/2, and we will be debugging these pieces of code.

![Debugging Pain](https://media.giphy.com/media/6yRVg0HWzgS88/giphy.gif)

For questions 1-3 use [vector_ops.cpp](./a/vector_ops.cpp). The intent of this code was to create a STL container resembling a matrix of elements (2x5).
 
The code compiles (no compile-time error) and also runs (no run-time errors).    
Yet, it fails to display the values (you would anticipate to see two rows of values).
```
0 1 2 3 4
0 1 2 3 4
```
The code has a few points of failure causing unintended behaviour.

For questions 4-5 use [main.cpp](./a/main.cpp) and the [Shape](./a/shape.h), [Rectangle](./a/rectangle.h) and [RectangleHelper](./a/rectanglehelper.h) class.

The code compiles (no compile-time error) and also runs (no run-time errors). Yet, it fails to determine line intersects.
Further, the RectangleHelper class that was supposed to print properties seems to be not working either.

1) QUESTION: When you request a change in capacity [reserve keyword](http://www.cplusplus.com/reference/vector/vector/reserve/), does that create elements of the vector.
A1) No ,this function has no effect on the vector size and cannot alter its elements.

2) QUESTION: What line of code is the first point of failure that could have been detected if the STL container was accessed correctly.
A2) Line 17 in vector ops , when pushback is used.

3) QUESTION: Once errors associated with creating / storing data into the matrix are fixed, what additional error in this code results in incorrect matrix size.
A3) Line 16 - for loop in vector ops,  results in 6 coloumns being accessed.

4) QUESTION: Why does the intercept method in `Rectangle` fail to report the intercept correctly?
A4) The intercept always returns true.

5) QUESTION: Why does the printArea method in `RectangleHelper` not print the correct area?
A5) Rectangle to print is not indicated.

Part B
------

A package `topics_masterclass` is provided in the [tutorials/week10/starter directory](../../tutorials/week10/starter),link it to your workspace.
To copy:
```bash
cd ~/catkin_ws/src/
ln -s <path-to-git>/tutorials/week10/starter/topics_masterclass
```
Build the package using the `catkin_make` command (what folder do you need to be in to execute this command?).

You will also need the stage_ros package, install it by below (replace kinetic with melocdic if you have 18.04 and ROS melodic)
```bash
sudo apt-get install ros-kinetic-stage-ros
```
Start roscore in one terminal
```bash
roscore
```
Start stage_ros in another terminal (replace kinetic with melocdic if you have 18.04 and ROS melodic)
```bash
rosrun stage_ros stageros /opt/ros/kinetic/share/stage/worlds/simple.world
```
You should have the simulator appear
![Simple World in Stage Simulator](http://4.bp.blogspot.com/_B6REL4AVpFA/Szk9ipweWTI/AAAAAAAAALc/orflaXzpcZk/s400/Picture+2.png)

The goal is to modify the provided package to obtain pose of robot (x,y yaw) from nav_msgs/Odometry.

1) QUESTION: How would you access orientation in the `nav_msgs/Odometry` message (HINT: rosmsg show). The answer needs full path to orinettaion from the msg (for String we had msg.data)
use rosmsg show nav_msgs/Odometry. Look up where orientation is shown,geometry_msgs/Quaternion. Then rosmsg show geometry_msgs/Quaternion.

2) QUESTION: What type of message is the orientation?
geometry message as mentioned in Q1.

3) QUESTION: Where is time of this message stored?
rosmsg show geometry_msgs/QuaternionStamped . Quaternionstamped - time stamped 

4) TASK: Use the [ROS TF] library helper function to get yaw from the orientation and print to screen (standard out)
yaw=tf::getyaw(msg->pose.pose.orientation);
std::cout << yaw: <<yaw<<std::endl;

5) TASK: Print the pose of robot (x,y yaw) to screen using ROS_INFO_STREAM-nope

[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html

