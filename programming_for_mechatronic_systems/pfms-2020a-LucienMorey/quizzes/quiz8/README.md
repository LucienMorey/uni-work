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
* The reserve function will reserve the memory for n elements if n is greater than the current capacity. It will not populate those elements with values.

2) QUESTION: What line of code is the first point of failure that could have been detected if the STL container was accessed correctly.
* line 12 is the first point of failure in the code. The attempt to reserve space for uninitialised elements means that the outer vector will remain at 0 size for the duration of the program. This has been fixed by using resize instead of reserve on line 10 so that null elements are initialised as well as allocated memory in the outer vector

3) QUESTION: Once errors associated with creating / storing data into the matrix are fixed, what additional error in this code results in incorrect matrix size.
* The for loop on line 17 also contains a logic error. The vector is 0 indexing so the final amount of columns in the vector ends up being 6. The condition of iteration has been changed to < to ensure the intended amount of columns are seen.

4) QUESTION: Why does the intercept method in `Rectangle` fail to report the intercept correctly?
* the recantgle class overwrote the protected centreX_ and centreY_ variables in the shape class. This meant that the recangle classs was referencing these values instead of the parent class values that were being overwritten by the setoffset method. to remedy this the private centre variables have been removed from the recantangle class.

5) QUESTION: Why does the printArea method in `RectangleHelper` not print the correct area?
The rectangle helper class is subclassed from the recangle. When the helper was created, another rectangle was created with height and with of 0 leading to 0 area. I have changed this so that the rectanglehelper is no longer subclassed fro mthe rectangle and instead has a rectangle setter so it can refer to a previously created rectangle.

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
* msg.pose.pose.orientation

2) QUESTION: What type of message is the orientation?
* the orientation is a quaternion structure containing 4 float64 values x,y,z,w

3) QUESTION: Where is time of this message stored?
* msg.header.stamp

4) TASK: Use the [ROS TF] library helper function to get yaw from the orientation and print to screen (standard out)
* included tf/transform_datatypes.h to obtain correct getYaw transform function. obtained pos x, y and orientation by dereferencing the message pointer and using th path as listed above for orientation and msg->pose.pose.position for the x and y values. then parsed the quaternion to getYaw to get the robot heading and printed with std::cout

5) TASK: Print the pose of robot (x,y yaw) to screen using ROS_INFO_STREAM
* altered the ROS_INFO_STREAM as follows ROS_INFO_STREAM("x: " << x << " y: " << y << " yaw " << yaw);

[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html

