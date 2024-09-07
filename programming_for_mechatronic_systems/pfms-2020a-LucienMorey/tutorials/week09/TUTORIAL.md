Week 9 Tutorial Questions
=========================

Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

Ex01: Modifying a ROS Node
---------------------------

Modify the provided package to do the following:

* Change the message type to publish/subscribe to a double.
* Every 0.2 seconds the `talker` node should draw samples from a Gaussian distribution (mean = 0.0, std dev = 1.0) and *publish* the samples to a topic
* The `listener` node should *subscribe* to the topic build a histogram of the numbers received (bin-size = 0.1)
* Upon recieving the 50th number the listener should display the histogram somehow and reset its counts 

### Questions ###

* Do you need to change the Message? refer to http://wiki.ros.org/std_msgs
* Will you use a container for the histogram?
* How would you display it?

Ex02: Publishing and Subscribing to Images/Video Streams
----------------------------

Now we will take a ROS Node and modify its code to make it do something new. This should help you get familiar with writing software for ROS.

Link the folder image_transport_tutorial to your catkin_ws src folder. 
```bash
cd ~/catkin_ws/src
ln -s ln -s ~/<your_git_location>/tutorials/week09/starter/image_transport_tutorial/
```
* Build the package using the `catkin_make` command (You need to be in the ~/catkin_ws folder to execute `catkin_make`)
* Open a terminal: run `roscore` first if not running yet
* Grab an image, such as GrumpyCat `wget https://media.wired.com/photos/5cdefc28b2569892c06b2ae4/master/w_1500,c_limit/Culture-Grumpy-Cat-487386121-2.jpg`
* Open a terminal: run the `my_publisher` node using `rosrun image_transport_tutorial my_publisher Culture-Gru
mpy-Cat-487386121-2.jpg` 
* Open a terminal: `my_subscriber` node using `rosrun image_transport_tutorial my_subscriber` 
* Read the source code and try to understand what is going on

### Questions ###

* How can we load another image?
* Can you modify this code to draw circles on the image prior to publishing (just one random circle)?
* Follwing from instructions in [Publishing Images], can you add a video stream from a webcam instead of the image?

[ROS Installation Instructions]: http://wiki.ros.org/ROS/Installation
[ROS Tutorials]: http://wiki.ros.org/ROS/Tutorials
[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
[Polar to Cartesian]: https://www.mathsisfun.com/polar-cartesian-coordinates.html
[Publishing Images]: http://wiki.ros.org/image_transport/Tutorials/PublishingImages
[Subscribing to Images]: http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
