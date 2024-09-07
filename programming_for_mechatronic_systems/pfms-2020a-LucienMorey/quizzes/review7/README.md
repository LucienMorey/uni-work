Quiz 7
======

Part A
------

This quiz revists aspects of a past quiz, extended with a library and use of OpenCV.
You have been provided a library that implements a Car. The car.h header file describes the public functions available and settings [car.h](./a/dep/bionic/car.h)

1) TASK: Modify the main to create three vehicles with provided specifications, add them to a STL container of Cars. Utilise the car library provided and refer to class `Car` in the header file.
[done]
2) TASK: Write a loop in the main that uses the 'Cars' and prints out the make, model and top speed
[done]
3) TASK: Race the cars 
``` [x]
//! 1. Accelerating each vehicle until it reachs top speed
//! 2. When each vehicle reaches top speed deccelerate it
//! 3. When the first vehicle reaches zero speed then stop the race
```
HINT: Use a loop to do this by going over a container of vehicles
HINT: What other information needs to be stored to know when to decelerate each vehicle and where would you store this

4) TASK: Progress of Cars on Track (STL, Classes and OpenCV)
[x]
Uncomment the raceDisplay.updateDisplay(cars) when you have cars available.
In the [display_race.cpp](./a/display_race.cpp) make following changes.

```
    //! 1. Convert distance traveled (odometry) into number of full circles down on track using
    //! RADIUS (which is in header file
    //! 2. Computing the remaining angle position (from the full loop)
```

5) TASK: Drawing the vehcile position on the track using OpenCV functions.
[x]

HINT FOR TASK 4 and 5: Distnace travelled needs to be converted into position on a circle, as the vehciles go round a circular track with RADIUS in [dsplay_race.h](./a/display_race.h). Convert the distance into number of loops and then find the angle the vehicle is at on the circle (2*PI*R is full circle, knowing the distance and R we can calculate the angle), convert this to X,Y using sin/cos and plot on circle.

Part B
------

This set of questions assumed you have installed ROS and followed material in week09 on UTSOnline: 
* created a catking worskpace
* compiled the beginner_tutorial package (in tutorial/week09/starter).

1) QUESTION: How does ROS know about your catkin workspace (HINT: in what file does it source the setup.bash)?
[inform ROS by editting 'bash.rc' in HOME directory accessible through gedit ~/.bashrc ==> source /opt/ros/melodic/setup.bash]
2) QUESTION: If you have started `roscore`, what is the command to run the `talker` node from your beginner_tutorials package.
[talker ==> rosrun beginner_tutorials talker]
3) QUESTION: What is the command to run the `listener` node from your beginner_tutorials package.
[listener ==> rosrun beginner_tutorials listener]
4) QUESTION: Examine the help that ROS provides on `rostopic`. Using `rostopic` print to screen the contents of the `chatter` topic. (You need to have `talker` node running).
[rostopic info /chatter, Type: std_msgs/String, Publishers: */ talker, Subscribers: None]
5) QUESTION: Having `listener` and `talker` running, using `rostopic` determine (a) the node subcribing to `/chatter` (b) the node publishing to `/chatter` and (c) the data type being used on this topic.
[a)Listener   b)Talker   c)std_msgs::String]
