Quiz 9
======

INFORMATION
------
- To develop the quiz 

symbolically link the [quiz9 folder](.) to your catkin_ws/src (ie if your quiz9 folder path is <git_repo_path>/quizzes/quiz9/ then execute:
```bash
cd ~/catkin_ws/src
ln -s <git_repo_path>/quizzes/quiz9/
```
- Compile with catkin_make
```bash
cd ~/catkin_ws
catkin_make
```

- You will need to run the a4_setup to enable testing your code
```bash
roslaunch a4_setup a4_setup.launch
```
- When running your package for Quiz9a
```bash
rosrun quiz9a quiz9a-sample
```
- When running your package for Quiz6b (substitute XXX for name of node)
```bash
rosrun quiz9b quiz9b-XXX
```
- Sections for TASKS in the quiz have a `(//! @todo)` in code to enable identification

*For part A*

After running `rosrun quiz9a quiz9a-sample`
You wil be able to request a goal via the terminal, type below
```bash
rosservice call /check_goal
```
then hit *TAB* twice, will auto populate with entier message, edit x,y and hit enter to test code.
```bash
rosservice call /check_goal "x: 0.0
y: 0.0
theta: 0.0"
```

*For Part B*

To test task 2-5 run the quiz9b-gen_path node `rosrun quiz9b quiz9b-gen_path`

Left click as many time as you wish on the image, when you right click the poses are sent (poses are aquired when you left click, all poses are sent across to your quiz9b-plot_path node).

Part A
------
1) QUESTION: Look at the code in [folder a](./a), what is the name of the package?

    quiz_9a


2) TASK: In the code [seperateThread](./a/src/sample.cpp) "ros::Rate rate_limiter(XXX)" located in [sample.cpp](./a/sample.cpp) achieve running thread at specific rate. Adjust it to run every 5 seconds.

3) QUESTION: In the code [requestGoal](./a/src/sample.cpp) we utilise a RequestGoal service. In what facility is it: (A) offer a service (incoming - recieve request) or (B) client (outgoing - makes request).  

    (A)

4) TASK: In the code [requestGoal](./a/src/sample.cpp) augument the RequestGoal service to return true if the global coordinate supplied in the requst is within current OgMap (just needs to be within the map - either free / occupied or unknown).

5) TASK: In the code [seperateThread](./a/src/sample.cpp), draw a circle at global location P(x,y)=(8.0,8.0) on the OgMap


Part B
------
1) QUESTION: Look at the code in [folder b](./b), what is the name of the package(s)

    quiz_9b

2) QUESTION: Look at the code in [folder b](./b), what is the name of the node(s)?

    quiz9b-plot_path, quiz_9b-gen_path

3) TASK: In [pathCallback of PfmsSample](./b/src/plot_path.cpp) add the series of poses received onto the pathArrayBuffer_

4) TASK: In [seperateThread of PfmsSample](./b/src/plot_path.cpp), convert the checkpoints (poses) to pixels on the image (Ogmap)

5) TASK: In [seperateThread of PfmsSample](./b/src/plot_path.cpp), draw the checkpoints on a copy of the OgMap
