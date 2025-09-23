# Project 1 - Reactive Robotics

## Instructions

Clone the repository into your catkin workspace:

```bashcd ~/catkin_ws/src
git clone https://github.com/Chimp56/IntelligentRobotics-project1 project1
cd ..
catkin_make
source devel/setup.bash
```


To launch the world, robot, and controller, run the following command in your terminal:

```bash
roslaunch project1 room_hallway_world.launch
```

To start mapping the environment using SLAM, run:

```bash
roslaunch project1 reactive_mapping.launch
```

This will use gmapping and the reactive controller module found in /scripts/reactive_controller.py to navigate and map the environment.

## Help

https://wiki.ros.org/rospy/Overview/

https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29