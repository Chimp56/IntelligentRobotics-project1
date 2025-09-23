# Project 1 - Reactive Robotics

The behaviors your robot will carry out are as follows, ordered from highest priority to lowest:

1. Halt if collision(s) detected by bumper(s).
2. Accept keyboard movement commands from a human user.
3. Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
4. Avoid asymmetric obstacles within 1ft in front of the robot.
5. Turn randomly (uniformly sampled within ±15°) after every 1ft of forward movement.
6. Drive forward.

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

https://github.com/JdeRobot/base/pull/732/files#diff-02e337c3ee81c97584f3d34f9b17885bb26feecfa34ca1ddddd9d4fa2925256d

https://answers.ros.org/question/334143/

https://docs.ros.org/en/noetic/api/turtlebot3_msgs/html/msg/SensorState.html

https://docs.ros.org/en/hydro/api/kobuki_msgs/html/msg/BumperEvent.html

https://wiki.ros.org/kobuki_bumper2pc/hydro