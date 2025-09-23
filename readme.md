# Project 1 - Reactive Robotics

## Instructions

To launch the world, robot, and controller, run the following command in your terminal:

```bash
roslaunch project1 room_hallway_world.launch
```

To start mapping the environment using SLAM, run:

```bash
roslaunch project1 reactive_mapping.launch
```

This will use gmapping and the reactive controller module found in /scripts/reactive_controller.py to navigate and map the environment.