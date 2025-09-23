#!/usr/bin/env python
import rospy
from std_msgs.msg import String



# constants
MAX_RANDOM_TURN_DEGREE_ANGLE = 15
FRONT_ESCAPE_DISTANCE_FEET = 1
FORWARD_MOVMENT_DISTANCE_FEET_BEFORE_TURN = 1
ESCAPE_TURN_DEGREE_ANGLE = 180
ESCAPE_TURN_DEGREE_ANGLE_VARIANCE = 30
FEET_PER_METER = 3.28084

class ReactiveController:
    def __init__(self):
        ...

    def on_collision(self):
        ...
        # Halt

    def on_symmetric_obstacle_ahead(self):
        ...
        # turn a random degree angle between ESCAPE_TURN_DEGREE_ANGLE - ESCAPE_TURN_DEGREE_ANGLE_VARIANCE and ESCAPE_TURN_DEGREE_ANGLE + ESCAPE_TURN_DEGREE_ANGLE

    def on_asymmetric_obstacle_ahead(self):
        ...

    def on_forward_movement_complete(self):
        ...
        # turn a random degree angle between -MAX_RANDOM_TURN_DEGREE_ANGLE and MAX_RANDOM_TURN_DEGREE_ANGLE

    def on_keypress(self, key):
        ...

    def drive_forward(self, distance_feet):
        ...
    