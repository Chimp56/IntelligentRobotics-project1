#!/usr/bin/env python
import rospy
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist



# constants
MAX_RANDOM_TURN_DEGREE_ANGLE = 15
FRONT_ESCAPE_DISTANCE_FEET = 1
FORWARD_MOVMENT_DISTANCE_FEET_BEFORE_TURN = 1
ESCAPE_TURN_DEGREE_ANGLE = 180
ESCAPE_TURN_DEGREE_ANGLE_VARIANCE = 30
FEET_PER_METER = 3.28084

class ReactiveController:
    def __init__(self):
        self.state = 'DRIVE_FORWARD'
        
        rospy.init_node('reactive_controller', anonymous=True)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        
        # Subscriber for bumper events
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        
        self.collision_detected = False

    def run(self):
        """
        High level control loop
        """
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            self.drive_forward()

            if self.collision_detected:
                rospy.loginfo("Robot halted due to collision")
                # Stay in collision state until manually reset
                rate.sleep()
                continue

            
            rate.sleep()

    def bumper_callback(self, data):
        """
        Callback function for bumper events
        """
        if data.state == BumperEvent.PRESSED:
            collision_detected_str = 'Collision detected! Bumper:' + str(data.bumper) + ' (0=LEFT, 1=CENTER, 2=RIGHT)'
            rospy.loginfo(collision_detected_str)
            self.collision_detected = True
            self.state = 'COLLISION'
            self.on_collision()
        elif data.state == BumperEvent.RELEASED:
            bumper_relaesed_str = "Bumper released: " + str(data.bumper)
            rospy.loginfo(bumper_relaesed_str)
            self.collision_detected = False

    def on_collision(self):
        """
        Handle collision by immediately halting the robot
        """
        rospy.loginfo("HALTING ROBOT - Collision detected!")
        
        halt_msg = Twist()
        halt_msg.linear.x = 0.0
        halt_msg.linear.y = 0.0
        halt_msg.linear.z = 0.0
        halt_msg.angular.x = 0.0
        halt_msg.angular.y = 0.0
        halt_msg.angular.z = 0.0
        
        # Publish the halt command
        self.cmd_vel_pub.publish(halt_msg)
        

    # def on_symmetric_obstacle_ahead(self):
    #     ...
    #     # turn a random degree angle between ESCAPE_TURN_DEGREE_ANGLE - ESCAPE_TURN_DEGREE_ANGLE_VARIANCE and ESCAPE_TURN_DEGREE_ANGLE + ESCAPE_TURN_DEGREE_ANGLE

    # def on_asymmetric_obstacle_ahead(self):
    #     ...

    # def on_forward_movement_complete(self):
    #     ...
    #     # turn a random degree angle between -MAX_RANDOM_TURN_DEGREE_ANGLE and MAX_RANDOM_TURN_DEGREE_ANGLE

    # def on_keypress(self, key):
    #     ...

    def drive_forward(self):
        forward_msg = Twist()
        forward_msg.linear.x = .8
        forward_msg.linear.y = 0.0
        forward_msg.linear.z = 0.0
        forward_msg.angular.x = 0.0
        forward_msg.angular.y = 0.0
        forward_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(forward_msg)





if __name__ == '__main__':
    try:
        controller = ReactiveController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


