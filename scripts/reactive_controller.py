#!/usr/bin/env python
import rospy
import random
import numpy as np
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



# constants
MAX_RANDOM_TURN_DEGREE_ANGLE = 15
FRONT_ESCAPE_DISTANCE_FEET = 1
FORWARD_MOVMENT_DISTANCE_FEET_BEFORE_TURN = 1
ESCAPE_TURN_DEGREE_ANGLE = 180
ESCAPE_TURN_DEGREE_ANGLE_VARIANCE = 30
FEET_PER_METER = 3.28084
METERS_PER_FEET = 1 / FEET_PER_METER

class ReactiveController:
    def __init__(self):
        self.state = 'DRIVE_FORWARD'
        
        rospy.init_node('reactive_controller', anonymous=True)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        
        # Subscriber for bumper events
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        
        # Subscriber for laser scan data
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.collision_detected = False
        self.obstacle_detected = False
        self.laser_data = None

    def run(self):
        """
        High level control loop
        """
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Check for collision first
            if self.collision_detected:
                rospy.loginfo("Robot halted due to collision")
                rate.sleep()
                continue
            
            # Check for obstacles ahead
            if self.laser_data is not None:
                self.check_obstacles_ahead()
            
            # Drive forward if no obstacles
            if not self.obstacle_detected and not self.collision_detected:
                self.drive_forward()
            
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

    def laser_callback(self, data):
        """
        Callback function for laser scan data
        """
        self.laser_data = data

    def check_obstacles_ahead(self):
        """
        Check if obstacles are within FRONT_ESCAPE_DISTANCE_FEET
        """
        if self.laser_data is None:
            return
        
        # Convert distance threshold to meters
        distance_threshold = FRONT_ESCAPE_DISTANCE_FEET * METERS_PER_FEET
        
        # Get laser ranges
        ranges = np.array(self.laser_data.ranges)
        
        # Filter out invalid readings (inf, nan, or beyond max range)
        valid_ranges = ranges[np.isfinite(ranges)]
        valid_ranges = valid_ranges[valid_ranges < self.laser_data.range_max]
        
        # Check front sector (typically -30 to +30 degrees from center)
        # Convert to indices
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        # Front sector indices (roughly -30 to +30 degrees)
        front_start_idx = int((-np.pi/6 - angle_min) / angle_increment)
        front_end_idx = int((np.pi/6 - angle_min) / angle_increment)
        
        # Ensure indices are within bounds
        front_start_idx = max(0, front_start_idx)
        front_end_idx = min(len(ranges), front_end_idx)
        
        # Get front ranges
        front_ranges = ranges[front_start_idx:front_end_idx]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        front_ranges = front_ranges[front_ranges < self.laser_data.range_max]
        
        # Check if any obstacle is within threshold
        if len(front_ranges) > 0 and np.min(front_ranges) < distance_threshold:
            self.obstacle_detected = True
            rospy.loginfo("Obstacle detected at {:.2f}m (threshold: {:.2f}m)".format(np.min(front_ranges), distance_threshold))
            
            # Determine if obstacle is symmetric or asymmetric
            if self.is_obstacle_symmetric(front_ranges):
                self.on_symmetric_obstacle_ahead()
            else:
                self.on_asymmetric_obstacle_ahead()
        else:
            self.obstacle_detected = False

    def is_obstacle_symmetric(self, front_ranges):
        """
        Determine if obstacle is symmetric by comparing left and right sides
        """
        if len(front_ranges) < 3:
            return True  # Default to symmetric if not enough data
        
        # Split front ranges into left and right halves
        mid = len(front_ranges) // 2
        left_ranges = front_ranges[:mid]
        right_ranges = front_ranges[mid:]
        
        # Calculate average distances for left and right
        left_avg = np.mean(left_ranges) if len(left_ranges) > 0 else float('inf')
        right_avg = np.mean(right_ranges) if len(right_ranges) > 0 else float('inf')
        
        # Consider symmetric if difference is small (within 20% of average)
        avg_distance = (left_avg + right_avg) / 2
        if avg_distance == float('inf'):
            return True
        
        difference = abs(left_avg - right_avg)
        threshold = 0.2 * avg_distance  # 20% tolerance
        
        is_symmetric = difference < threshold
        rospy.loginfo("Obstacle symmetry check: left={:.2f}m, right={:.2f}m, symmetric={}".format(left_avg, right_avg, is_symmetric))
        return is_symmetric

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
        

    def on_symmetric_obstacle_ahead(self):
        """
        Handle symmetric obstacle by turning a random degree angle
        """
        rospy.loginfo("Symmetric obstacle detected - executing escape turn")
        
        # Calculate random turn angle
        min_angle = ESCAPE_TURN_DEGREE_ANGLE - ESCAPE_TURN_DEGREE_ANGLE_VARIANCE
        max_angle = ESCAPE_TURN_DEGREE_ANGLE + ESCAPE_TURN_DEGREE_ANGLE_VARIANCE
        turn_angle = random.uniform(min_angle, max_angle)
        
        # Convert to radians
        turn_radians = np.radians(turn_angle)
        
        rospy.loginfo("Turning {:.1f} degrees ({:.2f} radians)".format(turn_angle, turn_radians))
        
        # Execute turn
        self.execute_turn(turn_radians)
        
        # Reset obstacle detection after turn
        self.obstacle_detected = False

    def on_asymmetric_obstacle_ahead(self):
        """
        Handle asymmetric obstacle by turning towards the clearer side
        """
        rospy.loginfo("Asymmetric obstacle detected - turning towards clearer side")
        
        if self.laser_data is None:
            # Default turn if no laser data
            self.execute_turn(np.radians(90))
            self.obstacle_detected = False
            return
        
        # Analyze left and right sides to determine clearer path
        ranges = np.array(self.laser_data.ranges)
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        # Get left and right sectors
        left_start = int((-np.pi/2 - angle_min) / angle_increment)
        left_end = int((-np.pi/6 - angle_min) / angle_increment)
        right_start = int((np.pi/6 - angle_min) / angle_increment)
        right_end = int((np.pi/2 - angle_min) / angle_increment)
        
        # Ensure indices are within bounds
        left_start = max(0, left_start)
        left_end = min(len(ranges), left_end)
        right_start = max(0, right_start)
        right_end = min(len(ranges), right_end)
        
        # Get average distances for left and right
        left_ranges = ranges[left_start:left_end]
        left_ranges = left_ranges[np.isfinite(left_ranges)]
        left_avg = np.mean(left_ranges) if len(left_ranges) > 0 else 0
        
        right_ranges = ranges[right_start:right_end]
        right_ranges = right_ranges[np.isfinite(right_ranges)]
        right_avg = np.mean(right_ranges) if len(right_ranges) > 0 else 0
        
        # Turn towards clearer side
        if left_avg > right_avg:
            rospy.loginfo("Turning left (clearer path)")
            self.execute_turn(np.radians(90))  # Turn left
        else:
            rospy.loginfo("Turning right (clearer path)")
            self.execute_turn(np.radians(-90))  # Turn right
        
        # Reset obstacle detection after turn
        self.obstacle_detected = False

    def execute_turn(self, angle_radians):
        """
        Execute a turn by the specified angle
        """
        # Calculate turn duration based on angular velocity
        angular_velocity = 0.5  # rad/s
        turn_duration = abs(angle_radians) / angular_velocity
        
        # Create turn command
        turn_msg = Twist()
        turn_msg.linear.x = 0.0
        turn_msg.linear.y = 0.0
        turn_msg.linear.z = 0.0
        turn_msg.angular.x = 0.0
        turn_msg.angular.y = 0.0
        turn_msg.angular.z = angular_velocity if angle_radians > 0 else -angular_velocity
        
        # Execute turn
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while (rospy.Time.now() - start_time).to_sec() < turn_duration:
            self.cmd_vel_pub.publish(turn_msg)
            rate.sleep()
        
        # Stop after turn
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)



    def drive_forward(self):
        forward_msg = Twist()
        forward_msg.linear.x = .1
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


