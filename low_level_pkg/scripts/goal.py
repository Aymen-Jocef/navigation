#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class RobotController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('goal_seeking_robot', anonymous=True)
        
        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        
        # Robot's state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Define the goal position
        self.tolerance = 0.03  # Tolerance for reaching the goal
    
    def odom_callback(self, msg):
        """
        Callback function to update the robot's position and orientation
        from odometry data.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract orientation (quaternion) and convert to Euler angles
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.theta = euler_from_quaternion(orientation_list)
    
    def move_to_goal(self,goal_x,goal_y):
        """
        Main method to control the robot to move toward the goal position.
        """
        rate = rospy.Rate(10)  # 10 Hz
        vel_msg = Twist()

        self.goal_x = goal_x
        self.goal_y = goal_y
        
        while not rospy.is_shutdown():
            # Calculate distance and angle to the goal
            distance = math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
            angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            
            # Check if the robot is within the tolerance
            if distance <= self.tolerance:
                rospy.loginfo("Goal reached!")
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                break
            
            # Proportional controller for linear and angular velocities
            linear_velocity = 1.5 * distance
            angular_velocity = 4.0 * (angle_to_goal - self.theta)
            
            # Normalize angular velocity to [-pi, pi]
            angular_velocity = (angular_velocity + math.pi) % (2 * math.pi) - math.pi
            
            # Set velocities
            vel_msg.linear.x = min(linear_velocity, 0.5)  # Limit max linear velocity
            vel_msg.angular.z = angular_velocity
            
            # Publish the velocity message
            self.velocity_publisher.publish(vel_msg)
            
            # Debugging information
            rospy.loginfo(f"Current Position: x={self.x:.2f}, y={self.y:.2f}, theta={math.degrees(self.theta):.2f}°")
            rospy.loginfo(f"Distance to Goal: {distance:.2f}, Angle to Goal: {math.degrees(angle_to_goal):.2f}°")
            
            # Sleep to maintain loop rate
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.move_to_goal(2,2)
    except rospy.ROSInterruptException:
        pass

