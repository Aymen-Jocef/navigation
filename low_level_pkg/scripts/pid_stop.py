#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
from sensor_msgs.msg import LaserScan
import math

class RobotController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('goal_seeking_robot', anonymous=True)

        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)

        # Robot's state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Initialize laser scan distances as an empty list
        self.distances = []

        # PID variables for linear velocity
        self.linear_kp = 1.5
        self.linear_ki = 0.02
        self.linear_kd = 0.3
        self.linear_error_sum = 0.0
        self.linear_last_error = 0.0

        # PID variables for angular velocity
        self.angular_kp = 4.0
        self.angular_ki = 0.02
        self.angular_kd = 0.3
        self.angular_error_sum = 0.0
        self.angular_last_error = 0.0

        # Define the goal position and orientation
        self.tolerance = 0.03  # Tolerance for reaching the goal
        self.angle_tolerance = 0.05  # Tolerance for aligning to the desired angle
        self.detection_range = 0.4

        # Timing
        self.last_time = time.time()

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

    def laser_scan_callback(self, scan_data):
        self.distances = scan_data.ranges
        self.angle_min = scan_data.angle_min
        self.angle_increment = scan_data.angle_increment
        self.range_min = scan_data.range_min  # Corrected this to range_min


    def align_to_angle(self, desired_angle):
        """
        Align the robot to a desired angle after reaching the goal.
        """
        rate = rospy.Rate(10)  # 10 Hz
        vel_msg = Twist()

        while not rospy.is_shutdown():


            # Calculate angular error
            angular_error = desired_angle - self.theta
            angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

            # Check if the robot is within the angle tolerance
            if abs(angular_error) <= self.angle_tolerance:
                rospy.loginfo("Aligned to the desired angle!")
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                break

            # PID control for angular velocity
            current_time = time.time()
            delta_time = current_time - self.last_time
            self.last_time = current_time

            self.angular_error_sum += angular_error * delta_time
            angular_derivative = (angular_error - self.angular_last_error) / delta_time
            self.angular_last_error = angular_error

            angular_velocity = (
                self.angular_kp * angular_error +
                self.angular_ki * self.angular_error_sum +
                self.angular_kd * angular_derivative
            )

            # Set angular velocity
            vel_msg.angular.z = angular_velocity

            # Publish the velocity message
            self.velocity_publisher.publish(vel_msg)

            # Debugging information
            rospy.loginfo(f"Current Angle: {math.degrees(self.theta):.2f}°, Desired Angle: {math.degrees(desired_angle):.2f}°")

            # Sleep to maintain loop rate
            rate.sleep()


    def move_to_goal(self, goal_x, goal_y, desired_angle):
        """
        Main method to control the robot to move toward the goal position
        and then align to the desired angle.
        """
        rate = rospy.Rate(10)  # 10 Hz
        vel_msg = Twist()

        self.goal_x = goal_x
        self.goal_y = goal_y

        while not rospy.is_shutdown():
            # Check if laser scan data is available
            if not self.distances:
                rospy.logwarn("Laser scan data not available yet. Waiting...")
                continue

            # Calculate distance and angle to the goal
            distance = math.sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)
            angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)

            # Check for obstacles in the lidar scan data
            object_detected = False
            for i in range(0, len(self.distances), 2):  # Loop through lidar data with a step of 2
                scan_distance = self.distances[i]
                if scan_distance < self.detection_range and scan_distance > self.range_min:
                    object_detected = True
                    angle = self.angle_min + i * self.angle_increment
                    angle_degrees = math.degrees(angle)
                    rospy.loginfo(f"Obstacle detected at angle {angle_degrees:.2f}° and distance {scan_distance:.2f}m.")
                    break

            if object_detected:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                break  # Exit the loop after stopping the robot

            # Check if the robot is within the tolerance to stop moving towards the goal
            if distance <= self.tolerance:
                rospy.loginfo("Goal reached!")
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                break  # Exit the loop after reaching the goal

            # PID control for linear velocity
            current_time = time.time()
            delta_time = current_time - self.last_time
            self.last_time = current_time

            linear_error = distance
            self.linear_error_sum += linear_error * delta_time
            linear_derivative = (linear_error - self.linear_last_error) / delta_time
            self.linear_last_error = linear_error

            linear_velocity = (
                self.linear_kp * linear_error +
                self.linear_ki * self.linear_error_sum +
                self.linear_kd * linear_derivative
            )

            # PID control for angular velocity
            angular_error = angle_to_goal - self.theta
            angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

            self.angular_error_sum += angular_error * delta_time
            angular_derivative = (angular_error - self.angular_last_error) / delta_time
            self.angular_last_error = angular_error

            angular_velocity = (
                self.angular_kp * angular_error +
                self.angular_ki * self.angular_error_sum +
                self.angular_kd * angular_derivative
            )

            # Set velocities with limits
            vel_msg.linear.x = min(linear_velocity, 0.5)  # Limit max linear velocity
            vel_msg.angular.z = angular_velocity

            # Publish the velocity message
            self.velocity_publisher.publish(vel_msg)

            # Sleep to maintain loop rate
            rate.sleep()

        # Align to the desired angle after reaching the goal or stopping due to obstacle
        self.align_to_angle(desired_angle)


if __name__ == '__main__':
    try:
        controller = RobotController()

        # First goal: (1, 1) with orientation 90°
        rospy.loginfo("Moving to the first goal: (1, 1, 90°)")
        controller.move_to_goal(1, 1, math.radians(0))

        # Second goal: (-1, 1) with orientation -90°
        rospy.loginfo("Moving to the second goal: (-1, 1, -90°)")
        controller.move_to_goal(1,-1, math.radians(-90))
 # Second goal: (-1, 1) with orientation -90°
        rospy.loginfo("Moving to the second goal: (0, 0, 0°)")
        controller.move_to_goal(0,0, math.radians(0))


    except rospy.ROSInterruptException:
        pass
