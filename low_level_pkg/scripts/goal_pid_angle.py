#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time


class RobotController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('goal_pid_angle')

        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber(
            '/odometry/filtered', Odometry, self.odom_callback)
        self.observer_subscriber = rospy.Subscriber(
            '/observer', Point, self.observer_callback)

        # Robot's state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # PID variables for linear velocity
        self.linear_kp = 1.5
        self.linear_ki = 0.021
        self.linear_kd = 0.4
        self.linear_error_sum = 0.0
        self.linear_last_error = 0.0

        # PID variables for angular velocity
        self.angular_kp = 2.0
        self.angular_ki = 0.09
        self.angular_kd = 0.66
        self.angular_error_sum = 0.0
        self.angular_last_error = 0.0

        # Define tolerances
        self.tolerance = 0.07  # Tolerance for reaching the goal
        self.angle_tolerance = 0.15  # Tolerance for aligning to the desired angle

        # Timing
        self.last_time = time.time()

        # Observer variables
        self.obstacle_detected = False
        self.obstacle_angle = 0.0

        self.min_z = 0.7
        self.max_z = 1.5
        self.angular_z = 0.0

    def odom_callback(self, msg):
        """
        Callback function to update the robot's position and orientation
        from odometry data.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract orientation (quaternion) and convert to Euler angles
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w]
        _, _, self.theta = euler_from_quaternion(orientation_list)

    def observer_callback(self, msg):
        """
        Callback function to handle obstacle detection data from the observer topic.
        """
        self.obstacle_detected = not bool(msg.x)
        self.obstacle_angle = msg.y

    def reset(self):
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.angular_last_error = 0.0
        self.linear_last_error = 0.0
        self.last_time = time.time()
        self.obstacle_detected = False

    def align_to_angle(self, desired_angle):
        """
        Align the robot to a desired angle after reaching the goal.
        """
        rate = rospy.Rate(10)  # 10 Hz
        vel_msg = Twist()

        while not rospy.is_shutdown() and not self.obstacle_detected:
            angular_error = desired_angle - self.theta
            angular_error = (angular_error + math.pi) % (2 *
                                                         math.pi) - math.pi  # Normalize to [-pi, pi]

            if abs(angular_error) <= self.angle_tolerance:
                rospy.loginfo("Aligned to the desired angle!")
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                break

            current_time = time.time()
            delta_time = current_time - self.last_time
            self.last_time = current_time

            self.angular_error_sum += angular_error * delta_time
            angular_derivative = (
                angular_error - self.angular_last_error) / delta_time
            self.angular_last_error = angular_error

            angular_velocity = (
                self.angular_kp * angular_error +
                self.angular_ki * self.angular_error_sum +
                self.angular_kd * angular_derivative
            )

            vel_msg.angular.z = angular_velocity
            self.velocity_publisher.publish(vel_msg)

            rospy.loginfo(
                f"Current Angle: {math.degrees(self.theta):.2f}°, Desired Angle: {math.degrees(desired_angle):.2f}°")
            rate.sleep()

    def move_to_goal(self, goal_x, goal_y, desired_angle):
        """
        Main method to control the robot to move toward the goal position
        and then align to the desired angle.
        """
        rate = rospy.Rate(10)  # 10 Hz
        vel_msg = Twist()

        while not rospy.is_shutdown():

            if self.obstacle_detected:
                rospy.logwarn("Obstacle detected! Avoiding the obstacle.")

                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                rate.sleep()

                while self.obstacle_detected:
                    self.angular_z = self.min_z + \
                        (self.max_z - self.min_z) * (1 - abs(self.obstacle_angle) / 80)
                    vel_msg.angular.z = -self.angular_z if 0 <= self.obstacle_angle <= 80 else self.angular_z
                    vel_msg.linear.x = 0.2
                    self.velocity_publisher.publish(vel_msg)
                    rate.sleep()

                rospy.loginfo("Obstacle avoided, resuming path to goal.")
                self.linear_error_sum = 0.0
                self.angular_error_sum = 0.0
                continue

            distance = math.sqrt((goal_x - self.x) **
                                 2 + (goal_y - self.y) ** 2)
            angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)

            if distance <= self.tolerance:
                rospy.loginfo("Goal reached!")
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                break

            current_time = time.time()
            delta_time = current_time - self.last_time
            self.last_time = current_time

            linear_error = distance
            self.linear_error_sum += linear_error * delta_time
            linear_derivative = (
                linear_error - self.linear_last_error) / delta_time
            self.linear_last_error = linear_error

            linear_velocity = (
                self.linear_kp * linear_error +
                self.linear_ki * self.linear_error_sum +
                self.linear_kd * linear_derivative
            )

            angular_error = angle_to_goal - self.theta
            angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi

            self.angular_error_sum += angular_error * delta_time
            angular_derivative = (
                angular_error - self.angular_last_error) / delta_time
            self.angular_last_error = angular_error

            angular_velocity = (
                self.angular_kp * angular_error +
                self.angular_ki * self.angular_error_sum +
                self.angular_kd * angular_derivative
            )

            vel_msg.linear.x = min(linear_velocity, 0.4)
            vel_msg.angular.z = angular_velocity - 0.15

            self.velocity_publisher.publish(vel_msg)

            rospy.loginfo(
                f"Current Position: x={self.x:.2f}, y={self.y:.2f}, theta={math.degrees(self.theta):.2f}°")
            rospy.loginfo(
                f"Distance to Goal: {distance:.2f}, Angle to Goal: {math.degrees(angle_to_goal):.2f}°")
            rospy.loginfo(
                f"Linear Velocity: {vel_msg.linear.x:.2f}, Angular Velocity: {vel_msg.angular.z:.2f}")

            rate.sleep()

        self.align_to_angle(desired_angle)


if __name__ == '__main__':
    try:
        controller = RobotController()

        controller.move_to_goal(2.0, 0.0, math.radians(-135))
        controller.reset()
        #rospy.sleep(0.25)

        controller.move_to_goal(1.0, -1.0, math.radians(135))
        controller.reset()
        #rospy.sleep(0.25)

        controller.move_to_goal(0.0, 0.0, math.radians(0))

    except rospy.ROSInterruptException:
        pass
