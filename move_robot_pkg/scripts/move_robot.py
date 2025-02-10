#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi

class MoveRobot:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        
        self.position = None
        self.yaw = None
        self.initial_position = None
        self.target_distance = 1  # meters
        self.target_angle = 90.0  # degrees
        self.current_state = "move_forward"
        self.i = 0
        
        self.rate = rospy.Rate(10)
        
    def odom_callback(self, msg):
        # Extract the robot's position and orientation (yaw angle)
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y**2 + orientation_q.z**2)
        self.yaw = atan2(siny_cosp, cosy_cosp) * (180.0 / pi)  # Convert to degrees
        
        if self.initial_position is None:
            self.initial_position = self.position

    def distance_traveled(self):
        if self.position and self.initial_position:
            return sqrt((self.position.x - self.initial_position.x)**2 +
                        (self.position.y - self.initial_position.y)**2)
        return 0.0

    def rotate_robot(self, angular_speed=30):
        twist = Twist()
        twist.angular.z = angular_speed * (pi / 180.0)  # Convert to radians/sec
        self.cmd_vel_pub.publish(twist)

    def move_forward(self, linear_speed=0.3):
        twist = Twist()
        twist.linear.x = linear_speed
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()  # Zero velocities to stop the robot
        self.cmd_vel_pub.publish(twist)

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.current_state == "move_forward":
                self.move_forward()
                if self.distance_traveled() >= self.target_distance:
                    self.i += 1 
                    if self.i == 5 :
                        self.i = 1
                    self.stop_robot()
                    rospy.sleep(1.0)  # Wait for the robot to stabilize
                    self.current_state = "rotate"
                    self.initial_position = None  # Reset for next move
                    angle = self.target_angle * self.i

            elif self.current_state == "rotate":
                if self.yaw is not None:
                    print("yaw = ", self.yaw)
                    # Rotate until the desired angle is reached
                    self.rotate_robot()
                    if abs((self.yaw + 360) % 360 - angle) < 3.0:  # Allowable error
                        self.stop_robot()
                        rospy.sleep(1.0)
                        self.current_state = "move_forward"

            elif self.current_state == "stop":
                self.stop_robot()
                break

            self.rate.sleep()

if __name__ == "__main__":
    try:
        robot = MoveRobot()
        robot.control_loop()
    except rospy.ROSInterruptException:
        pass
