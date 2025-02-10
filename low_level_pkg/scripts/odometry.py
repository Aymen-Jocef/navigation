#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, TransformStamped
import math

class OdometryPublisher:
    def __init__(self):
        # ROS Node initialization
        rospy.init_node('odometry_publisher', anonymous=True)
        
        # Initialize parameters
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # RPM variables
        self.last_right_rpm = 0.0
        self.last_left_rpm = 0.0

        # Time tracking
        self.last_time = rospy.Time.now()

        # Robot parameters
        self.wheel_diameter = 0.067  # meters
        self.wheel_distance = 0.325   # meters (wheel base)
        self.rpm_to_vel = math.pi * self.wheel_diameter / 60  # Convert RPM to m/s

        # Publishers and Subscribers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.right_wheel_sub = rospy.Subscriber('right_wheel_rpm', Float32, self.rightWheelCallback)
        self.left_wheel_sub = rospy.Subscriber('left_wheel_rpm', Float32, self.leftWheelCallback)

        # Initialize TF broadcaster
       # self.odom_broadcaster = tf.TransformBroadcaster()

    # Callback for right wheel RPM
    def rightWheelCallback(self, msg):
        self.last_right_rpm = msg.data

    # Callback for left wheel RPM
    def leftWheelCallback(self, msg):
        self.last_left_rpm = msg.data

    # Method to compute and publish odometry
    def publishOdometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Convert RPM to velocities
        right_vel = self.last_right_rpm * self.rpm_to_vel
        left_vel = self.last_left_rpm * self.rpm_to_vel

        # Compute robot velocities (linear and angular)
        self.vx = (right_vel + left_vel) / 2.0
        self.vth = (right_vel - left_vel) / self.wheel_distance

        # Update robot position
        delta_x = self.vx * math.cos(self.th) * dt
        delta_y = self.vx * math.sin(self.th) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Create quaternion for yaw (orientation)
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # Publish the transform
       # odom_trans = TransformStamped()
       # odom_trans.header.stamp = current_time
       # odom_trans.header.frame_id = "odom"
       # odom_trans.child_frame_id = "base_link"
        
       # odom_trans.transform.translation.x = self.x
       # odom_trans.transform.translation.y = self.y
       # odom_trans.transform.translation.z = 0.0
       # odom_trans.transform.rotation.x = odom_quat[0]
       # odom_trans.transform.rotation.y = odom_quat[1]
       # odom_trans.transform.rotation.z = odom_quat[2]
        #odom_trans.transform.rotation.w = odom_quat[3]
        
        # Send the transform
       # self.odom_broadcaster.sendTransform(
           # (odom_trans.transform.translation.x, odom_trans.transform.translation.y, odom_trans.transform.translation.z),
            #(odom_trans.transform.rotation.x, odom_trans.transform.rotation.y, odom_trans.transform.rotation.z, odom_trans.transform.rotation.w),
            #odom_trans.header.stamp,
            #odom_trans.child_frame_id,
            #odom_trans.header.frame_id
       # )

        # Create and publish the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        
        # Position and orientation
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        # Velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Update time
        self.last_time = current_time

    # Spin loop
    def spin(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publishOdometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        odom_pub = OdometryPublisher()
        odom_pub.spin()
    except rospy.ROSInterruptException:
        pass
