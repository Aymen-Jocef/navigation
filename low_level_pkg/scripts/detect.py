#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

DETECTION_RANGE = 0.3

def callback(scan_data, pub):
    distances = scan_data.ranges
    angle_min = scan_data.angle_min
    angle_increment = scan_data.angle_increment

    object_detected = False
    for i in range(0, len(distances), 2):
        distance = distances[i]
        
        if distance < DETECTION_RANGE and distance > scan_data.range_min:
            angle = angle_min + i * angle_increment
            angle_degrees = math.degrees(angle)
            
            if -90 <= angle_degrees <= 90:
                object_detected = True
                rospy.loginfo(f"Obstacle detected at angle {angle_degrees:.2f}° and distance {distance:.2f}m.")
                break

    if object_detected:
        pub.publish(True)
    else:
        pub.publish(False)

def laser_scan_listener():
    rospy.init_node('laser_detection', anonymous=True)
    pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback, pub)
    rospy.spin()

if __name__ == '__main__':
    try:
        laser_scan_listener()
    except rospy.ROSInterruptException:
        pass
