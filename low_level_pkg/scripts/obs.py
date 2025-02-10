#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math

class ObserverNode:
    def __init__(self):
        rospy.init_node('observer_node')
        self.observer_publisher = rospy.Publisher('/observer', Point, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        point = Point()

        for i, r in enumerate(msg.ranges):
            if r < 0.5 and r > 0:
                angle = math.degrees(msg.angle_min + i * msg.angle_increment)

                if -80 <= angle <= 80:
                    point.x = False
                    point.y = angle
                    point.z = 0.0

                    self.observer_publisher.publish(point)
                    break
        else:
            point.x = True
            point.y = 0.0
            self.observer_publisher.publish(point)

if __name__ == '__main__':
    try:
        observer = ObserverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
