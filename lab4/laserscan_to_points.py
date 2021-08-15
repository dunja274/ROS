#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import math
from visualization_msgs.msg import Marker


class Laser_to_Marker():
    
    def __init__(self):
        self.header = LaserScan().header
        self.ranges = []
        self.angle_min = 0
        self.angle_increment = 0

    def create_marker(self):
        marker = Marker()
        marker.header.frame_id = self.header.frame_id
        marker.header.stamp = self.header.stamp
        marker.type = Marker.POINTS
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = (0., 1., 0.)
        marker.scale.x = 0.5
        marker.scale.y = 0.5

        angles = [self.angle_min + i * self.angle_increment for i in range(0,len(self.ranges))]            
        point_converter = [(r*math.cos(a), r*math.sin(a)) for r, a in zip(self.ranges, angles)]

        for point in point_converter:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            marker.points.append(p)

        return marker

    def callback(self,data):
        self.header = data.header
        self.ranges = data.ranges
        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment

if __name__ == "__main__":

    rospy.init_node('laserscan_to_points')
    
    pub = rospy.Publisher('point_positions', Marker, queue_size=1)
    rate = rospy.Rate(25)  # 25 Hz
    markers = Laser_to_Marker()
    subscriber = rospy.Subscriber('scan', LaserScan, markers.callback)
    while not rospy.is_shutdown():
        pub.publish(markers.create_marker())
        rate.sleep()

