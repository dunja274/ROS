#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import LaserScan
import math
import tf2_ros
from visualization_msgs.msg import Marker


class Laser_to_Marker():
    
    def __init__(self):
        self.header = LaserScan().header
        self.ranges = []
        self.angle_min = 0
        self.angle_increment = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.marker = Marker()
        self.transform = TransformStamped()
        self.marker.header.frame_id = global_frame
        self.count = 0
        self.pub = rospy.Publisher('point_positions', Marker, queue_size=1)

    def create_marker(self):
        self.marker.type = Marker.POINTS
        self.marker.color.a = 1.0
        self.marker.color.r, self.marker.color.g, self.marker.color.b = (0., 1., 1.)
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        orientation = 2*math.atan2(self.transform.transform.rotation.z,self.transform.transform.rotation.w)

        angles = [self.angle_min + i * self.angle_increment + orientation for i in range(0,len(self.ranges))]            
        point_converter = [(r*math.cos(a), r*math.sin(a)) for r, a in zip(self.ranges, angles)]

        for point in point_converter:
            p = Point()
            p.x = point[0] + self.transform.transform.translation.x
            p.y = point[1] + self.transform.transform.translation.y
            p.z = self.transform.transform.translation.z
            self.marker.points.append(p)

        return self.marker

    def callback(self,data):
        if data.header.stamp < self.marker.header.stamp:
            print('Timestamp has jumped backwards, clearing the buffer.')
            self.marker.header.stamp = data.header.stamp
            self.marker.points.clear()
            self.tf_buffer.clear()
            return
        self.count+=1
        if accumulate_points == True:
            if self.count < accumulate_every_n:
                return
        self.count=0
        if accumulate_points == False:
            self.marker.points = []
        self.header = data.header
        self.marker.header.stamp = self.header.stamp
        self.ranges = data.ranges
        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment
        try:
            buf = self.tf_buffer
            self.transform = buf.lookup_transform(global_frame, data.header.frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            print('Tf exception')
            return
        self.pub.publish(self.create_marker())
        

if __name__ == "__main__":

    rospy.init_node('laserscan_to_points')
    global_frame = rospy.get_param('~global_frame', 'map')
    accumulate_points = rospy.get_param('~accumulate_points', False)
    accumulate_every_n = rospy.get_param('~accumulate_every_n', 50)
    print('Starting the laser scan visualizer node.')
    print(f'global_frame: {global_frame}, accumulate_points: {accumulate_points}, accumulate_every_n: {accumulate_every_n}')
    rate = rospy.Rate(25)  # 25 Hz
    markers = Laser_to_Marker()
    subscriber = rospy.Subscriber('scan', LaserScan, markers.callback)
    while not rospy.is_shutdown():
        rospy.spin()

        

