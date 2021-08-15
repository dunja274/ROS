#!/usr/bin/env python3
import rosbag
import sys
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class Visualizer:

    def __init__(self):
        self.marker = Marker()
        self.marker.header.stamp = rospy.Time(0)
        self.marker.type = Marker.LINE_STRIP
        self.marker.color.a = 1.0
        self.marker.color.r, self.marker.color.g, self.marker.color.b = (116, 59, 97)
        self.marker.scale.x = 0.5
        self.marker.pose.orientation.w = 1
        self.pub = rospy.Publisher('robot_positions', Marker, queue_size=1)

    def callback(self, data):
        for transform in data.transforms:
            if transform.child_frame_id == child_frame_id and transform.header.frame_id == frame_id:
                translation = transform.transform.translation
                p = Point()
                p.x = translation.x
                p.y = translation.y
                self.marker.header.frame_id = transform.header.frame_id
                if self.marker.header.stamp > transform.header.stamp:
                    self.marker.points = []
                    print('Timestamp has jumped backwards, clearing the trajectory.')
                self.marker.header.stamp = transform.header.stamp
                self.marker.points.append(p)
                self.pub.publish(self.marker)

    

if __name__ == "__main__":
    rospy.init_node('trajectory_visualizer', anonymous=False) 
    
    frame_id = rospy.get_param('~frame_id', 'map')
    child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
    print('Starting the trajectory visualizer node.')
    print(f'frame_id: {frame_id}, child_frame_id: {child_frame_id}')
    visualizer = Visualizer()
    subscriber = rospy.Subscriber('tf', TFMessage, visualizer.callback)
    while not rospy.is_shutdown():
        rospy.spin()