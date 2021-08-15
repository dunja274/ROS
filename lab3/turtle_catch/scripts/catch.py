#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from geometry_msgs.msg import Point


class Turtle:

    def pose_callback(self, data):
        self.pose = data
        self.pose.x = self.pose.x
        self.pose.y = self.pose.y

    def __init__(self,name):
        self.vel_pub = rospy.Publisher('/'+name +'/cmd_vel', Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber('/'+name +'/pose', Pose, self.pose_callback)
        
        self.cmd_vel = Twist()
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def get_distance(self, goal_pose):
        distance = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
        return  distance
	
    def get_ang_distance(self, goal_pose):
        ang_distance = (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)-self.pose.theta)
        return 5 * ang_distance

    def catch(self, goal_pose):
     
        self.cmd_vel.linear.x = self.get_distance(goal_pose)
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0

        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = self.get_ang_distance(goal_pose)

        self.vel_pub.publish(self.cmd_vel)
        self.rate.sleep()

        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.vel_pub.publish(self.cmd_vel)
        return
        

if __name__ == '__main__':
    rospy.init_node('catch', anonymous=True)
    pub = rospy.Publisher('turtles_distance', Point, queue_size=1)
    
    try:
        t2 = Turtle('turtle1')
        t1 = Turtle('turtle2')
        
        while not rospy.is_shutdown():
            while t2.get_distance(t1.pose):
            	pub.publish(abs(t2.pose.x-t1.pose.x),abs(t2.pose.y-t1.pose.y),0)
            	t2.catch(t1.pose)
       
    except rospy.ROSInterruptException:
        pass
