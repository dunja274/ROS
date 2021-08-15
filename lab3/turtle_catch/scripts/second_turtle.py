#!/usr/bin/env python3

import rospy
import turtlesim.srv



if __name__ == '__main__':
    rospy.init_node('second_turtle', anonymous=True)
    
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')
    
