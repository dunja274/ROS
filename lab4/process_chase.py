#!/usr/bin/env python3
import rosbag
import sys
from math import pow, sqrt

class Turtle:
    name = ''
    covered_dist = 0
    avg_vel = 0
    duration = 0

    def __init__(self, name, covered_dist, duration):
        self.name = name
        self.covered_dist = covered_dist
        self.duration = duration
        self.avg_vel = self.covered_dist/self.duration

def calc_distance(poses_list):
    previous_pose = ()
    distance = 0

    for pose in poses_list:
        if len(previous_pose):
            distance += sqrt(pow((pose[0]-previous_pose[0]),2)+pow((pose[1]-previous_pose[1]),2))
        previous_pose = pose
    
    return distance

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f'Usage: {sys.argv[0]} {sys.argv[1]}')
        sys.exit()

    inbag_filename = sys.argv[1]
    outbag_filename = 'processed_chase.bag'

    bag = rosbag.Bag(inbag_filename)

    runner_turtle = 'turtle2'
    chaser_turtle = 'turtle1'

    print(f'Processing input bagfile: {inbag_filename}')
    msg_counter = 0
    runner_pose = []
    chaser_pose = []

    with rosbag.Bag(outbag_filename, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inbag_filename, 'r').read_messages(topics=[f'/{runner_turtle}/pose',f'/{chaser_turtle}/pose']):
            if runner_turtle in topic:
                runner_pose.append((msg.x,msg.y,t.to_time()))
                outbag.write('/runner/pose', msg, t)
            else:
                chaser_pose.append((msg.x,msg.y,t.to_time()))
                outbag.write('/chaser/pose', msg, t)
            msg_counter += 1

    turtle1 = Turtle('Runner', calc_distance(runner_pose), runner_pose[-1][2] - runner_pose[0][2])
    turtle2 = Turtle('Chaser', calc_distance(chaser_pose), chaser_pose[-1][2] - chaser_pose[0][2])
    turtles = [turtle1, turtle2]

    for t in turtles:
        print(t.name)
        print(f'  Covered distance: {t.covered_dist:.2f} m')
        print(f'  Average velocity: {t.avg_vel:.2f} m/s')

    print(f'Chase duration: {turtles[1].duration:.2f} s')
    print(f'Wrote {msg_counter} messages to {outbag_filename}')
    
