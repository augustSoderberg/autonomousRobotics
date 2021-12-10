#!/usr/bin/env python
# August Soderberg
# asoderberg@brandeis.edu
# Autonomous Robotics
# PA - Line Follower
# 10/23/2020

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from constants import *

def line_found_cb(msg):
    global line_found
    line_found = msg.data

def pid_twist_cb(msg):
    global pid_twist
    pid_twist = msg

def finding_twist_cb(msg):
    global finding_twist
    finding_twist = msg

rospy.init_node('pilot')
line_found_sub = rospy.Subscriber('line_found', Bool, line_found_cb)
pid_twist_sub = rospy.Subscriber('pid_twist', Twist, pid_twist_cb)
finding_twist_sub = rospy.Subscriber('finding_twist', Twist, finding_twist_cb)
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


t = Twist()
pid_twist = Twist()
finding_twist = Twist()
rate = rospy.Rate(10)
line_found = False

while not rospy.is_shutdown():
    if (line_found):
        #Use twist from PID
        t = pid_twist
    else:
        #Use twist from the special line_finder node
        t = finding_twist
    vel_pub.publish(t)
    rate.sleep()