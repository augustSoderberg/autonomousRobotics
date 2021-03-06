#!/usr/bin/env python
# August Soderberg
# asoderberg@brandeis.edu
# Autonomous Robotics
# PA - Line Follower
# 10/23/2020

import rospy
from constants import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64

def center_cb(msg):
    global center_diff
    center_diff = IMAGE_WIDTH // 2 - msg.data
    #Memory of values used in integral caluclation
    global integral_diffs
    for i in range(0, len(integral_diffs) - 1):
        integral_diffs[i + 1] = integral_diffs[i]
    integral_diffs[0] = center_diff

def slope_cb(msg):
    global slope
    slope = msg.data

rospy.init_node('PID')
pub = rospy.Publisher('pid_twist', Twist, queue_size=1)
center_sub = rospy.Subscriber('center', Int32, center_cb)
slope_sub = rospy.Subscriber('slope', Float64, slope_cb)

t = Twist()
rate = rospy.Rate(10)
center_diff = 0
slope = 0
integral_diffs = [0.0]*INTEGRAL_MEMORY_LENGTH

while not rospy.is_shutdown():
    t.linear.x = LINEAR_SPEED
    #Angular is based on integral, derivative, and proportional error
    t.angular.z = ANGULAR_SPEED*(P_TUNER*(center_diff) + I_TUNER*sum(integral_diffs)/len(integral_diffs) + D_TUNER*slope)
    pub.publish(t)
    rate.sleep()