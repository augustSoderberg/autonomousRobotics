#!/usr/bin/env python
# August Soderberg
# asoderberg@brandeis.edu
# Autonomous Robotics
# PA - Line Follower
# 10/23/2020

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from constants import *

def odom_cb(msg):
    global x, y
    x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

rospy.init_node('gps_faker')
sub = rospy.Subscriber('odom', Odometry, odom_cb)
pub = rospy.Publisher('gps_data', Float64MultiArray, queue_size=1)

rate = rospy.Rate(10)
array = Float64MultiArray()
y = 0
x = 0

while not rospy.is_shutdown():
    array.data = [y, x, 4]
    pub.publish(array)
    rate.sleep()