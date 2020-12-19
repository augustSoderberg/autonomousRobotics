#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from constants import *


# COSI 119A
# PA8
# Joseph Pickens
# October 30, 2020
# ----------------
# Description: 
# The lidar_twist node subscribes to the scan topic and publishes a Twist message for the purpose of
# steering around obstacles

# callback function for the laser scan subscriber
def scan_cb(msg):
    front_scan = msg.ranges[(len(msg.ranges) - ANGLE_WIDTH_FROM_CENTER):len(msg.ranges)] + msg.ranges[0:ANGLE_WIDTH_FROM_CENTER]
    min_distance = min(front_scan)
    index = len(msg.ranges) // 2
    if min_distance != float("inf"):
        for i, distance in enumerate(front_scan):
            if distance == min_distance:
                index = i - ANGLE_WIDTH_FROM_CENTER
    # create Twist message and publish to lidar topic
    t = Twist()
    t.linear.x = LINEAR_SPEED
    obstacle_found = True
    if index == (len(msg.ranges) // 2) or min_distance > THRESHOLD_DISTANCE:
        t.angular.z = 0
        obstacle_found = False
    elif index <= 0:
        t.angular.z = ANGULAR_SPEED
    else:
        t.angular.z = -1*ANGULAR_SPEED
    twist_pub.publish(t)
    obstacle_pub.publish(obstacle_found)

rospy.init_node('lidar_twist')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)
twist_pub = rospy.Publisher('lidar_twist', Twist, queue_size=1)
obstacle_pub = rospy.Publisher('obstacle_found', Bool, queue_size=1)
rospy.spin()
