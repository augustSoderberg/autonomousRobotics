#!/usr/bin/env python
# August Soderberg
# asoderberg@brandeis.edu
# Autonomous Robotics
# PA - Line Follower
# 10/23/2020

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from constants import *

#Makes global the x coordinate of pixel closest to target pixel
def closest_white_x_cb(msg):
    global closest_white_x
    closest_white_x = msg.data

#Makes global the y coordinate of pixel closest to target pixel
def closest_white_y_cb(msg):
    global closest_white_y
    closest_white_y = msg.data

rospy.init_node('line_finder')
closest_white_x_sub = rospy.Subscriber('closest_white_x', Int32, closest_white_x_cb)
closest_white_y_sub = rospy.Subscriber('closest_white_y', Int32, closest_white_y_cb)
pub = rospy.Publisher('finding_twist', Twist, queue_size=1)

t = Twist()
rate = rospy.Rate(10)
closest_white_x = -1
closest_white_y = -1

while not rospy.is_shutdown():
    #If it can't see any white on screen, rotate
    if (closest_white_x < 0):
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED
    #If the white is too far left, turn left
    elif (closest_white_x < IMAGE_WIDTH // 2 - EPSILON):
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED
    #If the white is too far right, turn right
    elif (closest_white_x > IMAGE_WIDTH // 2 + EPSILON):
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED * -1
    #If the white is too far away, drive towards it
    elif (closest_white_y < BOTTOM_HORIZONTAL - EPSILON):
        t.linear.x = LINEAR_SPEED
        t.angular.z = 0
    #Rotates until the PID takes over
    else:
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED
    pub.publish(t)
    rate.sleep()