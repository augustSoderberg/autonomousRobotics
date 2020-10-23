#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from constants import *

def closest_white_x_cb(msg):
    global closest_white
    closest_white[0] = msg.data

def closest_white_y_cb(msg):
    global closest_white
    closest_white[1] = msg.data

rospy.init_node('line_finder')
closest_white_x_sub = rospy.Subscriber('closest_white_x', Int32, closest_white_y_cb)
closest_white_y_sub = rospy.Subscriber('closest_white_y', Int32, closest_white_y_cb)
pub = rospy.Publisher('finding_twist', Twist, queue_size=1)

t = Twist()
rate = rospy.Rate(10)
closest_white = (-1, -1)

while not rospy.is_shutdown():
    if (closest_white[0] < 0):
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED
    elif (closest_white[1] < IMAGE_WIDTH // 2 - EPSILON):
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED
    elif (closest_white[1] > IMAGE_WIDTH // 2 + EPSILON):
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED * -1
    else:
        t.linear.x = LINEAR_SPEED
        t.angular.z = 0
    pub.publish(t)
    rate.sleep()