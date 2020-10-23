#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from constants import *

def closest_white_x_cb(msg):
    global closest_white_x
    closest_white_x = msg.data

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
    if (closest_white_x < 0):
        print("finding")
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED
    elif (closest_white_x < IMAGE_WIDTH // 2 - EPSILON):
        print("adjusting left")
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED
    elif (closest_white_x > IMAGE_WIDTH // 2 + EPSILON):
        print("adjusting right")
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED * -1
    elif (closest_white_y < BOTTOM_HORIZONTAL - EPSILON):
        print("getting closer")
        t.linear.x = LINEAR_SPEED
        t.angular.z = 0
    else:
        print("lining up")
        t.linear.x = 0
        t.angular.z = ANGULAR_SPEED
    pub.publish(t)
    rate.sleep()