#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from constants import *

def closest_white_cb(msg):
    global closest_white
    closest_white = msg.data


rospy.init_node('line_finder')
closest_white_sub = rospy.Subscriber('closest_white', Int32MultiArray, closest_white_cb)
pub = rospy.Publisher('finding_twist', Twist, queue_size=1)

t = Twist()
rate = rospy.Rate(10)
closest_white = [-1, -1]

while not rospy.is_shutdown():
    if (closest_white[0] < 0):
        t.linear = 0
        t.angular = ANGULAR_SPEED
    elif (closest_white[1] < IMAGE_WIDTH // 2 - EPSILON):
        t.linear = 0
        t.angular = ANGULAR_SPEED
    elif (closest_white[1] > IMAGE_WIDTH // 2 + EPSILON):
        t.linear = 0
        t.angular = ANGULAR_SPEED * -1
    else:
        t.linear = LINEAR_SPEED
        t.angular = 0
    pub.publish(t)
    rate.sleep()