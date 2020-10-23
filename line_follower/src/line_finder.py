#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

rospy.init_node('line_finder')
closest_white_sub = rospy.Subscriber('closest_white', Int32MultiArray)
pub = rospy.Publisher('finding_twist', Twist, queue_size=1)