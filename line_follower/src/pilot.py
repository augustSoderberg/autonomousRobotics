#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

def cb(msg):
    print(len(msg.data[0]))


rospy.init_node('target_publisher')
sub = rospy.Subscriber('masked_img', Image, cb)
rospy.spin()
