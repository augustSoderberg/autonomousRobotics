#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

def cb(msg):
    min = msg.ranges[0]
    for i in range(0, 360):
        if min > msg.ranges[i]:
            min = msg.ranges[i]
    pub.publish(min)

rospy.init_node('lidar_helper')
pub = rospy.Publisher('lidar', Float32, queue_size = 1)
sub = rospy.Subscriber('/scan', LaserScan, cb)
rospy.spin()