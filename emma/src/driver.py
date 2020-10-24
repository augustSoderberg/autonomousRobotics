#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def cb(msg):
    global T 
    if msg.data > 0.5:
        T.linear.x = 0.3
    else:
        T.linear.x = 0

rospy.init_node('driver')
sub = rospy.Subscriber('lidar', Float32, cb)
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
rate = rospy.Rate(10)

T = Twist()

while not rospy.is_shutdown():
    pub.publish(T)
    rate.sleep()
