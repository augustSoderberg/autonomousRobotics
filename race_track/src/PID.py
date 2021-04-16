#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from constants import *

# COSI 119A
# PA8
# August Soderberg
# November 1, 2020
# ----------------
# Description:
# This is the PID controller which receives the data form the image_parser and publishes Twists to
# the driver.

# Stores the last 10 proportional errors to determine the proportional error, derivative error,
# and integral error
def cb(msg):
    global p_comp
    p_comp = msg.data
    global mem
    for i in range(MEMORY_LENGTH - 1, 0, -1):
        mem[i] = mem[i - 1]
    mem[0] = p_comp
    global d_comp
    d_comp = mem[0] - mem[1]



rospy.init_node('pid')
p_sub = rospy.Subscriber("p_comp", Int32, cb)
pub = rospy.Publisher("pid_twist", Twist, queue_size=1)

#Blank memory array and default values
mem = [0]*MEMORY_LENGTH
p_comp = 0
d_comp = 0

t = Twist()
rate = rospy.Rate(10)

t.linear.x = LINEAR_SPEED

while not rospy.is_shutdown():
    t.angular.z = P_TUNER*p_comp + D_TUNER*d_comp + I_TUNER*sum(mem)/len(mem)
    if t.angular.z > ANGULAR_SPEED:
        t.angular.z = ANGULAR_SPEED
    elif t.angular.z < -1*ANGULAR_SPEED:
        t.angular.z = -1*ANGULAR_SPEED
    pub.publish(t)
    rate.sleep()