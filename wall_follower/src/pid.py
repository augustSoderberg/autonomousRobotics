#!/usr/bin/env python
#August Soderberg
#asoderberg@brandeis.edu
#Wall Follower
#Autonomous Robotics
#10/4/2020

#This is a PID controller to control following state

#Imports
import rospy
from constants import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

#Make the angle of the minimum lidar value global
def cb_min_angle(msg):
    global min_angle
    min_angle = msg.data

#Make the difference between the expected distance and the true distance global
def cb_diff_dist(msg):
    global diffs
    for i in range(0, 4):
        diffs[i + 1] = diffs[i]
    diffs[0] = msg.data
    global diff_dist
    diff_dist = msg.data


#Init node
rospy.init_node('pid')

#Create publisher for suggested twist objects
pub = rospy.Publisher('twist', Twist, queue_size = 1)
#Subscriber for min_angle and diff_dist
sub_min_angle = rospy.Subscriber('min_angle', Int16, cb_min_angle)
sub_diff_dist = rospy.Subscriber('diff_dist', Float32, cb_diff_dist)

#Twist and rate object
t = Twist()
rate = rospy.Rate(10)

#List to hold the 5 most recent diff_dist values to be used by the integral calculator
diffs = [0.0, 0.0, 0.0, 0.0, 0.0]
#Set starting values for linear, angular speed, and min_angle and diff_dist
t.linear.x = LINEAR_SPEED
t.angular.z = 0
min_angle = 270
diff_dist = 0


while not rospy.is_shutdown():
    #Set the proportional component of the PID controller
    p_component = -1 * diff_dist
    if p_component > 1:
        p_component = 1
    elif p_component < -1:
        p_component = -1
    #Set the derivative component of the PID controller
    d_component = (min_angle - 270) / ANGLE_DIFF_REQUIRING_LINING_UP
    #Set the integral component of the PID controller
    i_component = -1 * sum(diffs) / len(diffs)
    #Add them all together, multiplied by their respective tuning values, and multiply everything
    #by the angular velocity
    t.angular.z = ANGULAR_SPEED * (P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component)
    #Linear speed never changes in this state
    t.linear.x = LINEAR_SPEED
    #Publish the twist to the driver
    pub.publish(t)
    rate.sleep()