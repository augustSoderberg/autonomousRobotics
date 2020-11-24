#!/usr/bin/env python
#August Soderberg
#asoderberg@brandeis.edu
#Wall Follower
#Autonomous Robotics
#10/4/2020

#This processes all of the scan values

#Imports
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from constants import *

#Process all the data from the LIDAR
def cb(msg):
    index_min = 0
    min = msg.ranges[0]
    #Find the minimum value and the index at which it occurs
    for i in range(0, 360):
        if (msg.ranges[i] < min):
            min = msg.ranges[i]
            index_min = i
    #Find the correct state the robot should be in
    state = FOLLOWING
    if (min == float('inf')):
        state = WANDERING
    elif (index_min > 270 + ANGLE_DIFF_REQUIRING_LINING_UP or index_min < 270 - ANGLE_DIFF_REQUIRING_LINING_UP):
        state = LINING_UP
    #publish
    pub_state.publish(state)
    pub_min_angle.publish(index_min)
    if (min > msg.range_max):
        pub_diff_dist.publish(msg.range_max)
    else:
        pub_diff_dist.publish(min - FOLLOWING_DIST)



#Init node
rospy.init_node('scan_values_handler')

#Subscriber for LIDAR
sub = rospy.Subscriber('scan', LaserScan, cb)
#Publisher for state, min angle, and diff_dist
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
pub_min_angle = rospy.Publisher('min_angle', Int16, queue_size = 1)
pub_diff_dist = rospy.Publisher('diff_dist', Float32, queue_size = 1) 

#Rate object
rate = rospy.Rate(10)

#Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 