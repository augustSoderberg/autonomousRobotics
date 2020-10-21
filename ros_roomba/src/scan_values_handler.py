#!/usr/bin/env python

#August Soderberg
#asoderberg@brandeis.edu   
#PA: ROS Roomba
#Autonomous Robotics
#9/21/2019

#This node is responsible for parcing all data from the robot and sending values to pilot which it
#will use to determine how to drive the robot
import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Bool
from constants import *

#This will assume the robot scans the lidar data in the correct secor determined by a constant
#to see if there is anything in the way, if there is something in the way it tells the robot
#to spin in the direction which will more quickly become clear.
def cb_scan(msg):
    state = DRIVING
    for i in range(360 - FRONT_SECTOR_DEGREES // 2, 360 + FRONT_SECTOR_DEGREES // 2):
        #Checks to see if we are too close to something and makes sure the value is valid.
        if msg.ranges[i % 360] < MIN_DIST and msg.ranges[i % 360] >= msg.range_min:
            state = SPINNING
    pub_state.publish(state)
    if state == SPINNING:
        pub_direction.publish(msg.ranges[(360 + FRONT_SECTOR_DEGREES // 2) % 360] < msg.ranges[(360 - FRONT_SECTOR_DEGREES // 2) % 360])

#Initialize node
rospy.init_node('scan_values_handler')

#Set up pubs/subs and a rate object
sub = rospy.Subscriber('scan', LaserScan, cb_scan)
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
pub_direction = rospy.Publisher('turn_direction', Bool, queue_size = 1)
rate = rospy.Rate(2)

#This was used during debugging and is now useless.
while not rospy.is_shutdown():
    rate.sleep()

