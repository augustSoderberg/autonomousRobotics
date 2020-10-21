#!/usr/bin/env python

#August Soderberg
#asoderberg@brandeis.edu   
#PA: Teach a robot to dance!
#Autonomous Robotics
#9/29/2020

#This node is responsible for parcing all data from the robot and sending values to dancer which
#will determine how to act around an obsticle
import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

#Constants
FRONT_SECTOR_DEGREES = 45
MIN_DIST = 0.35

#This will look through the data to see if there are any obstructions ahead or any obstructions
#around the robot at all and publish them.
def cb_scan(msg):
    wall_in_front = False
    wall_nearby = False
    for i in range(360 - FRONT_SECTOR_DEGREES // 2, 360 + FRONT_SECTOR_DEGREES // 2):
        #Checks to see if we are too close to something and makes sure the value is valid.
        if msg.ranges[i % 360] < MIN_DIST and msg.ranges[i % 360] >= msg.range_min:
            wall_in_front = True
    pub_front.publish(wall_in_front)

    for i in range(0, 360):
        #Checks to see if we are too close to something and makes sure the value is valid.
        if msg.ranges[i] < MIN_DIST and msg.ranges[i] >= msg.range_min:
            wall_nearby = True
    pub_near.publish(wall_nearby)

#Initialize node
rospy.init_node('scan_values_handler')

#Set up pubs/subs and a rate object
sub = rospy.Subscriber('scan', LaserScan, cb_scan)
pub_front = rospy.Publisher('wall_in_front', Bool, queue_size = 1)
pub_near = rospy.Publisher('wall_nearby', Bool, queue_size = 1)
rate = rospy.Rate(2)

#This was used during debugging and is now useless.
while not rospy.is_shutdown():
    rate.sleep()

