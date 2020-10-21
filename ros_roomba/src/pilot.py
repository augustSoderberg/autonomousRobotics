#!/usr/bin/env python

#August Soderberg
#asoderberg@brandeis.edu   
#PA: ROS Roomba
#Autonomous Robotics
#9/21/2019

#This node is responsible for telling the robot where to drive

import rospy

from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist
from constants import *
import random

#The state is decided and published by the scan_values_handler.
#This function just makes the published value a global variable
def cb_state(msg):
    global state
    state = msg.data

#The direction which the robot should turn is decided and published by the scan_values_handler.
#This function just makes the published value a global variable
def cb_direction(msg):
    global direction
    direction = msg.data

#Set up node and pubs/subs
rospy.init_node('pilot')
sub_state = rospy.Subscriber('state', Int16, cb_state)
sub_direction = rospy.Subscriber('turn_direction', Bool, cb_direction)
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Create a twist and rate object
t = Twist()
rate = rospy.Rate(2)

#Starting states, they will quickly be overwritten
state = WAITING
direction = TURN_RIGHT

#This entire while loop just says if we should be driving, drive forward, if we should be turning,
#turn in the direction we received from scan_values_handler, if we haven't gotten any info yet
#just wait, and anything else print an error.
while not rospy.is_shutdown():
    if state == DRIVING:
        t.angular.z = 0
        t.linear.x = LINEAR_SPEED
        pub.publish(t)
    elif state == SPINNING:
        if direction == TURN_RIGHT:
            t.angular.z = ANGULAR_SPEED*RIGHT
        else:
            t.angular.z = ANGULAR_SPEED*LEFT
        t.linear.x = 0
        while (state != DRIVING):
            pub.publish(t)
        starting_time = rospy.Time.now().to_sec()
        end_time = random.random()*PI / 2 / ANGULAR_SPEED
        while (rospy.Time.now().to_sec() - starting_time < end_time):
            pub.publish(t)
    elif state == WAITING:
        rate.sleep()
    else:
        print("ERROR: State Not Recognized")
        rospy.spin()
    
