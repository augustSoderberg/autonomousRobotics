#!/usr/bin/env python
#August Soderberg
#asoderberg@brandeis.edu
#Wall Follower
#Autonomous Robotics
#10/4/2020

#This node effectively drives the robot based on information from the other nodes.

#Imports
import rospy
from constants import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

#Makes the state message global
def cb_state(msg):
    global state
    state = msg.data

#Makes the angle of the minimum lidar value global
def cb_min_angle(msg):
    global min_angle
    min_angle = msg.data

#Makes the difference between real distance and following distance global
def cb_diff_dist(msg):
    global diff_dist
    diff_dist = msg.data

#Makes the twist object sent from PID global
def cb_twist(msg):
    global t_pid
    t_pid = msg

#Init node
rospy.init_node('driver')

#Make publisher for cmd_vel
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Subscribers for state of robot, angle of minimum LIDAR value, difference in the following distance
#and true distance, and the Twist object published by the PID controller
sub_state = rospy.Subscriber('state', Int16, cb_state)
sub_min_angle = rospy.Subscriber('min_angle', Int16, cb_min_angle)
sub_diff_dist = rospy.Subscriber('diff_dist', Float32, cb_diff_dist)
sub_twist = rospy.Subscriber('twist', Twist, cb_twist)

#Rate object
rate = rospy.Rate(10)

#Create two twist variable, one is modified here, one is copied from the PID messages
t = Twist()
t_pid = Twist()

#Starting state is WANDERING
state = WANDERING
#Starting angle of smallest LIDAR range is 270
min_angle = 270
#Difference in the desired and expected distances
diff_dist = 0



print("STARTING")

while not rospy.is_shutdown():
    print("IN WHILE: ", state)
    #If wandering
    if (state == WANDERING):
        t.angular.z = 0
        #While you're more than desired distance from an object just keep driving straight
        while (diff_dist > 0):
            print("IN WANDERING")
            t.linear.x = LINEAR_SPEED
            pub_vel.publish(t)
            rate.sleep()
        t.linear.x = 0
        pub_vel.publish(t)
    #If lining up
    elif (state == LINING_UP):
        t.linear.x = 0
        #Keep turning in the efficient direction until the robot more or less has the wall on the
        #right hand side
        while (min_angle > 270 + ANGLE_ERROR or min_angle < 270 - ANGLE_ERROR):
            print(min_angle)
            angle_multiplier = 1
            if (min_angle >= 90 and min_angle < 270):
                angle_multiplier = -1
            t.angular.z = ANGULAR_SPEED*angle_multiplier
            pub_vel.publish(t)
            rate.sleep()
        t.angular.z = 0
        pub_vel.publish(t)
    #If following the wall
    elif (state == FOLLOWING):
        #Publish the velocity message suggested by the PID controller
        pub_vel.publish(t_pid)
    else:
        print("STATE NOT FOUND")
    rate.sleep()


