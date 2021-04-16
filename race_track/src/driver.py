#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# COSI 119A
# PA8
# Joseph Pickens
# October 30, 2020
# ----------------
# Description:
# The driver node subscribes to the lidar_vel and line_image topics and publishes to cmd_vel
# to follow a racetrack and steer around obstacles.

# callback function for line_image subscriber
def line_cb(msg):
    global line_found
    line_found = msg.data

def obstacle_cb(msg):
    global obstacle_found
    obstacle_found = msg.data

# callback function for pid_twist subscriber
def pid_twist_cb(msg):
    global t_pid
    t_pid = msg

# callback function for lidar_twist subscriber
def lidar_twist_cb(msg):
    global t_lidar
    t_lidar = msg

line_found = False
obstacle_found = True
print_trigger = [True, True]
t_pid = Twist()
t_lidar = Twist()
t = Twist()

vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
line_image_sub = rospy.Subscriber('line_found', Bool, line_cb)
obstacle_sub = rospy.Subscriber('obstacle_found', Bool, obstacle_cb)
lidar_twist_sub = rospy.Subscriber('lidar_twist', Twist, lidar_twist_cb)
pid_sub = rospy.Subscriber('pid_twist', Twist, pid_twist_cb)

rospy.init_node('driver')

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if line_found and not obstacle_found:
        t = t_pid
        if print_trigger[0]:
            print("Following racetrack...")
            print_trigger[0] = False
            print_trigger[1] = True
    else:
        t = t_lidar
        if print_trigger[1]:
            print("Avoiding obstacle...")
            print_trigger[1] = False
            print_trigger[0] = True
    vel_pub.publish(t)
    rate.sleep()
