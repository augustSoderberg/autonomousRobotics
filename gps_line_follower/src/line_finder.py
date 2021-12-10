#!/usr/bin/env python
# August Soderberg
# asoderberg@brandeis.edu
# Autonomous Robotics
# PA - Line Follower
# 10/23/2020

import rospy, math
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool
from constants import *

def gps_cb(msg):
    global lat, long, fix, update_time, old_lat, old_long
    if fix == FIXED and (rospy.Time.now() - update_time).to_sec() > 0.5:
        old_lat, old_long = lat, long
        update_time = rospy.Time.now()
    lat, long, fix = msg.data

rospy.init_node('line_finder')
update_time = rospy.Time.now()

gps_data_sub = rospy.Subscriber('gps_data', Float64MultiArray, gps_cb)
halt_pub = rospy.Publisher('should_halt', Bool, queue_size=1)
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

line_start = np.array([START_LONG, START_LAT])
line_end = np.array([END_LONG, END_LAT])
rate = rospy.Rate(5)
lat, long, fix = 0, 0, FLOAT
old_lat, old_long = 0, 0
line = []
t = Twist()

x, y = np.subtract(line_end, line_start)
line_angle = np.arctan(y / x)

while not rospy.is_shutdown():
    if fix == FLOAT:
        t.linear.x = 0
        t.angular.z = 0
    else:
        t.linear.x = LINEAR_SPEED
        curr_point = np.array([long, lat])
        old_point = np.array([old_long, old_lat])
        distance_curr = np.abs(np.linalg.norm(np.cross(line_end - line_start, line_start - curr_point)))/np.linalg.norm(line_end - line_start)
        distance_old = np.abs(np.linalg.norm(np.cross(line_end - line_start, line_start - old_point)))/np.linalg.norm(line_end - line_start)
        side = np.cross(np.subtract(line_end, line_start), np.subtract(curr_point, line_start))
        side /= np.abs(side)
        # angle = np.arcsin((distance_old - distance_curr) / np.linalg.norm(np.subtract(old_point, curr_point)))
        # angle /= np.abs(angle)
        if np.linalg.norm(np.subtract(curr_point, line_start)) > np.linalg.norm(np.subtract(old_point, line_start)):
            direction = -1
        else:
            direction = 1
        p_comp = 1 * distance_curr * side
        d_comp = -10 * side * (distance_old - distance_curr)
        
        t.angular.z = ANGULAR_SPEED * (p_comp + d_comp) * direction
        # print("ANGLE: ", angle, "CLOSER?: ", distance_curr < distance_old)
    vel_pub.publish(t)
    rate.sleep()