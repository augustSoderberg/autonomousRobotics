#!/usr/bin/env python

#August Soderberg
#asoderberg@brandeis.edu   
#PA: Teach a robot to dance!
#Autonomous Robotics
#9/29/2020

#Imports
import rospy
import sys
import math
import tf
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#Speed constants
LINEAR_SPEED = 0.3
ANGULAR_SPEED = 3.1415926/8

#Time for tricks constants
FULL_SPIRAL_TIME = 30
ZIGZAG_TIME = 16
POINTY_STAR_TIME = 16
JOSTLE_TIME = 2

#A kind of encoding of the base velocity vectors for each command
velocity_vectors = {
    "h" : [0,0],
    "l" : [0,1],
    "r" : [0,-1],
    "f" : [1,0],
    "b" : [-1,0],
    "s" : [1,1],
    "z" : [1,1],
    #Pointy star
    "p" : [1,1],
    #Jostle (back and forth)
    "j" : [1,0]
}

#Tells us whether there is a wall infront
def front_cb(msg):
   global wall_in_front
   wall_in_front = msg.data

#Tells us whether there is a wall nearby
def near_cb(msg):
    global wall_nearby
    wall_nearby = msg.data
   
# Sets the state and tracks how long since you pressed a key.
def key_cb(msg):
   global state; global last_key_press_time
   state = msg.data.lower()
   last_key_press_time = rospy.Time.now().to_sec()


# print the state of the robot
def print_state():
   print("-------------")
   #If there is a wall nearby, tell the user what to do.
   if wall_nearby:
       if wall_in_front:
           print("Rotate until you are not facing a wall")
       else:
           print("Please drive forward until you are not near the wall")

   print("LAST KEY PRESSED: ", state)
   print("SECS SINCE LAST KEY PRESS: ", math.floor(rospy.Time.now().to_sec() - last_key_press_time))
   print("Linear Speed: ", t.linear.x)
   print("Angular Speed: ", t.angular.z)

#Computes inner product of 2 lists.
def multiply(list1, list2):
    list = []
    if len(list1) != len(list2):
        print("Error: Lists are different lengths")
    else:
        for i in range(0, len(list1)):
            list.append(list1[i] * list2[i])
    return list

#Sets velocity given a list representing vecter
def set_velocity(twist, vector):
    twist.linear.x = LINEAR_SPEED*vector[0]
    twist.angular.z = ANGULAR_SPEED*vector[1]

#Sets the changing variable in the spiral move
def spiral_transform_handler():
    if rospy.Time.now().to_sec() - last_key_press_time > FULL_SPIRAL_TIME:
        return [1, 0]
    return [((rospy.Time.now().to_sec() - last_key_press_time) / FULL_SPIRAL_TIME), 1 - ((rospy.Time.now().to_sec() - last_key_press_time) / FULL_SPIRAL_TIME)]

#Sets the changing variable in the zigzag move
def zigzag_transform_handler():
    if ((rospy.Time.now().to_sec() - last_key_press_time) % ZIGZAG_TIME) < (ZIGZAG_TIME / 4):
        return [.4, 0]
    elif ((rospy.Time.now().to_sec() - last_key_press_time) % ZIGZAG_TIME) < (ZIGZAG_TIME / 2):
        return [0, .5]
    elif ((rospy.Time.now().to_sec() - last_key_press_time) % ZIGZAG_TIME) < (3*ZIGZAG_TIME / 4):
        return [-.4, 0]
    else:
        return [0, -.5]

#Sets the changing variable in the star move
def star_transform_handler():
    if (rospy.Time.now().to_sec() - last_key_press_time) % (POINTY_STAR_TIME / 2) < (POINTY_STAR_TIME / 4):
        return [.4, 1]
    else:
        return [-.4, 1]

#Sets the changing variable in the jostle move
def jostle_transform_handler():
    if (rospy.Time.now().to_sec() - last_key_press_time) % (JOSTLE_TIME) < (JOSTLE_TIME / 2):
        return [1, 0]
    else:
        return [-1, 0]
    

# init node
rospy.init_node('dancer')

# subscribers/publishers
front_sub = rospy.Subscriber('wall_in_front', Bool, front_cb)
near_sub = rospy.Subscriber('wall_nearby', Bool, near_cb)

# RUN rosrun prrexamples key_publisher.py to get /keys
key_sub = rospy.Subscriber('keys', String, key_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# start in state halted, not near anything, and grab the current time
state = "h"
last_key_press_time = rospy.Time.now().to_sec()
wall_in_front = False
wall_nearby = False

# set rate
rate = rospy.Rate(10)

# publish cmd_vel from here 
t = Twist()

# Wait for published topics, exit on ^c
while not rospy.is_shutdown():
    # print out the current state and time since last key press
    print_state()
    
    #Set the appropriate velocity for the robot
    if (wall_in_front and state != "l" and state != "r") or (wall_nearby and state != "f" and state != "l" and state != "r"):
        state = "h"
    if state == "h" or state == "l" or state == "r" or state == "f" or state == "b":
        set_velocity(t, velocity_vectors.get(state))
    elif state == "s":
        set_velocity(t, multiply(velocity_vectors.get(state), spiral_transform_handler()))
    elif state == "z":
        set_velocity(t, multiply(velocity_vectors.get(state), zigzag_transform_handler()))
    elif state == "p":
        set_velocity(t, multiply(velocity_vectors.get(state), star_transform_handler()))
    elif state == "j":
        set_velocity(t, multiply(velocity_vectors.get(state), jostle_transform_handler()))
    else:
        print("UNKNOWN COMMAND --- HALTING")
        state = "h"
        set_velocity(t, velocity_vectors.get(state))

    cmd_vel_pub.publish(t)
    
    # run at 10hz
    rate.sleep()