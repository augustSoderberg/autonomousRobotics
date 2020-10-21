#August Soderberg
#asoderberg@brandeis.edu   
#PA: ROS Roomba
#Autonomous Robotics
#9/21/2019

#This is just a bunch of constants used by both classes.

#Represents the driving state
DRIVING = 0
#Represents the spinning state
SPINNING = 1
#Represents the waiting state
WAITING = 2

#Linear speed of robot
LINEAR_SPEED = 0.3
#Angular speed of robot
ANGULAR_SPEED = 0.42

#Closest we want to get to the wall, since the robot usually fails to stop instantly this is a very
#good value to ensure it never turns into the wall. 0.3 is around the absolute minimum
#Do not set lower than 0.25, the robot will run into walls because of the outer dimension
MIN_DIST = 0.35

#Total measure of the secter in front of us which we scan for objects.
FRONT_SECTOR_DEGREES = 45

#Representation of whether the robot should turn right.
TURN_RIGHT = True

#Constants which when multiplied by angular speed turn the robot in the correct direction
RIGHT = -1
LEFT = 1

#PI (I know I could just import but this seems easier)
PI = 3.1415926