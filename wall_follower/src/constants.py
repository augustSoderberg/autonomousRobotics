#August Soderberg
#asoderberg@brandeis.edu
#Wall Follower
#Autonomous Robotics
#10/4/2020

#These are a bunch of constants used in the nodes

#Represents the state where the robot just drives to get close to a wall
WANDERING = 0
#Represents the state where the robot is actively following the wall
FOLLOWING = 1
#Represents the state where the robot is spinning to put the wall on the right side
LINING_UP = 2

#How far the robot stays from the wall
FOLLOWING_DIST = 1

#How much error there can be in the angle while lining up
ANGLE_ERROR = 5

#The angle difference between a perfectly following robot and an adjusting robot which we will
#allow the robot to keep driving and adjusting without stopping
#Must be between 0 and 90
ANGLE_DIFF_REQUIRING_LINING_UP = 60

#Linear speed of the robot
LINEAR_SPEED = 0.3
#Angular speed of the robot
ANGULAR_SPEED = 3.1415926/6

#Multiplies used to tune the PID controller
#Proportional constant
P_CONSTANT = 1
#Integral constant
I_CONSTANT = 1.5
#Derivative constant
D_CONSTANT = .7