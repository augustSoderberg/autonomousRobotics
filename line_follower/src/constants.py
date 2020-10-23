# August Soderberg
# asoderberg@brandeis.edu
# Autonomous Robotics
# PA - Line Follower
# 10/23/2020

IMAGE_HEIGHT = 1080
IMAGE_WIDTH = 1920

#Top row to be scanned during following
TOP_HORIZONTAL = .75 * IMAGE_HEIGHT
#Bottom row to be scanned during following
BOTTOM_HORIZONTAL = .8 * IMAGE_HEIGHT

#Values in the masked image
WHITE = 255
BLACK = 0

#Relaxation of the pixel distances required.
EPSILON = 150

ANGULAR_SPEED = 0.4
LINEAR_SPEED = 0.3

P_TUNER = 0.001
I_TUNER = .0003
D_TUNER = 0.2

#Number of values stored to be used in PID calculations
INTEGRAL_MEMORY_LENGTH = 10
