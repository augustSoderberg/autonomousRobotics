# COSI 119A
# PA8
# August Soderberg
# Joe Pickens
# November 1, 2020
# ----------------
# Description:
# Constants used in our nodes.

TIME_STEP = 0.1

#Max speeds of robot
LINEAR_SPEED = 0.6
ANGULAR_SPEED = 0.4

#Used for PID controller
MEMORY_LENGTH = 10
P_TUNER = 0.003
D_TUNER = 0.01
I_TUNER = 0.003

#Used for LIDAR parsing
ANGLE_WIDTH_FROM_CENTER = 30
THRESHOLD_DISTANCE = 3

#Numbers used in image processing.
TOP_HORIZONTAL = 540
BOTTOM_HORIZONTAL = 740
DISTANCE_VERT_TO_EDGE = 300
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
BLACK = 0
WHITE = 255