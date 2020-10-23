#!/usr/bin/env python
# August Soderberg
# asoderberg@brandeis.edu
# Autonomous Robotics
# PA - Line Follower
# 10/23/2020

import rospy, cv2, cv_bridge, numpy
from constants import *
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float64, Bool

def dist_between(ax, ay, bx, by):
    return (((bx - ax)**2) + ((by - ay)**2))**.5

#In the "horiz" row of pixels in the image, find the center of the white blob in that line
def find_middle_white(image, horiz):
    horiz = int(horiz)
    found_white = False
    first_white = -1
    last_white = -1
    for i in range(0, IMAGE_WIDTH):
        if image[horiz, i] == WHITE:
            if not found_white:
                first_white = i
                found_white = True
            else:
                last_white = i 
    return (last_white + first_white) // 2

#Find the coordinates of the white pixel closest to the target pixel
def find_closest_white(image):
    target = [TOP_HORIZONTAL, IMAGE_WIDTH // 2]
    closest_white_x = -1
    closest_white_y = -1
    #Effectively compress the image by only searching 1 out of every 36 pixels.
    for i in range(0, IMAGE_WIDTH, 6):
        for j in range(0, IMAGE_HEIGHT, 6):
            if (image[j, i] == WHITE and dist_between(i, j, target[1], target[0]) < 
                dist_between(closest_white_x, closest_white_y, target[1], target[0])):
                closest_white_y = j
                closest_white_x = i
    return (closest_white_x, closest_white_y)



class CVImgSubPub:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.line_found_pub = rospy.Publisher('line_found', Bool, queue_size=1)
        self.closest_white_x_pub = rospy.Publisher('closest_white_x', Int32, queue_size=1)
        self.closest_white_y_pub = rospy.Publisher('closest_white_y', Int32, queue_size=1)
        self.center_pub = rospy.Publisher('center', Int32, queue_size=1)
        self.slope_pub = rospy.Publisher('slope', Float64, queue_size=1)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #Values on the boundary of yellow
        lower_yellow = numpy.array([18, 0, 0])
        upper_yellow = numpy.array([50, 255, 255])
        image = cv2.inRange(image, lower_yellow, upper_yellow)
        #Find the center of the white pixels in the horizontal lines TOP/BOTTOM_HORIZONTAL
        top = find_middle_white(image, TOP_HORIZONTAL)
        bottom = find_middle_white(image, BOTTOM_HORIZONTAL)
        #If line doesn't go through the scanned area
        if (top == -1 or bottom == -1):
            self.line_found_pub.publish(False)
            closest_white = find_closest_white(image)
            self.closest_white_x_pub.publish(closest_white[0])
            self.closest_white_y_pub.publish(closest_white[1])
        else:
            self.line_found_pub.publish(True)
            self.center_pub.publish((top + bottom) // 2)
            self.slope_pub.publish((bottom - top) / (BOTTOM_HORIZONTAL - TOP_HORIZONTAL))

        
rospy.init_node('image_parcer')
cvImgSubPub = CVImgSubPub()
rospy.spin()
        