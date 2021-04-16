#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from constants import *

# COSI 119A
# PA8
# August Soderberg
# November 1, 2020
# ----------------
# Description:
# The image parsing node which takes in the image, masks it, scans for relevent pixels, and
# publishes the data to the PID controller.

# In a vertical line, finds the middle pixel in the first white block.
def find_middle_white(image, vert):
    first_white = -1
    for i in range(TOP_HORIZONTAL, BOTTOM_HORIZONTAL):
        # Checks current and next pixel to eliminate noise.
        if image[i, vert] == WHITE and first_white == -1:
            if image[i+1, vert] == WHITE:
                first_white = i
        if image[i, vert] == BLACK and first_white != -1:
            if image[i+1, vert] == BLACK:
                return (i - 1 + first_white) // 2
    return -1

class CVImgSubPub:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.line_found_pub = rospy.Publisher('line_found', Bool, queue_size=1)
        self.p_component_pub = rospy.Publisher('p_comp', Int32, queue_size=1)
        
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        #First zone of red
        first_red_lower_bound = numpy.array([0, 165, 191])
        first_red_upper_bound = numpy.array([12, 255, 255])
        #Second zone of red
        second_red_lower_bound = numpy.array([167, 165, 191])
        second_red_upper_bound = numpy.array([179, 255, 255])
        #White zone
        white_lower_bound = numpy.array([0, 0, 191])
        white_upper_bound = numpy.array([179, 50, 255])
        #Masks red and white
        mask1 = cv2.inRange(image, first_red_lower_bound, first_red_upper_bound)
        mask2 = cv2.inRange(image, second_red_lower_bound, second_red_upper_bound)
        mask3 = cv2.inRange(image, white_lower_bound, white_upper_bound)
        #Combines masks
        image = cv2.bitwise_or(mask1, mask2)
        image = cv2.bitwise_or(image, mask3)
        #This handles the checking for proportional error in the designated boxes.
        verts = numpy.array([0, DISTANCE_VERT_TO_EDGE, IMAGE_WIDTH - DISTANCE_VERT_TO_EDGE, IMAGE_WIDTH - 1])
        for i in range(len(verts)):
            verts[i] = find_middle_white(image, verts[i])
        self.p_component_pub.publish(((verts[3] + verts[2]) // 2) - ((verts[0] + verts[1]) // 2))
        #Only true if values are all good.
        if verts[0] > verts[1] and verts[3] > verts[2] and min((verts[0], verts[1], verts[2], verts[3])) > -1:
            self.line_found_pub.publish(True)
        else:
            self.line_found_pub.publish(False)

        
rospy.init_node('image_parser')
cvImgSubPub = CVImgSubPub()
rospy.spin()