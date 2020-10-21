#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from constants import *
from sensor_msgs.msg import Image

class CVImgSubPub:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('masked_img', Image, queue_size=1)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([18, 0, 0])
        upper_yellow = numpy.array([50, 255, 255])
        image = cv2.inRange(image, lower_yellow, upper_yellow)
        top = find_middle_white(image, TOP_HORIZONTAL)
        bottom = find_middle_white(image, IMAGE_HEIGHT - 1)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image))
        print(top + "   " + bottom)
    
    def find_middle_white(self, image, horiz):
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

    
        

rospy.init_node('cv_img_sub_pub')
cvImgSubPub = CVImgSubPub()
rospy.spin()
        