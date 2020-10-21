#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from constants import *
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float64

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
        center_pub.publish((top + bottom) // 2)
        slope_pub.publish((IMAGE_HEIGHT - TOP_HORIZONTAL) / (top - bottom))
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image))
        print((top + bottom) // 2, (IMAGE_HEIGHT - TOP_HORIZONTAL) / (top - bottom))

    
        

rospy.init_node('cv_img_sub_pub')
cvImgSubPub = CVImgSubPub()
center_pub = rospy.Publisher('center', Int32, queue_size=1)
slope_pub = rospy.Publisher('slope', Float64, queue_size=1)
rospy.spin()
        