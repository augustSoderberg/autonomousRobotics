#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image

class CVImgSubPub:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('masked_img', Image, queue_size=1)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([50, 50, 170])
        upper_yellow = numpy.array([255, 255, 190])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        cv2.bitwise_and(image, image, image, mask=mask)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image))
        print('published')

rospy.init_node('cv_img_sub_pub')
cvImgSubPub = CVImgSubPub()
rospy.spin()
        