import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class cvImageParser:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("window", 960, 540)
        self.pub = rospy.Publisher('position', Float32, queue_size = 1)
        self.sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_cb)

    def image_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        cv2.imshow('window', image)
        cv2.waitKey(3)

rospy.init_node('image_parser') 
cvImageParser = cvImageParser()
rospy.spin()
