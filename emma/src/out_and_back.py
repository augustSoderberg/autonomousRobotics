#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('test')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size =  1)
T = Twist()
rate = rospy.Rate(10)
while rospy.Time.now().to_sec() <= 0:
    rate.sleep()

iTime = rospy.Time.now().to_sec()
while not rospy.is_shutdown():
    while rospy.Time.now().to_sec() < iTime + 3:
        print(iTime, rospy.Time.now().to_sec(), 'one')
        T.linear.x = 0.3
        pub.publish(T)

    T.linear.x = 0
    while rospy.Time.now().to_sec() < iTime + 9:
        print(iTime, rospy.Time.now().to_sec(), 'two')
        T.angular.z = 3.14/6
        pub.publish(T)
    
    T.angular.z = 0
    while rospy.Time.now().to_sec() < iTime + 12:
        print(iTime, rospy.Time.now().to_sec(), 'three')
        T.linear.x = 0.3
        pub.publish(T)
    
    T.linear.x = 0
    pub.publish(T)

