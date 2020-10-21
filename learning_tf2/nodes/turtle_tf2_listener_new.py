#!/usr/bin/env python 
#August Soderberg
#asodeberg@brandeis.edu
#Autonomous Robotics
#Double Follow with tf2
#10/11/2020

#imports
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

#Creates turtles and publishes cmd_vel
if __name__ == '__main__':
    #init node
    rospy.init_node('tf2_turtle_listener')

    #Set up tf
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #Spawn is a service provided in our ROS package and we set up a proxy for it
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    
    #Turtle0 and turtle1 are placeholders to make the indicies clearer
    turtle_names = ['turtle0', 'turtle1', 'turtle2', 'turtle3', 'turtle4', 'turtle5']

    #Spawn in turtle2 - 5
    for i in range(2, len(turtle_names)):
        spawner(4, 2, 0, turtle_names[i])
    
    #Create publishers
    turtle_vel2 = rospy.Publisher('%s/cmd_vel' % turtle_names[2], geometry_msgs.msg.Twist, queue_size=1)
    turtle_vel3 = rospy.Publisher('%s/cmd_vel' % turtle_names[3], geometry_msgs.msg.Twist, queue_size=1)
    turtle_vel4 = rospy.Publisher('%s/cmd_vel' % turtle_names[4], geometry_msgs.msg.Twist, queue_size=1)
    turtle_vel5 = rospy.Publisher('%s/cmd_vel' % turtle_names[5], geometry_msgs.msg.Twist, queue_size=1)

    #rate object
    rate = rospy.Rate(10.0)
    #Continually:
    while not rospy.is_shutdown():
        try:
            #Store the appropriate transform between the reactionary turtle and the commanding
            #turtle, i.e. transform between 2 and 1, 3 and 2, 4 and 3, 5 and 1.
            trans2 = tfBuffer.lookup_transform(turtle_names[2], 'turtle1', rospy.Time())
            trans3 = tfBuffer.lookup_transform(turtle_names[3], 'turtle2', rospy.Time())
            trans4 = tfBuffer.lookup_transform(turtle_names[4], 'turtle3', rospy.Time())
            trans5 = tfBuffer.lookup_transform(turtle_names[5], 'turtle1', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        #Create twist message for each turtle
        msg2 = geometry_msgs.msg.Twist()
        msg3 = geometry_msgs.msg.Twist()
        msg4 = geometry_msgs.msg.Twist()
        msg5 = geometry_msgs.msg.Twist()

        #Assign the angular and linear based on geometry to make sure the turtle is following or avoiding
        #the turtle it desires to follow or avoid
        msg2.angular.z = 4 * math.atan2(trans2.transform.translation.y, trans2.transform.translation.x)
        msg2.linear.x = 0.5 * math.sqrt(trans2.transform.translation.x ** 2 + trans2.transform.translation.y ** 2)

        msg3.angular.z = 4 * math.atan2(trans3.transform.translation.y, trans3.transform.translation.x)
        msg3.linear.x = 0.5 * math.sqrt(trans3.transform.translation.x ** 2 + trans3.transform.translation.y ** 2)
        
        msg4.angular.z = 4 * math.atan2(trans4.transform.translation.y, trans4.transform.translation.x)
        msg4.linear.x = 0.5 * math.sqrt(trans4.transform.translation.x ** 2 + trans4.transform.translation.y ** 2)

        msg5.angular.z = 4 * math.atan2(trans5.transform.translation.y, trans5.transform.translation.x)
        msg5.linear.x = -0.5 * 1 / math.sqrt(trans5.transform.translation.x ** 2 + trans5.transform.translation.y ** 2)

        #publish all twists.
        turtle_vel2.publish(msg2)
        turtle_vel3.publish(msg3)
        turtle_vel4.publish(msg4)
        turtle_vel5.publish(msg5)

        rate.sleep()