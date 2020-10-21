#!/usr/bin/env python  
#August Soderberg
#asodeberg@brandeis.edu
#Autonomous Robotics
#Double Follow with tf2
#10/11/2020

import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg

#Callback 
def handle_turtle_pose(msg, turtlename):
    #Publishes transforms
    br = tf2_ros.TransformBroadcaster()
    #Transform message
    t = geometry_msgs.msg.TransformStamped()

    #Sets all of the fields in the transform correctly
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    #publishes or broadcasts the transform
    br.sendTransform(t)

#Sets up the subscriber with all of the correct data.
if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()