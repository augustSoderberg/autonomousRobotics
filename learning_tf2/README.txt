August Soderberg
asoderberg@brandeis.edu
Autonomous Robotics
Double Follow with tf2
10/11/2020

HOW TO RUN: use the command $roslaunch learning_tf2 start_demo.launch

WHAT YOU WILL SEE: There will be turtle 1 which is controlled using the arrow keys. Turtle2 will
be following turtle1 around. Turtle3 will be following turtle2 around. Turtle4 will be following
turtle3 around. Turtle5 will be trying to avoid turtle1 as best as possible which unfortunately 
means turtle5 almost always ends up in the corner stuck.

This package contains many nodes used during the tutorial for tf2 from the ROS docs, however these
nodes are no longer necessary and are not used in my implementation. I only use the
turtle_tf2_broadcaster.py and turtle_tf2_listener_new.py nodes in my implementation. While the 
other nodes could be deleted, I didn't bother since I might want to use them in the future.
Unfortunately the turtle_tf2_listener_new node is very hard coded and would require several lines
to be changed just to add another turtle. This would be the first improvement I would make.