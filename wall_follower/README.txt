August Soderberg
asoderberg@brandeis.edu
Wall Follower
Autonomous Robotics
10/4/2020

HOW TO RUN: please use $roslaunch wall_follower wall_follower.launch

This will launch all three of the nodes correctly if they are marked as executable.

This is a wall following robot, it will drive with a wall on the right hand side at the distance 
set in the constants file.

The robot has three states, WANDERING where it will drive forward until it is the set distance
from a wall. FOLLOWING where it will try its best to keep the wall exactly on the right hand side
and the set distance away. And LINING_UP where the robot will spin around in the shortest direction
to make the closest wall be exactly on the right hand side.

I wrote and tuned a custom PID controller for the robot which is only used in the following mode.
If during the use of the PID controller for publishing velocities the robot finds itself with the
closest wall more the value defined in the constants from a perfect right hand following, it will
spin efficiently to line back up again with the wall. The robot is great at smoothly rounding
corners entirely based on the tuning of the PID. The robot can complete all tasks described in the
assignment as demonstrated in the video.

The robot functions greedily and will follow which ever wall is closest at any given time, always
on the right side.

BUGS:
If the robot finds itself with multiple walls equidistant from it, it can sometimes get trapped and
have a hard time choosing which one to follow and just turn back and forth. This is rare but the 
simulation makes it easy to set up cases in which this has happened. In the video you will see it
performs as expected and never has this bug.