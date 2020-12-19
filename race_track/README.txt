August Soderberg
Joe Pickens
Autonomous Robotics
Race Track PA
11/1/2020

HOW TO RUN:
Use the command $roslaunch race_track start_racer.launch

Implementation:
Our implementation for this PA was to utilize both the LIDAR and camera sensor data. For the
camera, we took in the raw image, processed it to be an entirely black image with white only where
the colors red or white used to be since these were the colors of the edges of the track. After
masking the image like this, we only considered two small boxes on the screen where the edges of
the track usually were. In these boxes we passed the difference between each box's average of its
left and right side white pixels into the PID controller and tuned it to have smooth drive when
there are no obstacles. If an obstacle ever gets within the minimum distance in the front cone of
the robot, we switch from camera + PID control to LIDAR control which turns the robot away from the
closest object in the front cone area we were examining. The driver node takes in both the Twist 
from the PID controller and the LIDAR parsing node and will decide to publish the PID Twist only if
line parsing data is good and the LIDAR isn't sensing any close objects in the front cone. 
Otherwise it will publish the LIDAR Twist to cmd_vel.

