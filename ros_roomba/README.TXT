August Soderberg
asoderberg@brandeis.edu
PA: ROS Roomba
Autonomous Robotics
9/21/2020

My baseline approach for this project was to select a secter in front of the robot in which we
would scan values from the LIDAR. As soon as the robot was sufficiently close to an object
within that secter we would stop the robot and see whether the boundary on the right or the 
boundary on the left was further, which ever was further would almost always mean our robot
could spin in that direction to become clear in the forward direction fastest. Once the robot
determined it was clear ahead we would instruct the robot to spin randomly anywhere from 0 to 90 
degrees in the same direction it was already spinning in order to force the robot to have the 
highest probablility of driving over all areas of the floor while minimizing wasted spinning time.

You may notice that I didn't use the ranges_max or inf values anywhere in my code.
This is because I don't care how far the robot could drive forward as long as it can drive forward 
which means the inf value is useless as well.

I chose 45 Degrees as the secter angle we search within somewhat randomly, it is plenty to avoid all
reasonable collisions. 

Bugs:
While I was unable to witness my robot ever contact a wall or get stuck there are a couple ways it
could happen. Objects so thin that the lasers miss the object, or setting MIN_DIST below 0.25. 
Additionally, with my efficient approach of spinning until free and then spinning random between 0 
and 90 degrees, I suspect there is a magical configuration which would create some kind of 
infinite loop where the only way out would be spinning around until free and then spinning more 
than 90 degrees. I have been unable to find such a set up in theory and I will confidently claim 
that the level of difficulty it would require means that the incredible time saving of my 
intelligent algorithm is well worth this nearly impossible (maybe truly impossible) issue.