August Soderberg
asoderberg@brandeis.edu
Autonomous Robotics
PA - Line Follower
10/23/2020

HOW TO RUN::
$ roslaunch line_follower start_follower.launch
This command will start all required nodes for the robot to begin following the line

METHOD:
If the camera is unable to see a yellow line in the narrow range used when following, the robot will
execute the line_finding algorithm until the line appears in the useful range. The line_finding
algorithm first will spin around until it seems some kind of yellow line in frame. Once it sees
some kind of yellow, it will turn until the yellow area closest to the robot is directly in front.
It will then drive forward until the yellow area is in the correct range for following, the turn
counterclockwise until the line is in a position in which the robot can follow it.
While following, the robot uses a PID control reading infromation from a relatively short
horizontal band of pixels in the bottom third of the screen. The robot uses the information from
this section of line to smoothly follow the line around turns.

KNOWN BUGS:
There are no known bugs although the PID does follow slightly off center, it was close enough for
me to call good and thus I left it alone.