August Soderberg

My code follows the PA very closely except for a few things:
On the spiral I decided to steadily decrase the rotation rate while steadily increasing linear rate
to create a nice even spiral.
When the robot gets near a wall at all it is only going to let you rotate it around until the front
is facing somewhere open, at which case you will be able to drive forward or spin, all other
options will continue to have the robot sit.
In this entire PA, "halt" really meant just sit stationary and wait.
I also added 2 more moves.
The first one is basically a four pointed star where each vertex is made of two symmetrical curves.
The second one is the jostle feature which just drives the robot back and forth quickly (When
the wheels aren't slipping all around).

Overall I thought this was a really easy assignement and I just wish our robot wasn't so bad at 
moving, and I think it would have been fun to see on a real robot. I wish I could just write a 
launch file which starts all nodes at once but I haven't been able to figure it out.

:::IMPORTANT NOTE:::
The robot in the simulation does this thing where it spins when it starts and stops, from the
velocity outputs it is easy to see that this is not because of anything I have done, it is
just an artifact of the simulation, please excuse it.
