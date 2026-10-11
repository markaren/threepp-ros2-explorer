# What the explorer is doing right now. Published on `exploration_status` about twice a
# second, and sent as the feedback of the Explore action.

# Constants are part of the message definition: both C++ and Python get them generated,
# so nobody has to remember that 1 means "exploring".
uint8 IDLE = 0          # waiting for a goal
uint8 EXPLORING = 1     # driving towards a frontier
uint8 RETURNING = 2     # exploration finished, driving back to the start
uint8 NAVIGATING = 3    # driving to a goal_pose the operator clicked
uint8 DONE = 4          # last job finished, nothing left to do

uint8 state

uint32 frontier_count           # frontier clusters still open
float32 explored_area           # m^2 of floor known to be free
float32 distance                # m driven since the job started
float32 elapsed                 # s since the job started
geometry_msgs/Point target      # where the robot is heading (map frame)
