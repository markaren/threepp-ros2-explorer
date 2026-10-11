"""The robot's brain: mapping, frontier detection, planning and path following.

The algorithm modules (occupancy_grid, frontiers, planner, pure_pursuit) use numpy only and
know nothing about ROS, so they can be read and tested on their own. The two node modules
(mapper_node, explorer_node) are the ROS glue around them.
"""
