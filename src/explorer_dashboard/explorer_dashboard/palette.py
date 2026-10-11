"""The colours of the dashboard, in one place (0xRRGGBB).

The 3D scene and the legend of the HUD must agree on what yellow means, so both take
their colours from here.
"""

BACKGROUND = 0x070a10
WALL = 0x9be4ff
ROBOT = 0x5dff8f       # the robot's ring, its heading wedge and its trail
PLAN = 0xffd23f
LIDAR = 0xff4d6d
FRONTIER = 0x4aa8ff
TARGET = 0xff5ad1
CONE = 0xff7a1a

# What the legend of the HUD lists, top to bottom.
LEGEND = (
    (ROBOT, 'robot and its trail'),
    (WALL, 'wall (occupied cell)'),
    (LIDAR, 'lidar hit'),
    (PLAN, 'planned path'),
    (FRONTIER, 'frontier'),
    (TARGET, 'chosen target'),
    (CONE, 'cone seen by the camera'),
)
