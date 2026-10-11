"""Mission control for the exploring robot.

The package has two halves that do not know each other:

* the ROS half talks to the other nodes:
      dashboard_node.py   the node, its callbacks, the operator's commands, main()
* the drawing half talks to threepp:
      window.py           canvas, renderer, camera, mouse and keys
      scene.py            the 3D picture: map, walls, robot, plan, frontiers, cones
      hud.py              the 2D text on top
      palette.py          the colours
      cursor.py           where the mouse cursor is

They meet in app.py (the render loop), and all they share is:

      world_model.py      plain data types, in ROS coordinates
      mailbox.py          the thread-safe hand-over between the two threads
      convert.py          ROS axes (z up) to threepp axes (y up), and other maths

A good reading order: mailbox.py, world_model.py, dashboard_node.py, app.py, and then
the drawing half.
"""
