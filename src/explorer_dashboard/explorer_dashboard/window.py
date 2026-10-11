"""The window: canvas, renderer, camera, mouse and keys. threepp only, no ROS.

Render thread only (see mailbox.py). The window, and the OpenGL context that comes
with it, belong to the thread that creates a DashboardWindow, and that thread must
make every later call too.
"""

import math
import time
from typing import Callable, List, Optional, Tuple

import threepp as tp

from explorer_dashboard import convert
from explorer_dashboard.cursor import cursor_in_window
from explorer_dashboard.hud import Hud
from explorer_dashboard.scene import WorldView

TITLE = 'Explorer dashboard: what the robot believes'
KEYS = ('E', 'C', 'R', 'G', 'F')
FRAME_TIME = 1.0 / 60.0

# The default view. The warehouse of the demo spans x in [-2, 24] and y in [-8, 8].
DEFAULT_CENTRE = (11.0, 0.0)
DEFAULT_WIDTH = 31.0
FIELD_OF_VIEW = 40.0                    # degrees, vertical
CAMERA_ELEVATION = math.radians(60.0)   # above the floor, looking down


class DashboardWindow:
    """Owns the window and draws one frame at a time: the 3D world, then the HUD."""

    def __init__(self, width: int, height: int, headless: bool) -> None:
        # headless=True gives a hidden surface with a real OpenGL context: nothing
        # shows on screen, but rendering and screenshots work. Tests and launch files
        # on a machine without an operator use it.
        self._canvas = tp.Canvas(TITLE, width=width, height=height, antialiasing=4,
                                 headless=headless)
        # GLRenderer is threepp's OpenGL renderer. It runs on any laptop, which is why
        # the dashboard uses it and not the Vulkan one.
        self._renderer = tp.GLRenderer(self._canvas)
        self.world = WorldView()
        self.hud = Hud(*self._canvas.size())

        self._camera = tp.PerspectiveCamera(FIELD_OF_VIEW, self._canvas.aspect(),
                                            0.1, 400.0)
        # OrbitControls turns mouse input into camera motion: left drag orbits, the
        # wheel zooms, right drag pans. It always orbits around its `target`.
        self._controls = tp.OrbitControls(self._camera, self._canvas)
        self._controls.enable_damping = True
        self._controls.min_distance = 2.0
        self._controls.max_distance = 120.0
        self.look_at_area(DEFAULT_CENTRE[0], DEFAULT_CENTRE[1], DEFAULT_WIDTH)

        self.follow = False
        self._key_was_down = {key: False for key in KEYS}
        self._canvas.on_window_resize(self._on_resize)

    def look_at_area(self, centre_x: float, centre_y: float, width: float) -> None:
        """Place the camera so that `width` metres around a ROS point fill the view."""
        half_view = math.radians(FIELD_OF_VIEW) / 2.0
        distance = (width / 2.0) / (math.tan(half_view) * self._canvas.aspect())
        self._controls.target = tp.Vector3(centre_x, 0.0, -centre_y)
        self._camera.position.set(centre_x,
                                  distance * math.sin(CAMERA_ELEVATION),
                                  -centre_y + distance * math.cos(CAMERA_ELEVATION))
        self._controls.update()
        self._placed_at = (self._camera.position.x, self._camera.position.y, self._camera.position.z)

    def camera_untouched(self) -> bool:
        """Is the camera still where look_at_area() put it? False once the operator
        (or the follow mode) has moved it."""
        position = self._camera.position
        return all(abs(now - then) < 1e-3 for now, then in
                   zip((position.x, position.y, position.z), self._placed_at))

    def _on_resize(self, width: int, height: int) -> None:
        self._camera.aspect = width / max(height, 1)
        self._camera.update_projection_matrix()
        self._renderer.set_size(width, height)
        self.hud.resize(width, height)

    def pressed_keys(self) -> List[str]:
        """The keys that went DOWN since the previous frame.

        threepp lets us poll whether a key is held right now. A held key reads "down"
        for many frames, but an action like "send the goal" must happen once per
        press. So we remember last frame's state and report only the change from up
        to down. This is called edge detection.
        """
        pressed = []
        for key in KEYS:
            is_down = self._canvas.is_key_down(key)
            if is_down and not self._key_was_down[key]:
                pressed.append(key)
            self._key_was_down[key] = is_down
        return pressed

    def floor_point_under_cursor(self) -> Optional[Tuple[float, float]]:
        """The ROS (x, y) on the floor that the mouse cursor points at, or None.

        A pixel on the screen is a whole ray in the world. unproject() turns a point
        in normalised device coordinates (x, y in [-1, 1], z = -1 on the near plane
        and +1 on the far plane) back into the world, so two calls give two points
        on the ray. Then we solve for where the ray crosses the floor, y = 0.
        """
        cursor = cursor_in_window(TITLE)
        if cursor is None:
            # No cursor position on this system: use the point the camera orbits.
            target = self._controls.target
            return convert.three_to_ros_xy(target.x, target.z)
        x, y = 2.0 * cursor[0] - 1.0, 1.0 - 2.0 * cursor[1]
        near = tp.Vector3(x, y, -1.0).unproject(self._camera)
        far = tp.Vector3(x, y, 1.0).unproject(self._camera)
        descent = near.y - far.y
        if descent <= 1e-9:
            return None   # the ray goes up or level: the cursor is on the sky
        along = near.y / descent
        return convert.three_to_ros_xy(near.x + along * (far.x - near.x),
                                       near.z + along * (far.z - near.z))

    def _follow_robot(self) -> None:
        """Move the orbit centre towards the robot, and the camera along with it.

        Shifting the camera and its target by the same vector keeps the angle and the
        zoom the operator has chosen. Moving only a tenth of the way per frame turns
        the robot's jerky 30 Hz pose into a smooth glide.
        """
        pose = self.world.robot_pose
        if not self.follow or pose is None:
            return
        target = self._controls.target
        step_x = 0.1 * (pose.x - target.x)
        step_z = 0.1 * (-pose.y - target.z)
        self._controls.target = tp.Vector3(target.x + step_x, 0.0, target.z + step_z)
        self._camera.position.x += step_x
        self._camera.position.z += step_z

    def draw(self) -> None:
        """Render one frame: the 3D world, then the HUD on top of it."""
        self.world.animate()
        self._follow_robot()
        self._controls.update()
        self._renderer.auto_clear = True
        self._renderer.render(self.world.scene, self._camera)
        self._renderer.auto_clear = False   # keep the 3D picture, draw over it
        self._renderer.render(self.hud.scene, self.hud.camera)

    def next_frame(self, frame: Callable[[], None]) -> bool:
        """Run `frame` once, show the result, and return False when the window closed.

        With vertical sync the graphics driver already waits for the monitor. A hidden
        (headless) window has no monitor to wait for and would spin a CPU core at
        thousands of frames per second, so we cap the rate ourselves.
        """
        started = time.perf_counter()
        still_open = self._canvas.animate_once(frame)
        spare = FRAME_TIME - (time.perf_counter() - started)
        if spare > 0.0:
            time.sleep(spare)
        return still_open

    def save_frame(self, path: str) -> None:
        """Write the frame that was drawn last to a PNG file."""
        self._renderer.save_frame(path)

    def close(self) -> None:
        self._canvas.close()
