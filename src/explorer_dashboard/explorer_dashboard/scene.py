"""The 3D picture of what the robot believes. threepp only: no ROS in this file.

Every method takes plain data in ROS coordinates (see world_model.py) and converts to
threepp's y-up axes at the last moment, through convert.py.

IMPORTANT: every function here calls OpenGL underneath, so all of it must run on the
render thread (see mailbox.py). Nothing in this file is thread-safe, and nothing needs
to be.
"""

import math
import time
from collections import deque
from typing import Callable, Deque, List, Optional, Tuple

import numpy as np
import threepp as tp

from explorer_dashboard import convert, palette
from explorer_dashboard.world_model import Frontiers, Grid, Pose2D, Scan

WALL_HEIGHT = 0.6          # m. The real walls are taller: low walls keep the floor visible
GROW_SECONDS = 0.8         # how long a newly discovered wall takes to rise

TRAIL_SPACING = 0.10       # m between two stored trail points
TRAIL_POINTS = 3000        # 300 m of trail, then the oldest part is forgotten
TRAIL_FADE_POINTS = 500    # the trail fades out over its newest 50 m
TELEPORT_DISTANCE = 1.0    # m. A jump this long in one frame is a reset, not driving


def _basic(color: int, opacity: float = 1.0) -> tp.MeshBasicMaterial:
    """An unlit material: the colour on screen is exactly the colour given.

    Most of the dashboard is information, not scenery, and information should not get
    darker because a lamp happens to be behind it.
    """
    material = tp.MeshBasicMaterial()
    material.color = color
    material.side = tp.Side.Double
    if opacity < 1.0:
        material.transparent = True
        material.opacity = opacity
        material.depth_write = False
    return material


class FloorMap:
    """The occupancy grid as a picture on the floor: one texel per grid cell."""

    def __init__(self, parent: tp.Object3D) -> None:
        self._material = _basic(0xffffff)
        # A 1 x 1 plane, laid flat. Its size and place are set from the grid's own
        # numbers in update(), so any grid size and resolution works.
        self._mesh = tp.Mesh(tp.PlaneGeometry(1.0, 1.0), self._material)
        self._mesh.rotate_x(-math.pi / 2.0)
        self._mesh.visible = False
        parent.add(self._mesh)
        self._texture: Optional[tp.Texture] = None
        self._shape: Optional[Tuple[int, ...]] = None

    def update(self, grid: Grid) -> None:
        # Row 0 of the grid is the row with the smallest y, and row 0 of a threepp
        # data texture is the bottom of the plane (v = 0). After the plane is laid
        # flat, its bottom edge is at +z, which is ROS -y. So the rows already match.
        image = convert.grid_to_rgb(grid.cells)
        if image.shape != self._shape:
            # First map, or the mapper changed the grid size: a new texture.
            self._texture = tp.data_texture(image, srgb=True)
            # Nearest-neighbour filtering keeps every cell a crisp square. The default
            # (linear) would blur the walls into the floor.
            self._texture.mag_filter = tp.Filter.Nearest
            self._texture.min_filter = tp.Filter.Nearest
            self._texture.generate_mipmaps = False
            self._texture.needs_update()
            self._material.map = self._texture
            self._material.needs_update()
            self._shape = image.shape
        else:
            # Same size as before: overwrite the pixels in place (no reallocation).
            self._texture.update_data(image)
        centre_x = grid.origin_x + grid.width_m / 2.0
        centre_y = grid.origin_y + grid.height_m / 2.0
        self._mesh.scale.set(grid.width_m, grid.height_m, 1.0)
        self._mesh.position.set(centre_x, 0.0, -centre_y)
        self._mesh.visible = True


class Walls:
    """The occupied cells, extruded: one box per cell, drawn with ONE draw call.

    An InstancedMesh holds a single box and a list of transforms. The graphics card
    draws the box once per transform. Thousands of separate Mesh objects would cost
    thousands of draw calls per frame, and it is the number of draw calls, not the
    number of triangles, that makes a laptop struggle.

    A wall that is new rises out of the floor over GROW_SECONDS, so the eye is drawn
    to what the robot has just discovered.
    """

    def __init__(self, parent: tp.Object3D) -> None:
        self._parent = parent
        self._material = tp.MeshStandardMaterial()
        self._material.color = palette.WALL
        self._material.roughness = 0.55
        self._material.emissive = 0x0d4f6e
        self._mesh: Optional[tp.InstancedMesh] = None
        self._capacity = 0
        self._resolution = 0.0
        # Per grid cell: the time it was first seen occupied, or infinity for "not a
        # wall". The layout tells us when the mapper has switched to another grid.
        self._layout: Optional[Tuple] = None
        self._first_seen = np.empty((0, 0))
        # Per instance (one per occupied cell), in the order of the instance list.
        self._centres = np.empty((0, 2), dtype=np.float32)
        self._born = np.empty(0)
        self._full_height = np.empty(0, dtype=bool)

    def update(self, grid: Grid) -> None:
        now = time.monotonic()
        layout = (grid.cells.shape, grid.resolution, grid.origin_x, grid.origin_y)
        if layout != self._layout:
            self._first_seen = np.full(grid.cells.shape, np.inf)
            self._layout = layout
        occupied = grid.cells >= convert.OCCUPIED_THRESHOLD
        was_occupied = np.isfinite(self._first_seen)
        if np.array_equal(occupied, was_occupied):
            return  # Most new maps only change free space: nothing to rebuild.
        self._first_seen[occupied & ~was_occupied] = now
        self._first_seen[~occupied] = np.inf   # the mapper changed its mind

        rows, cols = np.nonzero(occupied)
        if len(rows) > self._capacity or grid.resolution != self._resolution:
            self._allocate(max(2048, 2 * len(rows)), grid.resolution)
        self._centres = convert.cell_centres(grid, rows, cols)
        self._born = self._first_seen[rows, cols]
        self._full_height = np.zeros(len(rows), dtype=bool)
        self._mesh.set_count(len(rows))
        self._mesh.visible = len(rows) > 0
        self.animate()

    def animate(self) -> None:
        """Called every frame: move the walls that are still rising."""
        rising = np.nonzero(~self._full_height)[0]
        if len(rising) == 0:
            return
        progress = np.clip((time.monotonic() - self._born[rising]) / GROW_SECONDS,
                           0.0, 1.0)
        # Ease out: fast at first, then settling. 1 - (1 - t)^3 goes from 0 to 1.
        scale = np.maximum(1.0 - (1.0 - progress) ** 3, 0.02)
        matrix = tp.Matrix4()
        for index, height in zip(rising.tolist(), scale.tolist()):
            x, y = self._centres[index]
            # Scale the box in y, and lift it by half its new height so that its
            # bottom stays on the floor.
            matrix.make_scale(1.0, height, 1.0)
            matrix.set_position(float(x), height * WALL_HEIGHT / 2.0, -float(y))
            self._mesh.set_matrix_at(index, matrix)
        # The matrices live in a buffer on the graphics card. This flag tells threepp
        # to upload the changed copy before the next draw.
        self._mesh.instance_matrix_needs_update()
        self._full_height[rising] = progress >= 1.0

    def _allocate(self, capacity: int, resolution: float) -> None:
        """An InstancedMesh has a fixed capacity: when it is full, make a bigger one."""
        if self._mesh is not None:
            self._parent.remove(self._mesh)
        box = tp.BoxGeometry(resolution, WALL_HEIGHT, resolution)
        self._mesh = tp.InstancedMesh(box, self._material, capacity)
        # threepp culls an object when its bounding sphere is off screen, and for an
        # InstancedMesh that sphere is the one box at the origin. Switch culling off,
        # or the whole building vanishes when the origin leaves the picture.
        self._mesh.frustum_culled = False
        self._parent.add(self._mesh)
        self._capacity = capacity
        self._resolution = resolution


class Ribbon:
    """A polyline drawn as a flat strip of triangles.

    Why not tp.Line? OpenGL core profile draws lines exactly one pixel wide, and a
    one-pixel line is invisible on a classroom projector. A strip has a width in
    metres. The vertex buffer is allocated once, at full capacity, and only refilled:
    set_draw_range() says how much of it holds real data.
    """

    def __init__(self, parent: tp.Object3D, capacity: int, width: float, height: float,
                 color: int, glow: bool) -> None:
        self._capacity = capacity
        self._half_width = width / 2.0
        self._height = height
        self._positions = np.zeros((6 * (capacity - 1), 3), dtype=np.float32)
        self._colors = np.ones((6 * (capacity - 1), 3), dtype=np.float32)
        self._geometry = tp.BufferGeometry()
        self._geometry.set_attribute('position', self._positions)
        self._geometry.set_attribute('color', self._colors)
        self._geometry.set_draw_range(0, 0)
        material = _basic(color)
        material.vertex_colors = True   # final colour = material colour * vertex colour
        if glow:
            # Additive blending adds the strip's colour to what is behind it, so a
            # vertex colour of black is fully transparent. That is how the trail fades.
            material.transparent = True
            material.blending = tp.Blending.Additive
            material.depth_write = False
        self._mesh = tp.Mesh(self._geometry, material)
        self._mesh.frustum_culled = False
        parent.add(self._mesh)

    def set_points(self, points_xy: np.ndarray,
                   brightness: Optional[np.ndarray] = None) -> None:
        """Lay the strip along (N, 2) ROS points. `brightness` is one 0..1 per point."""
        points_xy = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
        if len(points_xy) > self._capacity:
            # Too many points for the buffer: keep every k-th one.
            keep = np.linspace(0, len(points_xy) - 1, self._capacity).astype(int)
            points_xy = points_xy[keep]
            brightness = None if brightness is None else brightness[keep]
        count = len(points_xy)
        if count < 2:
            self._geometry.set_draw_range(0, 0)
            return
        # The direction of travel at each point (central differences), turned by 90
        # degrees, gives the direction in which the strip is widened.
        tangent = np.gradient(points_xy, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-6)
        side = np.column_stack((-tangent[:, 1], tangent[:, 0])) * self._half_width
        left = convert.ros_xy_to_three(points_xy + side, self._height)
        right = convert.ros_xy_to_three(points_xy - side, self._height)
        # Two triangles per segment: (left0, right0, left1) and (right0, right1, left1).
        vertices = 6 * (count - 1)
        quads = self._positions[:vertices].reshape(count - 1, 6, 3)
        quads[:, 0], quads[:, 1], quads[:, 2] = left[:-1], right[:-1], left[1:]
        quads[:, 3], quads[:, 4], quads[:, 5] = right[:-1], right[1:], left[1:]
        self._geometry.update_attribute('position', self._positions)
        if brightness is not None:
            shade = self._colors[:vertices].reshape(count - 1, 6, 3)
            start, end = brightness[:-1, None], brightness[1:, None]
            shade[:, 0], shade[:, 1], shade[:, 2] = start, start, end
            shade[:, 3], shade[:, 4], shade[:, 5] = start, end, end
            self._geometry.update_attribute('color', self._colors)
        self._geometry.set_draw_range(0, vertices)


class RobotMarker:
    """The robot: a puck with an arrow on top, inside a ring that is easy to find."""

    def __init__(self, parent: tp.Object3D) -> None:
        self._group = tp.Group()
        self._group.visible = False
        # Drawn one and a half times its real size: on a projector, a 0.5 m robot in
        # a 26 m building is a dot.
        self._group.scale.set(1.5, 1.5, 1.5)
        parent.add(self._group)

        body_material = tp.MeshStandardMaterial()
        body_material.color = 0xf3f6fa
        body_material.roughness = 0.4
        body = tp.Mesh(tp.CylinderGeometry(0.26, 0.26, 0.22, 32), body_material)
        body.position.y = 0.13
        self._group.add(body)

        # The arrow points along the group's local +x, which is the robot's forward
        # axis in ROS and in threepp alike (the rule three.x = ros.x leaves x alone).
        arrow = tp.BufferGeometry()
        tip, notch = (0.24, 0.0), (-0.07, 0.0)
        back_left, back_right = (-0.17, 0.17), (-0.17, -0.17)
        outline = np.float32([tip, back_left, notch, tip, notch, back_right])
        arrow.set_attribute('position', convert.ros_xy_to_three(outline, 0.245))
        self._group.add(tp.Mesh(arrow, _basic(0x0c1a12)))

        ring = tp.Mesh(tp.RingGeometry(0.50, 0.58, 48), _basic(palette.ROBOT))
        ring.rotate_x(-math.pi / 2.0)
        ring.position.y = 0.03
        self._group.add(ring)

        # A wedge on the floor in front of the robot: the heading, readable from far.
        wedge = tp.BufferGeometry()
        wedge.set_attribute('position', convert.ros_xy_to_three(
            np.float32([(1.05, 0.0), (0.66, 0.20), (0.66, -0.20)]), 0.03))
        self._group.add(tp.Mesh(wedge, _basic(palette.ROBOT)))

    def set_pose(self, pose: Pose2D) -> None:
        self._group.position.set(pose.x, 0.0, -pose.y)
        # A yaw of theta about ROS z is a rotation of theta about threepp y.
        self._group.rotation.y = pose.yaw
        self._group.visible = True


class MarkerPool:
    """A set of identical objects whose number changes: frontiers, cones.

    Objects are created the first time they are needed and afterwards only shown,
    hidden and moved. Creating and destroying meshes every time a message arrives
    would allocate GPU buffers over and over.
    """

    def __init__(self, parent: tp.Object3D, make: Callable[[], tp.Object3D]) -> None:
        self._parent = parent
        self._make = make
        self._items: List[tp.Object3D] = []

    def show(self, points_xy: np.ndarray) -> None:
        points = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2).tolist()
        while len(self._items) < len(points):
            item = self._make()
            self._parent.add(item)
            self._items.append(item)
        for index, item in enumerate(self._items):
            item.visible = index < len(points)
            if item.visible:
                x, y = points[index]
                item.position.set(x, 0.0, -y)


def _make_frontier() -> tp.Object3D:
    sphere = tp.Mesh(tp.SphereGeometry(0.22, 20, 14), _basic(palette.FRONTIER, 0.9))
    sphere.position.y = 0.32
    group = tp.Group()
    group.add(sphere)
    return group


def _make_target() -> tp.Object3D:
    """The chosen frontier: a map pin with a ring on the floor."""
    group = tp.Group()
    head = tp.Mesh(tp.SphereGeometry(0.26, 24, 16), _basic(palette.TARGET))
    head.position.y = 1.45
    group.add(head)
    pin = tp.Mesh(tp.CylinderGeometry(0.035, 0.035, 1.45, 8), _basic(palette.TARGET))
    pin.position.y = 0.725
    group.add(pin)
    ring = tp.Mesh(tp.RingGeometry(0.34, 0.44, 40), _basic(palette.TARGET))
    ring.rotate_x(-math.pi / 2.0)
    ring.position.y = 0.04
    group.add(ring)
    return group


def _make_cone() -> tp.Object3D:
    """A traffic cone, drawn larger than life so it shows on a projector."""
    group = tp.Group()
    orange = tp.MeshStandardMaterial()
    orange.color = palette.CONE
    orange.emissive = 0x7a2c00
    orange.roughness = 0.5
    white = tp.MeshStandardMaterial()
    white.color = 0xffffff
    white.emissive = 0x666666
    radius, height = 0.22, 0.62
    cone = tp.Mesh(tp.ConeGeometry(radius, height, 24), orange)
    cone.position.y = height / 2.0 + 0.04
    group.add(cone)
    # The white band: a slice of a slightly fatter cone, between 40 % and 60 % height.
    band = tp.Mesh(tp.CylinderGeometry(radius * 0.4 + 0.01, radius * 0.6 + 0.01,
                                       height * 0.2, 24), white)
    band.position.y = height * 0.5 + 0.04
    group.add(band)
    foot = tp.Mesh(tp.BoxGeometry(0.52, 0.04, 0.52), orange)
    foot.position.y = 0.02
    group.add(foot)
    return group


class WorldView:
    """Everything the 3D pass draws. One set_* method per thing the robot believes."""

    def __init__(self) -> None:
        self.scene = tp.Scene()
        self.scene.background = tp.Background(palette.BACKGROUND)

        # Soft light from everywhere plus one lamp from the side, so the top and the
        # sides of a wall get different shades and the walls read as solid.
        self.scene.add(tp.HemisphereLight(0xdcecff, 0x1a2230, 1.3))
        lamp = tp.DirectionalLight(0xffffff, 1.6)
        lamp.position.set(-6.0, 14.0, 9.0)
        self.scene.add(lamp)

        # A faint 1 m grid under everything: it gives the unknown a scale. It is 60 m
        # wide, so it reaches far beyond the building wherever the map origin is.
        grid_lines = tp.GridHelper(60, 60, tp.Color(0x1c2a3d), tp.Color(0x121b29))
        grid_lines.position.y = -0.02   # just below the floor, to avoid flicker
        self.scene.add(grid_lines)

        self._floor = FloorMap(self.scene)
        self._walls = Walls(self.scene)
        self._trail = Ribbon(self.scene, TRAIL_POINTS, 0.12, 0.02, palette.ROBOT, glow=True)
        self._plan = Ribbon(self.scene, 2048, 0.11, 0.07, palette.PLAN, glow=False)
        self._frontiers = MarkerPool(self.scene, _make_frontier)
        self._cones = MarkerPool(self.scene, _make_cone)
        self._target = _make_target()
        self._target.visible = False
        self.scene.add(self._target)
        self._robot = RobotMarker(self.scene)
        self._scan = self._make_scan_points()

        self._trail_points: Deque[Tuple[float, float]] = deque(maxlen=TRAIL_POINTS)
        self.robot_pose: Optional[Pose2D] = None

    def _make_scan_points(self) -> tp.BufferGeometry:
        geometry = tp.BufferGeometry()
        self._scan_buffer = np.zeros((2048, 3), dtype=np.float32)
        geometry.set_attribute('position', self._scan_buffer)
        geometry.set_draw_range(0, 0)
        material = tp.PointsMaterial()
        material.color = palette.LIDAR
        material.size = 0.2
        # A lidar hit lies ON a wall, so it would be buried inside the wall's box.
        # Without the depth test the points are painted over whatever is in front.
        material.depth_test = False
        points = tp.Points(geometry, material)
        points.frustum_culled = False
        points.render_order = 10
        self.scene.add(points)
        return geometry

    def animate(self) -> None:
        """Advance what moves by itself. Called once per frame, before drawing."""
        self._walls.animate()

    def set_grid(self, grid: Grid) -> None:
        self._floor.update(grid)
        self._walls.update(grid)

    def set_robot(self, pose: Pose2D) -> None:
        self._robot.set_pose(pose)
        self.robot_pose = pose
        self._extend_trail(pose)

    def _extend_trail(self, pose: Pose2D) -> None:
        if self._trail_points:
            last_x, last_y = self._trail_points[-1]
            moved = math.hypot(pose.x - last_x, pose.y - last_y)
            if moved < TRAIL_SPACING:
                return
            if moved > TELEPORT_DISTANCE:
                self._trail_points.clear()   # the world was reset: start a new trail
        self._trail_points.append((pose.x, pose.y))
        count = len(self._trail_points)
        # Brightness by age: the newest point is 1, and it falls to a faint 0.12 over
        # TRAIL_FADE_POINTS points.
        age = np.arange(count - 1, -1, -1, dtype=np.float32)
        brightness = np.clip(1.0 - age / TRAIL_FADE_POINTS, 0.12, 1.0)
        self._trail.set_points(np.float32(self._trail_points), brightness)

    def clear_trail(self) -> None:
        """Forget where the robot has been (the operator pressed reset)."""
        self._trail_points.clear()
        self._trail.set_points(np.empty((0, 2), dtype=np.float32))

    def set_scan(self, scan: Scan) -> None:
        count = min(len(scan.points_xy), len(self._scan_buffer))
        self._scan_buffer[:count] = convert.ros_xy_to_three(scan.points_xy[:count],
                                                            scan.height)
        self._scan.update_attribute('position', self._scan_buffer)
        self._scan.set_draw_range(0, count)

    def set_plan(self, points_xy: np.ndarray) -> None:
        self._plan.set_points(points_xy)

    def set_frontiers(self, frontiers: Frontiers) -> None:
        self._frontiers.show(frontiers.points_xy)
        self._target.visible = frontiers.target_xy is not None
        if frontiers.target_xy is not None:
            self._target.position.set(float(frontiers.target_xy[0]), 0.0,
                                      -float(frontiers.target_xy[1]))

    def set_cones(self, points_xy: np.ndarray) -> None:
        self._cones.show(points_xy)
