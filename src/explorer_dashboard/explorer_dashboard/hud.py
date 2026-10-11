"""The head-up display: 2D text drawn on top of the 3D picture. threepp only.

A HUD is a second, tiny scene. It is rendered after the 3D scene, without clearing the
picture in between, through an orthographic camera whose units are pixels: x from 0
(left) to the window width, y from 0 (bottom) to the window height.

Render thread only, like everything that calls threepp (see mailbox.py).
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import threepp as tp

from explorer_dashboard import palette
from explorer_dashboard.world_model import RobotState, Status

TEXT = 0xe8f1ff
DIM = 0x8fa3bf
PANEL = 0x0b111b
ALERT = 0xff3b4e
NOTICE = 0xffd23f
STATE_COLORS: Dict[RobotState, int] = {
    RobotState.UNKNOWN: 0x8fa3bf,
    RobotState.IDLE: 0xc9d6ea,
    RobotState.EXPLORING: palette.ROBOT,
    RobotState.RETURNING: palette.PLAN,
    RobotState.NAVIGATING: palette.FRONTIER,
    RobotState.DONE: palette.WALL,
}
KEY_HELP = ('E explore    C cancel    R reset    G goal at cursor    F follow robot'
            '    mouse: orbit, zoom, pan')


@dataclass(frozen=True)
class HudData:
    """Everything the HUD shows in one frame."""

    status: Status
    safety_stop: bool
    follow: bool
    waiting_for: Sequence[str]   # what has not been heard from yet, e.g. 'map'
    notices: Sequence[str]       # short messages for the operator, newest last


def format_clock(seconds: float) -> str:
    """90.4 -> '01:30'."""
    whole = int(max(seconds, 0.0))
    return f'{whole // 60:02d}:{whole % 60:02d}'


class Label:
    """One line of text that only rebuilds its geometry when the text changes.

    Text2D turns a string into triangles. Doing that for every label in every frame
    would be wasted work: most of the HUD is the same from one frame to the next.
    """

    def __init__(self, scene: tp.Scene, font: tp.Font, size: float, color: int) -> None:
        self._font = font
        self._size = size
        self._text = ''
        self._color = color
        self._mesh = tp.Text2D(font, '', size=size)
        self._mesh.set_color(tp.Color(color))
        # The HUD is drawn over a finished 3D picture whose depth buffer is still
        # there. Without this, a wall close to the camera could hide the text.
        self._mesh.material.depth_test = False
        # A renderer draws all opaque objects first and the transparent ones after
        # them. The panels are transparent, so the text must be in the transparent
        # group too, or the panels would be painted over it. Inside that group,
        # render_order decides: panels (1) first, text (2) last.
        self._mesh.material.transparent = True
        self._mesh.render_order = 2
        scene.add(self._mesh)

    def set(self, text: str, color: int) -> None:
        if text != self._text:
            self._mesh.set_text(text)
            self._text = text
        if color != self._color:
            self._mesh.set_color(tp.Color(color))
            self._color = color

    def place(self, x: float, y: float) -> None:
        """Put the left end of the baseline at pixel (x, y)."""
        self._mesh.position.set(x, y, 0.0)

    def show(self, visible: bool) -> None:
        self._mesh.visible = visible

    def width(self) -> float:
        return self._font.advance(self._text, self._size)


class Panel:
    """A translucent dark rectangle behind a group of labels, for contrast."""

    def __init__(self, scene: tp.Scene, color: int, opacity: float) -> None:
        material = tp.MeshBasicMaterial()
        material.color = color
        material.transparent = True
        material.opacity = opacity
        material.depth_test = False
        self.mesh = tp.Mesh(tp.PlaneGeometry(1.0, 1.0), material)
        self.mesh.render_order = 1   # panels first, then the labels on top of them
        scene.add(self.mesh)

    def place(self, left: float, bottom: float, width: float, height: float) -> None:
        self.mesh.scale.set(width, height, 1.0)
        self.mesh.position.set(left + width / 2.0, bottom + height / 2.0, -1.0)


class CameraPanel:
    """The robot's camera picture as a rectangle in the HUD, with a thin frame."""

    def __init__(self, scene: tp.Scene, font: tp.Font) -> None:
        self._frame = Panel(scene, palette.ROBOT, 1.0)
        self._frame.mesh.visible = False
        self._material = tp.MeshBasicMaterial()
        self._material.depth_test = False
        # In the transparent group like everything else in the HUD, so render_order
        # can put it on top of its frame.
        self._material.transparent = True
        self._mesh = tp.Mesh(tp.PlaneGeometry(1.0, 1.0), self._material)
        self._mesh.render_order = 2
        self._mesh.visible = False
        scene.add(self._mesh)
        self._caption = Label(scene, font, 14, DIM)
        self._caption.set('ROBOT CAMERA, with detections', DIM)
        self._caption.show(False)
        self._texture: Optional[tp.Texture] = None
        self._shape: Optional[Tuple[int, ...]] = None

    def place(self, left: float, bottom: float, width: float, height: float) -> None:
        self._frame.place(left - 2.0, bottom - 2.0, width + 4.0, height + 4.0)
        self._mesh.scale.set(width, height, 1.0)
        self._mesh.position.set(left + width / 2.0, bottom + height / 2.0, 0.0)
        self._caption.place(left, bottom + height + 8.0)

    def set_image(self, image: np.ndarray) -> None:
        """Show a new picture: (H, W, 3) RGB bytes, bottom row first."""
        if image.shape != self._shape:
            self._texture = tp.data_texture(image, srgb=True)
            self._texture.needs_update()
            self._material.map = self._texture
            self._material.needs_update()
            self._shape = image.shape
        else:
            # Same size as the last picture: overwrite the pixels, keep the texture.
            self._texture.update_data(image)
        self._mesh.visible = True
        self._frame.mesh.visible = True
        self._caption.show(True)


class Hud:
    """State, numbers, camera picture, legend, safety stop banner, messages, key help.

    Layout: a column on the left holds the status, the camera picture and the legend.
    The 3D view is framed so that the map fills the rest (see DashboardApp).
    """

    ROWS = ('Explored', 'Distance', 'Elapsed', 'Frontiers')
    NOTICE_LINES = 3

    def __init__(self, width: int, height: int) -> None:
        self.scene = tp.Scene()
        self.camera = tp.OrthographicCamera(0.0, float(width), float(height), 0.0,
                                            -10.0, 10.0)
        font = tp.FontLoader().default_font()   # built into threepp: no font file

        self._status_panel = Panel(self.scene, PANEL, 0.82)
        self._help_panel = Panel(self.scene, PANEL, 0.82)
        self._stop_panel = Panel(self.scene, ALERT, 0.95)

        self._title = Label(self.scene, font, 15, DIM)
        self._title.set('WHAT THE ROBOT BELIEVES', DIM)
        self._state = Label(self.scene, font, 34, TEXT)
        self._names = [Label(self.scene, font, 19, DIM) for _ in self.ROWS]
        self._values = [Label(self.scene, font, 19, TEXT) for _ in self.ROWS]
        for label, name in zip(self._names, self.ROWS):
            label.set(name, DIM)
        self._follow = Label(self.scene, font, 15, DIM)
        self._waiting = Label(self.scene, font, 16, NOTICE)
        self._help = Label(self.scene, font, 16, TEXT)
        self._help.set(KEY_HELP, TEXT)
        self._notices = [Label(self.scene, font, 18, NOTICE)
                         for _ in range(self.NOTICE_LINES)]
        self._stop = Label(self.scene, font, 30, 0xffffff)
        self._stop.set('SAFETY STOP', 0xffffff)

        # The legend: one coloured square and one line of text per kind of object.
        self._camera_panel = CameraPanel(self.scene, font)
        self._legend_panel = Panel(self.scene, PANEL, 0.82)
        self._legend_swatches = [Panel(self.scene, color, 1.0)
                                 for color, _ in palette.LEGEND]
        self._legend_labels = [Label(self.scene, font, 15, TEXT) for _ in palette.LEGEND]
        for label, (_, text) in zip(self._legend_labels, palette.LEGEND):
            label.set(text, TEXT)

        self._width = 0
        self._height = 0
        self.resize(width, height)

    def resize(self, width: int, height: int) -> None:
        """Lay everything out again for a window of width x height pixels."""
        self._width, self._height = width, height
        self.camera.right = float(width)
        self.camera.top = float(height)
        self.camera.update_projection_matrix()

        left, top = 24.0, height - 24.0
        self._status_panel.place(left - 12.0, top - 226.0, 344.0, 238.0)
        self._title.place(left, top - 16.0)
        self._state.place(left, top - 62.0)
        for index in range(len(self.ROWS)):
            baseline = top - 98.0 - 28.0 * index
            self._names[index].place(left, baseline)
            self._values[index].place(left + 136.0, baseline)
        self._follow.place(left, top - 212.0)
        column_width = 344.0
        # To the right of the column, so it is never under the camera picture.
        self._waiting.place(left + column_width + 12.0, top - 16.0)

        # The camera picture, under the status panel, as wide as the column and in the
        # camera's own 16:9 shape.
        picture_width = column_width - 24.0
        picture_height = picture_width * 9.0 / 16.0
        picture_bottom = top - 226.0 - 36.0 - picture_height
        self._camera_panel.place(left, picture_bottom, picture_width, picture_height)

        # The legend, under the picture.
        legend_top = picture_bottom - 22.0
        legend_height = 22.0 * len(palette.LEGEND) + 12.0
        self._legend_panel.place(left - 12.0, legend_top - legend_height + 12.0,
                                 column_width, legend_height)
        for index, label in enumerate(self._legend_labels):
            baseline = legend_top - 16.0 - 22.0 * index
            self._legend_swatches[index].place(left, baseline - 1.0, 13.0, 13.0)
            label.place(left + 26.0, baseline)

        self._help_panel.place(0.0, 0.0, float(width), 38.0)
        self._help.place(left, 13.0)
        # Messages for the operator: bottom of the 3D view, clear of the column.
        for index, notice in enumerate(self._notices):
            notice.place(left + column_width + 12.0, 52.0 + 26.0 * index)
        self._place_stop_banner()

    def _place_stop_banner(self) -> None:
        text_width = self._stop.width()
        centre = self._width / 2.0
        self._stop_panel.place(centre - text_width / 2.0 - 28.0, self._height - 78.0,
                               text_width + 56.0, 56.0)
        self._stop.place(centre - text_width / 2.0, self._height - 62.0)

    def set_camera_image(self, image: np.ndarray) -> None:
        self._camera_panel.set_image(image)

    def update(self, data: HudData) -> None:
        status = data.status
        self._state.set(status.state.value, STATE_COLORS[status.state])
        values = (f'{status.explored_area:.1f} m2', f'{status.distance:.1f} m',
                  format_clock(status.elapsed), str(status.frontier_count))
        for label, value in zip(self._values, values):
            label.set(value, TEXT)
        self._follow.set('camera: following the robot' if data.follow
                         else 'camera: free', palette.ROBOT if data.follow else DIM)

        # Tells the operator at a glance which node is not running yet.
        self._waiting.set('waiting for: ' + ', '.join(data.waiting_for)
                          if data.waiting_for else '', NOTICE)

        # The banner must be impossible to miss, and impossible to see by mistake.
        self._stop_panel.mesh.visible = data.safety_stop
        self._stop.show(data.safety_stop)

        # Newest notice at the bottom, older ones stacked above it.
        newest_first: List[str] = list(reversed(data.notices))[:self.NOTICE_LINES]
        for index, label in enumerate(self._notices):
            label.set(newest_first[index] if index < len(newest_first) else '', NOTICE)
