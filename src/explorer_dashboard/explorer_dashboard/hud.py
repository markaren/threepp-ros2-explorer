"""The head-up display: the 2D overlay drawn on top of the 3D picture. threepp only.

It is built on `threepp.lesson.Hud`, the toolkit that draws the overlays of the lab's
lesson films: rounded panels, system fonts, circles, pictures. That toolkit is
IMMEDIATE MODE: nothing is created once and updated later. We call begin(), then
describe the whole overlay from scratch (a panel here, a text there), then end(). The
toolkit keeps a pool of meshes behind the scenes and reuses them, so "drawing the same
card again" does not build a new mesh. The gain for us is that there is no layout
state to keep in sync: what is on screen is exactly what the last description said.

A film describes its overlay every frame, because in a film everything moves. Our
numbers change twice a second, so update() only describes the overlay again when
something it shows has changed (see update()). In between, the meshes of the last
description simply stay where they are.

Coordinates are pixels in a DESIGN SPACE with the origin in the top-left corner. Ours
is always 720 units high and as wide as the window's shape demands (1280 for 16:9), no
matter how many real pixels the window has. So the overlay scales with the window: on
a full-screen projector the text grows with the picture and stays readable from the
back row.

Render thread only, like everything that calls threepp (see mailbox.py).
"""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Sequence, Tuple

import numpy as np
import threepp as tp
from threepp.lesson import Hud as Toolkit

from explorer_dashboard import palette
from explorer_dashboard.world_model import RobotState, Status

# The design language, shared with the simulator's window.
TEXT = 0xf2f5fa      # values
DIM = 0x9fb2cc       # labels
FAINT = 0x70839e     # the least important text
CARD = 0x0d131e      # the fill of a card
ACCENT = 0x19d3ff    # the one accent colour: frames and small highlights
ALERT = 0xe8324a
NOTICE = 0xffd23f
STATE_COLORS: Dict[RobotState, int] = {
    RobotState.UNKNOWN: 0x8fa3bf,
    RobotState.IDLE: 0xc9d6ea,
    RobotState.EXPLORING: palette.ROBOT,
    RobotState.RETURNING: palette.PLAN,
    RobotState.NAVIGATING: palette.FRONTIER,
    RobotState.DONE: palette.WALL,
}

DESIGN_HEIGHT = 720.0    # the height of the design space, whatever the window size
MARGIN = 16.0            # from the window edge to a card
GAP = 12.0               # between two cards
PAD = 16.0               # from the edge of a card to its content
COLUMN = 344.0           # the width of the left column
STRIP_HEIGHT = 36.0      # the key help strip at the bottom
CHART_SIZE = (312, 30)   # the area chart in the status card, in design pixels
HISTORY_SECONDS = 1.0    # one chart sample per second of exploring

KEYS = (('E', 'explore'), ('C', 'cancel'), ('R', 'reset'), ('G', 'goal at cursor'),
        ('F', 'follow robot'))
MOUSE_HELP = 'mouse: orbit, zoom, pan'
CAMERA_TOPIC = 'camera/annotated/compressed'


@dataclass(frozen=True)
class HudData:
    """Everything the HUD shows in one frame."""

    status: Status
    cone_count: int              # how many cones the cone spotter has reported
    safety_stop: bool
    follow: bool
    waiting_for: Sequence[str]   # what has not been heard from yet, e.g. 'map'
    notices: Sequence[str]       # short messages for the operator, newest last


def format_clock(seconds: float) -> str:
    """90.4 -> '01:30'."""
    whole = int(max(seconds, 0.0))
    return f'{whole // 60:02d}:{whole % 60:02d}'


class Hud:
    """Status, camera picture and legend in a left column; the rest floats over the map.

    Layout (the 3D view is framed so that the map fills what the column leaves free,
    see DashboardApp):

        +----------+---------------------------------+
        | status   | waiting for      SAFETY STOP    |
        | camera   |                                 |
        | legend   | operator log       (the map)    |
        +----------+                                 |
        | key caps ...................               |
        +--------------------------------------------+
    """

    def __init__(self, width: int, height: int) -> None:
        # pool_limit: how many meshes the toolkit keeps ready. Every new value ("01:15",
        # "01:16", ...) is a new text mesh, and beyond the limit the toolkit frees the
        # ones that were drawn longest ago, so the pool cannot grow without bound.
        self._ui = Toolkit(width, height, pool_limit=500)
        # The window renders this scene, through this camera, after the 3D pass.
        self.scene = self._ui.scene
        self.camera = self._ui.camera
        self._shown: Optional[HudData] = None   # what the meshes on screen describe
        self._stale = True                      # True: describe again, data or not
        self.resize(width, height)

        self._camera_texture: Optional[tp.Texture] = None
        self._camera_shape: Optional[Tuple[int, ...]] = None

        # The chart is a small picture that we paint ourselves with numpy and show
        # with the toolkit's image(): one mesh and one draw call, however long the
        # history is. Twice the design size, so it stays sharp when scaled up.
        self._history: Deque[Tuple[float, float]] = deque(maxlen=3600)
        self._chart_pixels = np.zeros((2 * CHART_SIZE[1], 2 * CHART_SIZE[0], 4), np.uint8)
        self._chart_pixels[..., 0] = (ACCENT >> 16) & 0xff
        self._chart_pixels[..., 1] = (ACCENT >> 8) & 0xff
        self._chart_pixels[..., 2] = ACCENT & 0xff
        self._chart_texture = tp.data_texture(self._chart_pixels, srgb=True)

        # The text of the key strip never changes, so it is measured once.
        self._key_widths = [22.0 + 7.0 + self._ui.text_width(action, 13)
                            for _, action in KEYS]
        help_width = self._ui.text_width(MOUSE_HELP, 13)
        self._strip_width = 2 * 14.0 + sum(self._key_widths) + 18.0 * len(KEYS) + help_width

    def resize(self, width: int, height: int) -> None:
        """Fit the design space to a window of width x height real pixels.

        The design space keeps its height and takes the window's aspect ratio. Nothing
        else has to be laid out again: the next update() draws everything from scratch
        anyway. That is immediate mode paying off.
        """
        self._ui.H = DESIGN_HEIGHT
        self._ui.W = DESIGN_HEIGHT * width / max(height, 1)
        self.camera.right = self._ui.W
        self.camera.top = self._ui.H
        self.camera.update_projection_matrix()
        self._stale = True

    def set_camera_image(self, image: np.ndarray) -> None:
        """Show a new camera picture: (H, W, 3) RGB bytes, bottom row first."""
        if image.shape != self._camera_shape:
            self._camera_texture = tp.data_texture(image, srgb=True)
            self._camera_texture.needs_update()
            self._camera_shape = image.shape
            self._stale = True   # the camera card must be described with the new texture
        else:
            # Same size as the last picture: overwrite the pixels, keep the texture.
            self._camera_texture.update_data(image)

    def column_fraction(self) -> float:
        """How much of the window's width the left column takes, 0 to 1.

        The 3D view asks, so it can frame the map in the part that stays free. The
        column has a fixed width in design units, so in a narrow window it takes a
        larger share.
        """
        return min((MARGIN + COLUMN + GAP) / self._ui.W, 0.45)

    def update(self, data: HudData) -> None:
        """Show `data`. Called every frame, but it only works when something changed.

        Describing the overlay costs a few milliseconds of Python (about 100 pooled
        objects to place and paint), and a frame has 16 of them in total. The data
        changes about twice a second, so comparing it with what is already on screen
        saves that work in 58 frames out of 60. HudData is a frozen dataclass, so ==
        compares every field. (A new camera picture needs no new description: it
        overwrites the pixels of the texture the camera card already shows.)
        """
        self._record(data.status)
        if data == self._shown and not self._stale:
            return
        self._shown = data
        self._stale = False
        ui = self._ui
        ui.begin()
        # Each call is drawn on top of the one before it: cards first, then content.
        bottom = self._status_card(MARGIN, MARGIN, data)
        bottom = self._camera_card(MARGIN, bottom + GAP)
        self._legend_card(MARGIN, bottom + GAP)
        self._key_strip(data.follow)
        map_left = MARGIN + COLUMN + GAP
        self._waiting_card(map_left, MARGIN, data.waiting_for)
        self._notice_card(map_left, data.notices)
        if data.safety_stop:
            self._stop_banner(map_left)
        ui.end()   # hides whatever was drawn last frame but not this one

    # ---- Building blocks ---------------------------------------------------------------

    def _card(self, x: float, y: float, width: float, height: float) -> None:
        """A dark, translucent, rounded rectangle with a hairline outline."""
        self._ui.panel(x, y, width, height, radius=13, fill=CARD, alpha=0.78,
                       outline=0xffffff, outline_alpha=0.12, width=1.0)

    def _label(self, x: float, y: float, text: str, color: int = DIM) -> None:
        """The title of a card: small, dim, upper case. It names, it never shouts.

        The toolkit could also letter-space it (tracking=...), but then it draws the
        text one glyph at a time: one mesh and one draw call per letter, every frame.
        Measured here, 60 title letters cost more render time than the whole 3D map,
        so only the SAFETY STOP banner, which is rarely on screen, uses tracking.
        """
        self._ui.text(x, y, text.upper(), size=11.5, color=color, kind='semibold')

    def _caption(self, x: float, y: float, text: str) -> None:
        """The name of one value: a smaller title."""
        self._ui.text(x, y, text.upper(), size=10.5, color=DIM, kind='semibold')

    # ---- The left column ------------------------------------------------------------------

    def _status_card(self, x: float, y: float, data: HudData) -> float:
        """State, six tiles of numbers and the chart. Returns the bottom edge."""
        ui = self._ui
        status = data.status
        height = 246.0
        left = x + PAD
        self._card(x, y, COLUMN, height)
        self._label(left, y + 14, 'What the robot believes')

        # The state: a word in its colour, with a dot that has a soft halo.
        color = STATE_COLORS[status.state]
        ui.circle(left + 8, y + 54, 12, fill=color, alpha=0.22)
        ui.circle(left + 8, y + 54, 6, fill=color)
        ui.text(left + 28, y + 54, status.state.value, size=28, color=color, kind='bold',
                anchor='lm')

        tiles = (
            ('Explored', f'{status.explored_area:.1f}', 'm\u00b2', TEXT),
            ('Distance', f'{status.distance:.1f}', 'm', TEXT),
            ('Elapsed', format_clock(status.elapsed), '', TEXT),
            ('Frontiers', str(status.frontier_count), '', TEXT),
            ('Cones found', str(data.cone_count), '', TEXT),
            ('View', 'FOLLOW' if data.follow else 'FREE', '', ACCENT if data.follow else TEXT),
        )
        tile_width = (COLUMN - 2 * PAD) / 3.0
        for index, (name, value, unit, value_color) in enumerate(tiles):
            tile_x = left + (index % 3) * tile_width
            tile_y = y + 84 + (index // 3) * 50
            self._caption(tile_x, tile_y, name)
            # One mesh per value. The toolkit also offers live=True, which draws a
            # string glyph by glyph from a pool of single characters. That is meant
            # for text that changes EVERY frame. Ours changes twice a second, and one
            # new mesh per new value is cheaper than five glyph meshes per value.
            baseline = tile_y + 36
            ui.text(tile_x, baseline, value, size=22, color=value_color, kind='numeric',
                    anchor='ls')
            if unit:
                ui.text(tile_x + ui.text_width(value, 22, 'numeric') + 4, baseline, unit,
                        size=12.5, color=DIM, anchor='ls')

        chart_y = y + 190
        self._label(left, chart_y, 'Explored area over time')
        ui.panel(left, chart_y + 18 + CHART_SIZE[1], CHART_SIZE[0], 1, radius=0,
                 fill=DIM, alpha=0.35)
        if len(self._history) >= 2:
            ui.image(left, chart_y + 18, CHART_SIZE[0], CHART_SIZE[1], self._chart_texture)
        return y + height

    def _camera_card(self, x: float, y: float) -> float:
        """The robot's camera picture in a thin accent frame. Returns the bottom edge."""
        ui = self._ui
        picture_width = COLUMN - 2 * PAD
        picture_height = picture_width * 9.0 / 16.0   # the camera's own 16:9 shape
        height = 38.0 + picture_height + PAD
        left, top = x + PAD, y + 38
        self._card(x, y, COLUMN, height)
        self._label(left, y + 14, 'Robot camera')
        ui.text(x + COLUMN - PAD, y + 15, CAMERA_TOPIC, size=10.5, color=FAINT,
                kind='mono', anchor='ra')
        if self._camera_texture is None:
            ui.panel(left, top, picture_width, picture_height, radius=3, fill=0x05080d,
                     alpha=0.9)
            ui.text(left + picture_width / 2, top + picture_height / 2, 'no picture yet',
                    size=13, color=FAINT, anchor='mm')
        else:
            ui.image(left, top, picture_width, picture_height, self._camera_texture)
        ui.outline(left - 1.5, top - 1.5, picture_width + 3, picture_height + 3,
                   color=ACCENT, alpha=0.9, width=1.5, radius=3)
        return y + height

    def _legend_card(self, x: float, y: float) -> None:
        """What each colour in the 3D view means: a dot and a name, in two columns."""
        ui = self._ui
        rows = (len(palette.LEGEND) + 1) // 2
        self._card(x, y, COLUMN, 38.0 + 20.0 * rows + 8.0)
        self._label(x + PAD, y + 14, 'Legend')
        for index, (color, name) in enumerate(palette.LEGEND):
            dot_x = x + PAD + 5 + (index // rows) * 146
            dot_y = y + 48 + (index % rows) * 20
            ui.circle(dot_x, dot_y, 5, fill=color)
            ui.text(dot_x + 14, dot_y, name, size=12.5, color=TEXT, anchor='lm')

    def _key_strip(self, follow: bool) -> None:
        """The keys as key caps: a small outlined box per key, then what it does."""
        ui = self._ui
        y = ui.H - MARGIN + 4 - STRIP_HEIGHT
        middle = y + STRIP_HEIGHT / 2
        self._card(MARGIN, y, self._strip_width, STRIP_HEIGHT)
        x = MARGIN + 14
        for (key, action), width in zip(KEYS, self._key_widths):
            # The F cap lights up while the camera follows: the strip shows state too.
            lit = follow and key == 'F'
            ui.panel(x, middle - 11, 22, 22, radius=5, fill=0x1b2536, alpha=0.95,
                     outline=ACCENT if lit else 0xffffff,
                     outline_alpha=0.9 if lit else 0.32, width=1.0)
            ui.text(x + 11, middle, key, size=12.5, color=ACCENT if lit else TEXT,
                    kind='semibold', anchor='mm')
            ui.text(x + 29, middle, action, size=13, color=DIM, anchor='lm')
            x += width + 18
        ui.text(x, middle, MOUSE_HELP, size=13, color=FAINT, anchor='lm')

    # ---- Over the map ------------------------------------------------------------------------

    def _waiting_card(self, x: float, y: float, waiting_for: Sequence[str]) -> None:
        """Which topics have not been heard yet: says at a glance which node is missing."""
        if not waiting_for:
            return
        ui = self._ui
        text = ',  '.join(waiting_for)
        self._card(x, y, max(ui.text_width(text, 14), 110.0) + 2 * 14, 56)
        ui.circle(x + 19, y + 18, 4, fill=NOTICE)
        self._label(x + 30, y + 12, 'Waiting for', color=NOTICE)
        ui.text(x + 14, y + 29, text, size=14, color=TEXT)

    def _notice_card(self, x: float, notices: Sequence[str]) -> None:
        """Messages for the operator, newest at the bottom, on a card of their own."""
        if not notices:
            return
        ui = self._ui
        lines = list(notices)[-3:]
        width = max(max(ui.text_width(line, 14) for line in lines), 120.0) + 2 * 14
        height = 36.0 + 22.0 * len(lines) + 8.0
        y = ui.H - MARGIN + 4 - STRIP_HEIGHT - GAP - height
        self._card(x, y, width, height)
        self._label(x + 14, y + 12, 'Operator log')
        for index, line in enumerate(lines):
            newest = index == len(lines) - 1
            ui.text(x + 14, y + 34 + 22 * index, line, size=14,
                    color=TEXT if newest else DIM)

    def _stop_banner(self, map_left: float) -> None:
        """Impossible to miss, and never shown by mistake: red, top centre of the map."""
        ui = self._ui
        title, size, tracking = 'SAFETY STOP', 26, 3.0
        reason = 'something is too close: forward motion is blocked'
        letters = sum(ui.text_width(letter, size, 'bold') for letter in title)
        title_width = letters + tracking * (len(title) - 1)
        # As wide as the longer of its two lines, plus room for the warning sign.
        width = max(title_width, ui.text_width(reason, 12)) + 92.0
        height = 66.0
        x = (map_left + ui.W) / 2 - width / 2
        y = MARGIN
        ui.panel(x, y, width, height, radius=14, fill=ALERT, alpha=0.96,
                 outline=0xffffff, outline_alpha=0.5, width=1.5)
        ui.circle(x + 36, y + height / 2, 14, fill=0xffffff)
        ui.text(x + 36, y + height / 2, '!', size=21, color=ALERT, kind='bold',
                anchor='mm')
        ui.text(x + 64, y + 25, title, size=size, color=0xffffff, kind='bold',
                anchor='lm', tracking=tracking)
        ui.text(x + 65, y + 49, reason, size=12, color=0xffffff, alpha=0.9, anchor='lm')

    # ---- The chart ---------------------------------------------------------------------------

    def _record(self, status: Status) -> None:
        """Keep one (elapsed, explored area) sample per second and repaint the chart."""
        if self._history and status.elapsed < self._history[-1][0] - HISTORY_SECONDS:
            self._history.clear()   # the clock jumped back: a new job has started
            self._stale = True
        if self._history and status.elapsed < self._history[-1][0] + HISTORY_SECONDS:
            return
        self._history.append((status.elapsed, status.explored_area))
        if len(self._history) >= 2:
            self._paint_chart()

    def _paint_chart(self) -> None:
        """Paint the history as a filled curve into the chart's pixels (numpy only)."""
        rows, columns = self._chart_pixels.shape[:2]
        seconds, areas = np.float32(self._history).T
        # The height of the curve at every pixel column, in pixel rows (row 0 is the
        # bottom of the picture, as in every threepp data texture).
        curve = np.interp(np.linspace(seconds[0], seconds[-1], columns), seconds, areas)
        curve = 2.0 + curve / max(float(areas.max()), 1e-6) * (rows - 5.0)
        row = np.arange(rows)[:, None]
        # The line on top spans from the previous column's height to this one's, so a
        # steep rise is a connected stroke and not a row of loose dots.
        before = np.concatenate((curve[:1], curve[:-1]))
        low = np.minimum(curve, before) - 1.5
        high = np.maximum(curve, before) + 1.5
        on_line = (row >= low) & (row <= high)
        # Only the alpha channel carries the shape: opaque line, faint fill below it.
        self._chart_pixels[..., 3] = np.where(on_line, 255, np.where(row < curve, 64, 0))
        self._chart_texture.update_data(self._chart_pixels)
