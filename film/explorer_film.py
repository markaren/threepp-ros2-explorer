"""A robot explores a warehouse: the showcase film of this demo, made with threepp.lesson.

    python explorer_film.py --remeasure                 # say every line once, write the .speech.json
    python explorer_film.py                             # the film (1080p30), lesson_out/explorer_film.mp4
    python explorer_film.py --stills 5,40,90            # single frames
    python explorer_film.py --sheet                     # contact sheet, one frame every 3 s
    python explorer_film.py --preview --no-voice        # 960x540, silent, fast

Under three minutes: what the demo is, how its six nodes are connected, where threepp sits in a
ROS 2 project, how to run it, and how it was made.

Every moving picture comes from ONE real run of the demo (see README.md in this folder for how
it was recorded): the dashboard's own window and the robot's camera with the cone spotter's
boxes were captured while it ran, and the simulator's views were drawn again afterwards, by the
same simulator, from the robot's recorded poses. The footage is not in the repository; point
EXPLORER_FOOTAGE at it (default: footage/ next to this file).

The film itself is threepp too: text, cards, the node graph and the video panes are a
`threepp.lesson.Hud`, rendered by threepp's OpenGL renderer and piped to ffmpeg.
"""
from __future__ import annotations

import atexit
import csv
import json
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np

import threepp as tp
from threepp.lesson import (DIM, TEXT, VOICE_LEAD, Hud, Narration, Speech, Stage, ease_out, envelope, fade_in_out,
                            ffmpeg_exe, remap, run, smooth)

FPS = 30
_HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(_HERE)
FOOTAGE = os.environ.get("EXPLORER_FOOTAGE", os.path.join(_HERE, "footage"))
TAKE = os.path.join(FOOTAGE, "take1")        # the live run: dashboard video, camera images, index
REPLAY = os.path.join(FOOTAGE, "replay")     # the simulator's views of that run, drawn afterwards

C_CPP = 0x6cb6ff          # C++
C_PY = 0xffd866           # Python
C_TP = 0x7ee787           # threepp
C_ROS = 0xb9c6dc          # a topic
C_CONE = 0xff8a3d
INK = 0x05070b
CARD = 0x0f1622

# ── the script ────────────────────────────────────────────────────────────────
CAPTION_TEXT = {
    "o1": "A robot explores a warehouse it has never seen. It is a ROS 2 demo for students, and threepp is its "
          "world, its sensors and its screen.",
    "w1": "The simulator is one C++ node. threepp draws the warehouse, and plays the robot's hardware.",
    "w2": "The lidar is a ray caster against the scene. The camera is a second render, from the robot's own eye.",
    "g1": "Everything else only sees ROS 2 topics. So each node can be written in the language that suits it.",
    "g2": "A mapper turns laser scans into a map. An explorer picks a frontier, the edge of what is known, and "
          "plans a path there.",
    "g3": "A safety monitor, in C++, sits between the planner and the motors. It stops the robot before it hits "
          "anything.",
    "g4": "A cone spotter finds traffic cones in the camera image, and uses the lidar to place them on the map.",
    "g5": "And the dashboard is threepp again, this time from Python. It draws what the robot believes.",
    "b1": "Left, the world as it is. Right, the world as the robot believes it to be, built from its own scans.",
    "b2": "It drives to a frontier, the map grows, and it picks the next one. When none are left, it drives home.",
    "f1": "In C++, CMake fetches threepp and links it like any other library. The render loop keeps the main "
          "thread, and ROS 2 spins on a second one.",
    "f2": "In Python, it is a wheel from PyPI, installed next to ROS 2 by the same tool.",
    "u1": "Running it takes three commands. Pixi brings ROS 2 from RoboStack, and threepp with it.",
    "a1": "The demo was built in two to three hours by an AI coding agent, Claude Code, with a person steering.",
    "a2": "Sub-agents wrote the Python packages in parallel, against one written interface. The result was checked "
          "in headless renders, full runs and unit tests.",
    "a3": "The first version worked, and looked like a game from the nineties. One line of feedback later, it "
          "looked like this.",
    "e1": "ROS 2 carries the messages. threepp is the world, the sensors and the screen.",
}
# how the narrator says the names on screen
WORDS = {"ROS 2": "Ross two", "C++": "C plus plus", "CMake": "C make", "PyPI": "pie P I", "Pixi": "Pixie",
         "RoboStack": "Robo Stack", "AI": "A I"}
SPEECH = Speech(CAPTION_TEXT, os.path.join(_HERE, "explorer_film.speech.json"), words=WORDS,
                voice_only={"o1", "e1"})     # read over the title and the end card, not drawn
PLAN = [("open", 0.5, ["o1"], 0.3), ("world", 0.3, ["w1", "w2"], 0.5),
        ("graph", 0.6, ["g1", "g2", "g3", "g4", "g5"], 1.0), ("belief", 0.6, ["b1", "b2"], 1.2),
        ("fit", 0.6, ["f1", "f2"], 0.6), ("use", 0.5, ["u1"], 1.6), ("made", 0.5, ["a1", "a2", "a3"], 1.4),
        ("end", 0.5, ["e1"], 3.2)]
TL, CAP = SPEECH.layout(PLAN, gap=0.35)

# Where on the run's clock (seconds from the first recorded frame) each stretch of footage starts.
HERO0 = 79.5        # the opening: the robot drives up to a cone
FIT0 = 108.0        # behind the cards of the second half: a long aisle
END0 = 168.0        # the ending: the drive home
RUN_END = 198.0
WIPE_AT = 60.0      # the "after" picture of the before/after


def c0(k):
    return CAP[k][0]


def c1(k):
    return CAP[k][1]


# ── footage ───────────────────────────────────────────────────────────────────
class Video:
    """A video decoded by ffmpeg at `size`, optionally cropped first (w, h, x, y in source
    pixels). Frames come in order; asking for an earlier one, or one far ahead, restarts the
    decoder there."""

    def __init__(self, path, fps, size, crop=None):
        self.path, self.fps, (self.w, self.h), self.crop = path, fps, size, crop
        self.proc, self.k, self.cur = None, -1, None
        if not os.path.isfile(path):
            raise FileNotFoundError(f"{path}: the film's footage is not in the repository, see film/README.md")
        atexit.register(self.close)

    def _start(self, k):
        self.close()
        vf = ([f"crop={self.crop[0]}:{self.crop[1]}:{self.crop[2]}:{self.crop[3]}"] if self.crop else [])
        vf.append(f"scale={self.w}:{self.h}:flags=bicubic")
        self.proc = subprocess.Popen(
            [ffmpeg_exe(), "-loglevel", "error", "-ss", f"{max(k - 0.5, 0.0) / self.fps:.5f}", "-i", self.path,
             "-vf", ",".join(vf), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], stdout=subprocess.PIPE, bufsize=0)
        self.k = k - 1

    def frame(self, k):
        k = max(int(k), 0)
        if self.proc is None or k < self.k or k > self.k + 12 * self.fps:
            if self.cur is None or k != self.k:
                self._start(k)
        size = self.w * self.h * 3
        while self.k < k:
            data = bytearray()
            while len(data) < size:
                chunk = self.proc.stdout.read(size - len(data))
                if not chunk:
                    return self.cur                       # past the end: hold the last frame
                data += chunk
            self.k += 1
            self.cur = np.frombuffer(bytes(data), np.uint8).reshape(self.h, self.w, 3)
        return self.cur

    def at(self, seconds):
        return self.frame(int(seconds * self.fps + 1e-6))

    def close(self):
        if self.proc is not None:
            try:
                self.proc.kill()
                self.proc.wait(timeout=5)
            except Exception:
                pass
            self.proc = None


def read_image(path, size=None):
    """An image file as (H, W, 3) uint8 RGB."""
    img = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(path)
    if size is not None:
        img = cv2.resize(img, size, interpolation=cv2.INTER_AREA if img.shape[1] > size[0] else cv2.INTER_CUBIC)
    return np.ascontiguousarray(img[:, :, ::-1])


class Take:
    """The live run: its clock, the dashboard's window, the robot's camera, and what happened when."""

    def __init__(self, camera_size):
        self.camera_size = camera_size                       # the size the camera's images are shown at
        with open(os.path.join(TAKE, "index.json")) as f:
            self.index = json.load(f)
        with open(os.path.join(TAKE, "track.json")) as f:
            self.t0 = json.load(f)["t0"]
        with open(os.path.join(TAKE, "dash_frames.csv")) as f:
            self.dash_times = np.array([float(r["unix_time"]) for r in csv.DictReader(f)]) - self.t0
        self.annotated_times = np.array(self.index["annotated"]) - self.t0
        self.status = np.array(self.index["status"])
        self.status[:, 0] -= self.t0
        self.cones = [(t - self.t0, len(c)) for t, c in self.index["cones"]]
        self._jpeg = (None, None)

    def dash_frame(self, rt):
        return max(int(np.searchsorted(self.dash_times, rt, side="right")) - 1, 0)

    def annotated(self, rt):
        """The robot's camera with the cone spotter's boxes, as it was published at run time rt."""
        k = min(max(int(np.searchsorted(self.annotated_times, rt, side="right")) - 1, 0), len(self.annotated_times) - 1)
        if self._jpeg[0] != k:
            self._jpeg = (k, read_image(os.path.join(TAKE, "annotated", f"{k:05d}.jpg"), self.camera_size))
        return self._jpeg[1]

    def state(self, rt):
        k = min(max(int(np.searchsorted(self.status[:, 0], rt, side="right")) - 1, 0), len(self.status) - 1)
        return int(self.status[k, 1])

    def cones_found(self, rt):
        return max((n for t, n in self.cones if t <= rt), default=0)


class Slot:
    """A texture that shows a new numpy image whenever it is given one."""

    def __init__(self):
        self.tex, self.shape, self.last = None, None, None

    def texture(self, img):
        if self.tex is not None and img is self.last:
            return self.tex
        self.last = img
        flipped = np.ascontiguousarray(img[::-1])          # textures take their rows bottom first
        if self.tex is None or self.shape != flipped.shape:
            self.tex, self.shape = tp.data_texture(flipped, True), flipped.shape
        else:
            self.tex.update_data(flipped)
        return self.tex


# ── the node graph ────────────────────────────────────────────────────────────
# key: (x, y, w, h, name, what it does, language, uses threepp)
NODES = {
    "sim": (85, 130, 330, 390, "sim", "the world, the robot's hardware", "C++", True),
    "safety": (630, 110, 340, 136, "safety_monitor", "the reflex: stop before a hit", "C++", False),
    "explorer": (1230, 110, 340, 136, "explorer", "picks frontiers, plans paths", "Python", False),
    "mapper": (630, 372, 340, 136, "mapper", "laser scans into a map", "Python", False),
    "cone": (630, 640, 340, 136, "cone_spotter", "cones, by camera and lidar", "Python", False),
    "dash": (1230, 500, 340, 320, "dashboard", "what the robot believes", "Python", True),
}
# (from, to, start, end, topic, where the label sits: fraction along, dx, dy, anchor)
EDGES = [
    ("sim", "mapper", (415, 440), (630, 440), "scan", (0.5, 0, -24, "m")),
    ("mapper", "explorer", (970, 420), (1270, 246), "map", (0.5, -34, -14, "m")),
    ("explorer", "safety", (1230, 178), (970, 178), "cmd_vel_nav", (0.5, 0, -24, "m")),
    ("sim", "safety", (415, 196), (630, 160), "scan", (0.5, 0, -26, "m")),
    ("safety", "sim", (630, 222), (415, 258), "cmd_vel", (0.5, 0, 26, "m")),
    ("sim", "cone", (330, 520), (630, 700), "camera/image, scan", (0.55, -40, 30, "m")),
    ("cone", "dash", (970, 708), (1230, 708), "cones", (0.5, 0, -24, "m")),
    ("mapper", "dash", (970, 470), (1230, 580), "map", (0.5, 30, -22, "m")),
    ("explorer", "dash", (1480, 246), (1480, 500), "plan, frontiers, status", (0.5, 16, 0, "l")),
    ("dash", "explorer", (1370, 500), (1370, 246), "explore (an action)", (0.5, -16, 0, "r")),
]
GRAPH_DX = 120      # the graph was laid out against the left edge; this centres it
NODES = {k: (v[0] + GRAPH_DX,) + v[1:] for k, v in NODES.items()}
EDGES = [(a, b, (p0[0] + GRAPH_DX, p0[1]), (p1[0] + GRAPH_DX, p1[1]), topic, label) for a, b, p0, p1, topic, label in EDGES]


class Painter:
    def __init__(self, ov, take, videos, stills, said):
        self.ov, self.take, self.v, self.stills, self.said = ov, take, videos, stills, said
        self.slots = {}
        self.captions = SPEECH.captions(CAP)
        # when each node, and each topic, enters the graph: on the word that names it
        s = said
        self.node_in = {"sim": TL.start("graph") + 0.2, "mapper": s("g2", "mapper", 0.02),
                        "explorer": s("g2", "explorer", 0.38), "safety": s("g3", "safety", 0.03),
                        "cone": s("g4", "cone", 0.03), "dash": s("g5", "dashboard", 0.08)}
        self.edge_in = [s("g2", "scans", 0.2), s("g2", "frontier", 0.5), s("g3", "planner", 0.45),
                        s("g3", "stops", 0.65), s("g3", "motors", 0.55), s("g4", "camera", 0.4),
                        c0("g5") + 0.9, c0("g5") + 1.2, c0("g5") + 1.5, c0("g5") + 1.8]

    # helpers ------------------------------------------------------------------
    def show(self, slot, img, rect, a=1.0):
        if a <= 0.003 or img is None:
            return
        self.ov.image(*rect, self.slots.setdefault(slot, Slot()).texture(img), alpha=a)

    def veil(self, k):
        if k > 0.003:
            self.ov.panel(-20, -20, 1960, 1120, radius=0, fill=INK, alpha=k)

    def chip(self, x, y, text, color, a, size=17, anchor="l"):
        """A small filled label. Returns its width."""
        ov = self.ov
        w = ov.text_width(text, size, "semibold") + 22
        x0 = x if anchor == "l" else x - w
        ov.panel(x0, y, w, size + 13, radius=(size + 13) / 2, fill=color, alpha=0.95 * a)
        ov.text(x0 + w / 2, y + (size + 13) / 2, text, size=size, color=0x0b0f17, alpha=a, kind="semibold", anchor="mm")
        return w

    def tag(self, x, y, title, sub, a, color=TEXT, anchor="l"):
        """A dark label with a title and a quieter second line."""
        if a <= 0.003:
            return
        ov = self.ov
        w = max(ov.text_width(title, 24, "semibold"), ov.text_width(sub, 19) if sub else 0) + 44
        h = 82 if sub else 50
        x0 = x if anchor == "l" else x - w
        ov.panel(x0, y, w, h, radius=14, fill=0x0d131e, alpha=0.84 * a)
        ov.text(x0 + 22, y + 25, title, size=24, color=color, alpha=a, kind="semibold", anchor="lm")
        if sub:
            ov.text(x0 + 22, y + 60, sub, size=19, color=DIM, alpha=a, anchor="lm")

    def pane(self, slot, img, rect, a, title=None, sub=None, color=TEXT):
        """A picture in a frame, with its name above it."""
        if a <= 0.003:
            return
        x, y, w, h = rect
        self.ov.panel(x - 6, y - 6, w + 12, h + 12, radius=10, fill=0x0d131e, alpha=0.9 * a)
        self.show(slot, img, rect, a)
        self.ov.outline(x - 1, y - 1, w + 2, h + 2, 0x8aa0c0, 0.35 * a, width=1.5, radius=2)
        if title:
            self.ov.text(x, y - 46, title, size=22, color=color, alpha=a, kind="semibold", tracking=3)
            if sub:
                tw = self.ov.text_width(title, 22, "semibold") + 3 * (len(title) - 1)
                self.ov.text(x + tw + 22, y - 44, sub, size=20, color=DIM, alpha=a)

    def mono_rows(self, x, y, rows, size, lh, a, t=None, t_in=0.0, stagger=0.3):
        """Lines of code, each a list of (text, colour) parts; they appear one after another."""
        for j, parts in enumerate(rows):
            al = a if t is None else a * smooth(remap(t, t_in + stagger * j, t_in + stagger * j + 0.35))
            if parts:
                self.ov.rich(x, y + j * lh, [(s, c, "mono") for s, c in parts], size=size, alpha=al, anchor="lm")

    def card(self, x, y, w, h, a, title=None, color=DIM):
        self.ov.panel(x, y, w, h, radius=18, fill=CARD, alpha=0.9 * a, outline=0x8aa0c0, outline_alpha=0.2)
        if title:
            self.ov.text(x + 28, y + 24, title, size=17, color=color, alpha=a, kind="semibold", tracking=2.4)

    # the film -------------------------------------------------------------------
    def draw(self, t):
        self.background(t)
        for beat in (self.opening, self.world, self.graph, self.belief, self.fit, self.use, self.made, self.end):
            beat(t)
        self.ov.captions(t, self.captions)

    def background(self, t):
        """The full-frame picture behind everything, and how far it is dimmed."""
        w2, g0, g1 = c0("w2"), TL.start("graph"), TL.end("graph")
        f0, m1, e0 = TL.start("fit"), TL.end("made"), TL.start("end")
        full = (0, 0, 1920, 1080)
        if t < w2 + 0.4:                                   # the chase camera, from the first frame
            self.show("chase", self.v["chase"].at(HERO0 + t), full, 1.0 - smooth(remap(t, w2, w2 + 0.4)))
        if w2 <= t < g1 + 0.5:                             # the overview: bright for the lidar, dim under the graph
            a = smooth(remap(t, w2, w2 + 0.4)) * (1.0 - smooth(remap(t, g1 - 0.1, g1 + 0.5)))
            self.show("over", self.v["over"].at(HERO0 + t), full, a)
            self.veil(0.86 * smooth(remap(t, g0 - 0.3, g0 + 0.6)) * a)
        cut = c0("a3") + 0.6                               # from here the before/after fills the frame
        if f0 - 0.5 <= t < cut:                            # an aisle, dim, behind the cards
            a = envelope(t, f0 - 0.5, cut, 0.6, 0.4)
            self.show("chase", self.v["chase"].at(FIT0 + (t - f0)), full, a)
            self.veil(0.8 * a)
        if t >= max(e0 - 0.4, cut):                        # the drive home
            self.show("chase", self.v["chase"].at(END0 + (t - e0)), full, smooth(remap(t, e0 - 0.4, e0 + 0.3)))

    def opening(self, t):
        # The title lies over the robot, so it comes after a moment of clear picture and
        # leaves before the robot reaches the cone.
        ti, to = 1.4, TL.end("open") - 2.4
        a = envelope(t, ti, to, 0.5, 0.5)
        if a <= 0.003:
            return
        ov = self.ov
        y0 = 664
        ov.panel(-20, y0 - 56, 1960, 500, radius=0, fill=INK, alpha=0.74 * a)
        rise = 22 * (1 - ease_out(remap(t, ti, ti + 0.6)))
        x0 = 130
        ov.rich(x0 + 4, y0 + rise, [("ROS 2", C_ROS, "semibold"), ("   C++", C_CPP, "semibold"),
                                    ("   PYTHON", C_PY, "semibold"), ("   THREEPP", C_TP, "semibold")],
                size=24, alpha=a, anchor="la")
        w = ov.text_width("A robot explores a warehouse", 104, "semibold")
        ov.text(x0, y0 + 148 + rise, "A robot explores a warehouse", size=104, color=TEXT, alpha=a, kind="semibold",
                anchor="ls")
        ov.panel(x0 + 4, y0 + 176 + rise, w * smooth(remap(t, ti + 0.2, ti + 0.8)), 6, radius=3, fill=C_TP, alpha=a)
        sa = a * smooth(remap(t, ti + 0.6, ti + 1.2))
        ov.text(x0 + 4, y0 + 222 + rise, "Six ROS 2 nodes in two languages, simulated and drawn with threepp",
                size=34, color=DIM, alpha=sa, kind="light")

    def world(self, t):
        s, e, w2 = TL.start("world"), TL.end("world"), c0("w2")
        if not s - 0.2 <= t <= e + 0.4:
            return
        ov = self.ov
        # the simulator, by name
        a = envelope(t, c0("w1") + 0.2, w2 - 0.1, 0.4, 0.3)
        if a > 0.003:
            ov.panel(70, 64, 590, 124, radius=16, fill=0x0d131e, alpha=0.86 * a)
            ov.text(96, 100, "explorer_sim", size=34, color=TEXT, alpha=a, kind="mono", anchor="lm")
            cx = 96 + ov.text_width("explorer_sim", 34, "mono") + 22
            cx += self.chip(cx, 85, "C++", C_CPP, a) + 10
            self.chip(cx, 85, "threepp", C_TP, a)
            ov.text(96, 152, "one node: the world, and the robot's hardware", size=22, color=DIM, alpha=a, anchor="lm")
        # the lidar, seen from above
        a = envelope(t, w2 + 0.3, e + 0.3, 0.4, 0.4)
        self.tag(70, 64, "LIDAR", "360 rays a scan, cast against the scene by threepp::Raycaster", a, color=0x4fd6ff)
        # the camera, through the robot's eye
        tc = self.said("w2", "camera", 0.52) - 0.25
        a = envelope(t, tc, e + 0.3, 0.45, 0.4)
        if a > 0.003:
            g = 0.94 + 0.06 * ease_out(remap(t, tc, tc + 0.5))
            w, h = 720 * g, 405 * g
            x, y = 1856 - w, 892 - h
            self.pane("eye", self.v["eye"].at(HERO0 + t), (x, y, w, h), a)
            self.tag(x + 14, y + 14, "CAMERA", "a second render, from the robot's eye", a, color=C_CONE)

    # the graph ------------------------------------------------------------------
    def node(self, key, t, a):
        x, y, w, h, name, role, lang, uses_tp = NODES[key]
        ov = self.ov
        ti = self.node_in[key]
        on = smooth(remap(t, ti, ti + 0.45))
        focus = envelope(t, ti, ti + 2.6, 0.3, 0.9)                      # just named: brighter for a moment
        color = C_CPP if lang == "C++" else C_PY
        ghost = 0.16 * a * (1 - on)
        if ghost > 0.003:
            ov.outline(x, y, w, h, 0x8aa0c0, ghost, width=1.5, radius=16)
        al = a * on
        if al <= 0.003:
            return
        ov.panel(x, y, w, h, radius=16, fill=0x121b2a, alpha=0.96 * al)
        ov.outline(x, y, w, h, color, (0.55 + 0.45 * focus) * al, width=2.0 + 1.5 * focus, radius=16)
        ov.text(x + 22, y + 34, name, size=28, color=TEXT, alpha=al, kind="mono", anchor="lm")
        ov.text(x + 22, y + 72, role, size=19, color=DIM, alpha=al, anchor="lm")
        cx = x + 22
        cx += self.chip(cx, y + 94, lang, color, al, size=15) + 8
        if uses_tp:
            self.chip(cx, y + 94, "threepp", C_TP, al, size=15)
        if key == "sim":                                                  # the two windows, live in their boxes
            self.show("g_sim", self.v["chase_s"].at(HERO0 + t), (x + 16, y + 140, 298, 168), al)
            ov.text(x + 22, y + 334, "lidar: a threepp Raycaster", size=18, color=DIM, alpha=al, anchor="lm")
            ov.text(x + 22, y + 362, "camera: a second render", size=18, color=DIM, alpha=al, anchor="lm")
        if key == "dash":
            k = self.take.dash_frame(HERO0 + t)
            self.show("g_dash", self.v["dash_s"].frame(k), (x + 16, y + 136, 308, 173), al)

    def edge(self, i, t, a):
        src, dst, p0, p1, topic, (frac, dx, dy, anchor) = EDGES[i]
        ti = self.edge_in[i]
        on = a * smooth(remap(t, ti, ti + 0.5))
        if on <= 0.003:
            return
        ov = self.ov
        p0, p1 = np.array(p0, float), np.array(p1, float)
        grow = ease_out(remap(t, ti, ti + 0.6))
        tip = p0 + (p1 - p0) * grow
        fresh = envelope(t, ti, ti + 2.4, 0.2, 1.0)
        col = TEXT if fresh > 0.5 else C_ROS
        ov.dashed(p0, tip, col, 3.0, (0.6 + 0.4 * fresh) * on, dash=12, gap=9, phase=-46 * (t - ti))
        if grow > 0.98:
            u = (p1 - p0) / np.linalg.norm(p1 - p0)
            ov.arrow2d(p1 - u * 16, p1, col, 3.0, 15, on)
        m = p0 + (p1 - p0) * frac + np.array([dx, dy], float)
        la = on * smooth(remap(t, ti + 0.25, ti + 0.7))
        tw = ov.text_width(topic, 18, "mono")
        x0 = m[0] - (tw / 2 if anchor == "m" else tw if anchor == "r" else 0)
        ov.panel(x0 - 9, m[1] - 15, tw + 18, 30, radius=8, fill=0x0a0f18, alpha=0.9 * la)
        ov.text(x0, m[1], topic, size=18, color=TEXT if fresh > 0.5 else C_ROS, alpha=la, kind="mono", anchor="lm")

    def graph(self, t):
        s, e = TL.start("graph"), TL.end("graph")
        a = envelope(t, s, e + 0.2, 0.5, 0.5)
        if a <= 0.003:
            return
        ov = self.ov
        ov.text(88 + GRAPH_DX, 62, "HOW THE PARTS ARE CONNECTED", size=22, color=DIM, alpha=a, kind="semibold", tracking=4)
        x = 1570 + GRAPH_DX
        for text, color in (("threepp", C_TP), ("Python", C_PY), ("C++", C_CPP)):
            x -= self.chip(x, 48, text, color, a, size=16, anchor="r") + 10
        ov.text(x - 6, 62, "one box, one ROS 2 node", size=19, color=DIM, alpha=a, anchor="rm")
        for i in range(len(EDGES)):
            self.edge(i, t, a)
        for key in NODES:
            self.node(key, t, a)

    # truth and belief -------------------------------------------------------------
    def belief(self, t):
        s, e = TL.start("belief"), TL.end("belief")
        a = envelope(t, s, e + 0.1, 0.5, 0.5)
        if a <= 0.003:
            return
        ov = self.ov
        ov.panel(-20, -20, 1960, 1120, radius=0, fill=0x080b12, alpha=a)
        rt = 3.0 + (RUN_END - 3.0) * remap(t, s + 0.8, e - 1.6)       # the whole run, in one beat
        left, right, cam = (24, 132, 924, 520), (972, 132, 924, 520), (768, 676, 384, 216)
        self.pane("b_over", self.v["over_b"].at(rt), left, a, "THE WORLD AS IT IS", "explorer_sim, C++", C_CPP)
        self.pane("b_dash", self.v["dash_b"].frame(self.take.dash_frame(rt)), right, a, "WHAT THE ROBOT BELIEVES",
                  "dashboard, Python", C_PY)
        self.pane("b_cam", self.take.annotated(rt), cam, a)
        ov.text(cam[0] - 30, cam[1] + 78, "WHAT ITS CAMERA SEES", size=22, color=C_CONE, alpha=a, kind="semibold",
                tracking=3, anchor="rm")
        ov.text(cam[0] - 30, cam[1] + 116, "the cone spotter's boxes, drawn by the node itself", size=20, color=DIM,
                alpha=a, anchor="rm")
        ov.text(cam[0] - 30, cam[1] + 146, "a colour threshold in OpenCV, not a neural network", size=20, color=DIM,
                alpha=a, anchor="rm")
        # what the explorer reports, and the cones it has placed
        x = cam[0] + cam[2] + 34
        state = {0: "IDLE", 1: "EXPLORING", 2: "DRIVING HOME", 3: "NAVIGATING", 4: "DONE"}[self.take.state(rt)]
        col = {"EXPLORING": C_TP, "DRIVING HOME": C_PY, "DONE": TEXT}.get(state, DIM)
        ov.circle(x + 9, cam[1] + 79, 8, fill=col, alpha=a)
        ov.text(x + 30, cam[1] + 78, state, size=26, color=col, alpha=a, kind="semibold", anchor="lm", tracking=2)
        ov.text(x, cam[1] + 124, "cones on the map", size=20, color=DIM, alpha=a, anchor="lm")
        n = self.take.cones_found(rt)
        for k in range(6):
            cx = x + ov.text_width("cones on the map", 20) + 28 + 26 * k
            ov.circle(cx, cam[1] + 124, 8, fill=C_CONE if k < n else 0x2a3342, alpha=a)
        ov.text(x, cam[1] + 158, "one run, played many times faster", size=20, color=DIM, alpha=a, anchor="lm")

    # where threepp sits -------------------------------------------------------------
    def fit(self, t):
        s, e = TL.start("fit"), TL.end("fit")
        a = envelope(t, s, e, 0.5, 0.45)
        if a <= 0.003:
            return
        ov = self.ov
        ident, fn, st, punc = 0xe6ecf5, 0x82aaff, 0xc3e88d, 0x8fa3bf
        tp_ = C_TP
        # C++
        self.card(90, 96, 850, 300, a, "C++     src/explorer_sim/CMakeLists.txt", C_CPP)
        self.mono_rows(122, 172, [
            [("FetchContent_Declare", fn), ("(", punc), ("threepp", tp_)],
            [("    GIT_REPOSITORY ", ident), ("https://github.com/markaren/threepp.git", st)],
            [("    GIT_TAG ", ident), ("2026-10-11", st), (")", punc)],
            [("FetchContent_MakeAvailable", fn), ("(", punc), ("threepp", tp_), (")", punc)],
            [("target_link_libraries", fn), ("(", punc), ("simulator PUBLIC ", ident), ("threepp::threepp", tp_), (")", punc)],
        ], 21, 40, a, t, c0("f1") + 0.4, 0.3)
        # Python
        t2 = c0("f2") - 0.2
        a2 = a * smooth(remap(t, t2, t2 + 0.5))
        self.card(980, 96, 850, 300, a2, "PYTHON     pixi.toml, and a node", C_PY)
        self.mono_rows(1012, 172, [
            [("[feature.jazzy.pypi-dependencies]", fn)],
            [("threepp", tp_), (" = ", punc), ('">=2026.10.11"', st)],
            [],
            [("import ", 0xc792ea), ("threepp", tp_), (" as ", 0xc792ea), ("tp", ident)],
            [("from ", 0xc792ea), ("threepp.lesson", tp_), (" import ", 0xc792ea), ("Hud", ident)],
        ], 21, 40, a2, t, t2 + 0.4, 0.3)
        # the threads of the C++ node
        t3 = self.said("f1", "render", 0.55) - 0.3
        a3 = a * smooth(remap(t, t3, t3 + 0.5))
        if a3 <= 0.003:
            return
        ov.text(96, 452, "INSIDE THE C++ NODE", size=20, color=DIM, alpha=a3, kind="semibold", tracking=3.5)
        ax, bx, y, w, h = 96, 1100, 500, 720, 170
        for x, title, sub, sub2, col in (
                (ax, "the main thread", "threepp: physics, the lidar, the camera, the window", "the render loop lives here", C_TP),
                (bx, "the ROS 2 threads", "rclcpp: an executor for callbacks, a publisher for sensor data", "ROS 2 spins here", C_ROS)):
            ov.panel(x, y, w, h, radius=18, fill=0x121b2a, alpha=0.96 * a3, outline=col, outline_alpha=0.6)
            ov.text(x + 30, y + 44, title, size=30, color=col, alpha=a3, kind="semibold", anchor="lm")
            ov.text(x + 30, y + 94, sub, size=21, color=TEXT, alpha=a3, anchor="lm")
            ov.text(x + 30, y + 130, sub2, size=20, color=DIM, alpha=a3, anchor="lm")
        x0, x1 = ax + w + 14, bx - 14
        la = a3 * smooth(remap(t, t3 + 0.5, t3 + 1.0))
        ov.dashed((x1, y + 52), (x0 + 16, y + 52), TEXT, 3.0, 0.9 * la, dash=12, gap=9, phase=-46 * t)
        ov.arrow2d((x0 + 16, y + 52), (x0, y + 52), TEXT, 3.0, 15, la)
        ov.dashed((x0, y + 122), (x1 - 16, y + 122), TEXT, 3.0, 0.9 * la, dash=12, gap=9, phase=-46 * t)
        ov.arrow2d((x1 - 16, y + 122), (x1, y + 122), TEXT, 3.0, 15, la)
        mx = (x0 + x1) / 2
        ov.text(mx, y + 26, "Mailbox<VelocityCommand>", size=17, color=C_ROS, alpha=la, kind="mono", anchor="mm")
        ov.text(mx, y + 148, "ThreadSafeQueue<Telemetry>", size=17, color=C_ROS, alpha=la, kind="mono", anchor="mm")
        ov.text(96, y + h + 44, "Both channels come from the course's book. No ROS in the threepp half of the node, "
                "no threepp in the ROS half.", size=22, color=DIM, alpha=la, anchor="lm")

    # how to run it ----------------------------------------------------------------------
    def use(self, t):
        s, e = TL.start("use"), TL.end("use")
        a = envelope(t, s, e, 0.5, 0.45)
        if a <= 0.003:
            return
        ov = self.ov
        self.card(90, 150, 700, 262, a, "RUN IT", C_TP)
        prompt, cmd = 0x6b7a90, 0xe6ecf5
        self.mono_rows(124, 236, [[("> ", prompt), ("pixi install -e jazzy", cmd)],
                                  [("> ", prompt), ("pixi run -e jazzy build", cmd)],
                                  [("> ", prompt), ("pixi run -e jazzy demo", cmd)]], 30, 58, a, t, c0("u1") + 0.5, 0.75)
        na = a * smooth(remap(t, c0("u1") + 2.6, c0("u1") + 3.2))
        for j, line in enumerate(("Windows, with pixi and the Visual Studio Build Tools.",
                                  "ROS 2 Jazzy comes from RoboStack.",
                                  "threepp comes from PyPI (Python) and GitHub (C++).",
                                  "Two windows open: the world as it is,",
                                  "and what the robot believes.")):
            ov.text(96, 470 + 40 * j + (16 if j >= 3 else 0), line, size=24, color=TEXT if j >= 3 else DIM, alpha=na,
                    anchor="lm")
        # the two windows, as the demo opens them
        wa = a * smooth(remap(t, s + 0.5, s + 1.1))
        wb = a * smooth(remap(t, s + 0.9, s + 1.5))
        self.pane("u_sim", self.stills["sim"], (1010, 122, 660, 371), wa, "EXPLORER_SIM", "the world as it is", C_CPP)
        self.pane("u_dash", self.stills["dash"], (1010, 548, 660, 371), wb, "DASHBOARD", "what the robot believes", C_PY)

    # how it was made ----------------------------------------------------------------------
    def made(self, t):
        s, e, a3 = TL.start("made"), TL.end("made"), c0("a3")
        if not s - 0.1 <= t <= e + 0.4:
            return
        ov = self.ov
        a = envelope(t, s, a3 + 0.1, 0.5, 0.5)
        if a > 0.003:
            ov.text(130, 150, "HOW IT WAS MADE", size=22, color=C_TP, alpha=a, kind="semibold", tracking=4)
            for j, line in enumerate(("Two to three hours.", "One AI coding agent. One person steering.")):
                ov.text(126, 226 + 78 * j, line, size=62, color=TEXT, alpha=a * smooth(remap(t, s + 0.3 + 0.5 * j, s + 0.9 + 0.5 * j)),
                        kind="semibold", anchor="lm")
            ov.text(130, 384, "Claude Code wrote the code and the documentation, and afterwards this film. A person set "
                    "the direction and judged the result.", size=25, color=DIM,
                    alpha=a * smooth(remap(t, s + 1.6, s + 2.2)), anchor="lm")
            rows = (("The contract first", "docs/interfaces.md: every topic, type and frame, before any node"),
                    ("Sub-agents in parallel", "navigation, vision and the dashboard, each against that contract"),
                    ("Checked without a window", "headless renders through a ROS service, 9 full runs, 68 unit tests"),
                    ("A review agent", "read the code and the documentation: 26 findings, fixed"),
                    ("A fresh clone", "built and run from nothing, as a student would"))
            t0 = c0("a2") - 0.2
            for j, (head, body) in enumerate(rows):
                al = a * smooth(remap(t, t0 + 0.75 * j, t0 + 0.75 * j + 0.5))
                y = 500 + 68 * j
                ov.circle(142, y, 7, fill=C_TP, alpha=al)
                ov.text(170, y, head, size=27, color=TEXT, alpha=al, kind="semibold", anchor="lm")
                ov.text(540, y, body, size=24, color=DIM, alpha=al, anchor="lm")
        # before and after: the first version, then one line of feedback
        b = envelope(t, a3 - 0.2, e + 0.4, 0.5, 0.4)
        if b <= 0.003:
            return
        tw = self.said("a3", "later", 0.72)
        u = smooth(remap(t, tw - 0.5, tw + 0.9))               # the wipe, left to right
        first, after = self.stills["first"], self.stills["after"]
        x = int(round(u * first.shape[1]))
        if x <= 0 or x >= first.shape[1]:
            img = first if x <= 0 else after
        else:                                                  # left of the line: the present
            img = first.copy()
            img[:, :x] = after[:, :x]
        self.show("before_after", img, (0, 0, 1920, 1080), b)
        if 0.0 < u < 1.0:
            ov.line([(u * 1920, 0), (u * 1920, 1080)], 0xffffff, 3.0, 0.9 * b)
        self.tag(1850, 64, "THE FIRST VERSION", "boxes for everything, and it already explored and mapped", b * (1 - u),
                 color=TEXT, anchor="r")
        qa = b * envelope(t, self.said("a3", "looked", 0.3) - 0.2, tw + 0.2, 0.4, 0.4)
        if qa > 0.003:
            ov.panel(360, 300, 1200, 250, radius=22, fill=0x0d131e, alpha=0.9 * qa)
            ov.text(410, 342, "THE FEEDBACK, IN FULL", size=18, color=C_CONE, alpha=qa, kind="semibold", tracking=3)
            ov.text(408, 420, "“You can do better.", size=58, color=TEXT, alpha=qa, kind="semibold", anchor="lm")
            ov.text(408, 492, "It looks like a 90s game.”", size=58, color=TEXT, alpha=qa, kind="semibold", anchor="lm")
        self.tag(1850, 64, "THE SAME BUILDING, LATER THAT AFTERNOON",
                 "an HDRI, lamps, Blender models, ambient occlusion and bloom, drawn by threepp",
                 b * smooth(remap(u, 0.75, 1.0)), color=C_TP, anchor="r")

    def end(self, t):
        s = TL.start("end")
        if t < s:
            return
        ov = self.ov
        ti = c0("e1") + 0.2
        a = smooth(remap(t, ti, ti + 0.6))
        if a <= 0.003:
            return
        ov.panel(-20, 560, 1960, 600, radius=0, fill=INK, alpha=0.8 * a)
        rise = 18 * (1 - ease_out(remap(t, ti, ti + 0.7)))
        ov.text(130, 640 + rise, "ROS 2 carries the messages.", size=60, color=TEXT, alpha=a, kind="semibold", anchor="lm")
        ov.rich(130, 722 + rise, [("threepp", C_TP, "semibold"), (" is the world, the sensors and the screen.", TEXT, "semibold")],
                size=60, alpha=a * smooth(remap(t, ti + 0.9, ti + 1.5)), anchor="lm")
        la = a * smooth(remap(t, ti + 2.2, ti + 2.9))
        ov.text(132, 826, "github.com/markaren/threepp", size=34, color=C_TP, alpha=la, anchor="lm")
        ov.text(132, 880, "pip install threepp", size=28, color=TEXT, alpha=la, kind="mono", anchor="lm")
        ov.text(132, 958, "Models and textures: Blender and Poly Haven (CC0).  Voice: Kokoro.  "
                "Text, graph and edit: threepp.lesson.", size=21, color=DIM, alpha=la, anchor="lm")


def word_clock(outdir, voiced):
    """said(key, word, fallback): the film time at which a word of a line is heard, from
    Kokoro's own alignment; without a voice, `fallback` of the way through the line."""
    narr = Narration(SPEECH.voice, cache_dir=os.path.join(outdir, "voice_cache"), words=SPEECH.words) if voiced else None
    cache = {}

    def said(key, word, fallback):
        if narr is not None:
            try:
                if key not in cache:
                    cache[key] = narr.word_times(CAPTION_TEXT[key])
                for w, a, _ in cache[key]:
                    if w.lower().startswith(word.lower()):
                        return c0(key) + VOICE_LEAD + a
                print(f"[words] {word!r} is not among the words of line {key}")
            except RuntimeError as e:
                print(f"[words] no alignment for line {key} ({e})")
        return c0(key) + VOICE_LEAD + fallback * SPEECH.seconds(key)
    return said


def _option(name, default):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


def setup(width, height):
    t_start = time.time()
    sx, sy = width / 1920, height / 1080

    def px(w, h):                                             # a design size, in rendered pixels (even)
        return (max(int(round(w * sx / 2)) * 2, 2), max(int(round(h * sy / 2)) * 2, 2))

    take = Take(px(384, 216))

    videos = {
        "chase": Video(os.path.join(REPLAY, "sim_chase.mp4"), 30, (width, height)),
        "over": Video(os.path.join(REPLAY, "sim_over.mp4"), 30, (width, height)),
        "eye": Video(os.path.join(REPLAY, "sim_eye.mp4"), 30, px(720, 405)),
        "chase_s": Video(os.path.join(REPLAY, "sim_chase.mp4"), 30, px(298, 168)),
        "dash_s": Video(os.path.join(TAKE, "dash.mp4"), 20, px(308, 173)),
        # the building, without the empty floor around it; the map, without the dashboard's cards
        "over_b": Video(os.path.join(REPLAY, "sim_over.mp4"), 30, px(924, 520), crop=(1600, 900, 0, 60)),
        "dash_b": Video(os.path.join(TAKE, "dash.mp4"), 20, px(924, 520), crop=(1320, 742, 545, 175)),
    }
    after = Video(os.path.join(REPLAY, "sim_over.mp4"), 30, (width, height))
    stills = {
        "sim": read_image(os.path.join(REPO, "docs", "images", "sim_chase.png"), px(660, 371)),
        "dash": read_image(os.path.join(REPO, "docs", "images", "dashboard.png"), px(660, 371)),
        "first": read_image(os.path.join(FOOTAGE, "first_version.png"), (width, height)),
        "after": after.at(WIPE_AT).copy(),
    }
    after.close()
    st = Stage(width, height, floor=False, fog=None, design=(1920, 1080))
    ov = Hud(1920, 1080)
    said = word_clock(_option("--outdir", "lesson_out"), "--no-voice" not in sys.argv)
    painter = Painter(ov, take, videos, stills, said)
    print(f"[setup] {time.time() - t_start:.1f}s, film {TL.duration:.1f} s on the script clock: "
          + ", ".join(f"{n} {TL.start(n):.1f}" for n, *_ in PLAN))

    def render(t):
        ov.begin()
        painter.draw(t)
        ov.fade(fade_in_out(t, TL.duration, fade_in=0.3, fade_out=0.8))
        ov.end()
        return st.frame(t, hud=ov)
    return render, painter.captions


if __name__ == "__main__":
    run("explorer_film", TL.duration, setup, fps=FPS, speech=SPEECH)
