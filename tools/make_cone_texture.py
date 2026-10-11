"""Paints the texture of the traffic cones: src/explorer_sim/assets/cone.png.

    pixi run -e jazzy python tools/make_cone_texture.py

A cone's texture is unrolled like a label on a bottle: left to right goes once around
the cone, the top row of the picture is the tip and the bottom row is the base.

What is painted, and why each part is the way it is:

  * the plastic: orange, RGB (255, 106, 0), with slow changes in brightness so it does
    not look like a flat fill. Only the brightness changes. The cone detector
    (explorer_vision) finds cones by their HUE, so the hue must stay put.
  * one white reflective band, from 46 % to 62 % of the height. Real cones have one, and
    it is what makes the detector's job interesting: the band cuts the orange in two.
  * scuffs near the base, where a cone gets kicked and dragged: darker streaks (still
    orange, only darker) and a few thin grey scratches.

The numbers at the top are the contract with the detector. Change the band, and
explorer_vision/cone_detector.py has to agree.
"""
import os

import cv2
import numpy as np

SIZE = 512
ORANGE = np.array([255, 106, 0], np.float32)   # RGB
WHITE = np.array([238, 240, 240], np.float32)
BAND = (0.46, 0.62)                            # of the height, measured from the base

rng = np.random.default_rng(7)                 # fixed seed: the same picture every time


def smooth_noise(cells: int) -> np.ndarray:
    """Random values that change slowly across the picture, between -1 and 1.

    A coarse grid of random numbers, blown up with smooth interpolation. It wraps
    around left to right, so no seam shows where the picture's two edges meet on the cone.
    """
    coarse = rng.uniform(-1, 1, (cells, cells)).astype(np.float32)
    coarse = np.concatenate([coarse, coarse[:, :1]], axis=1)          # last column = first
    return cv2.resize(coarse, (SIZE + SIZE // cells, SIZE), interpolation=cv2.INTER_CUBIC)[:, :SIZE]


height_from_base = 1.0 - (np.arange(SIZE, dtype=np.float32) + 0.5) / SIZE    # per row
rows = height_from_base[:, None]

# ---- the plastic: orange, brightness varied by a few percent
brightness = 1.0 + 0.05 * smooth_noise(6) + 0.025 * smooth_noise(24)
brightness -= 0.10 * np.clip((0.10 - rows) / 0.10, 0, 1)        # grimy towards the base
image = ORANGE[None, None, :] * brightness[:, :, None]

# ---- scuffs: short darker streaks in the lower third
for _ in range(40):
    x = int(rng.integers(0, SIZE))
    y = int(SIZE * (1.0 - rng.uniform(0.02, 0.34)))
    length = int(rng.integers(8, 46))
    streak = np.zeros((SIZE, SIZE), np.float32)
    cv2.line(streak, (x, y), (x + length, y + int(rng.integers(-3, 4))), 1.0, int(rng.integers(1, 4)), cv2.LINE_AA)
    streak = cv2.GaussianBlur(streak, (0, 0), 1.2)
    image *= (1.0 - 0.22 * streak)[:, :, None]

# ---- the reflective band, with the fine pattern of reflective sheeting
in_band = (rows >= BAND[0]) & (rows <= BAND[1])
xx, yy = np.meshgrid(np.arange(SIZE), np.arange(SIZE))
weave = 1.0 - 0.035 * (((xx + yy) // 4) % 2)                     # faint diagonal weave
band = WHITE[None, None, :] * (weave * (1.0 + 0.015 * smooth_noise(12)))[:, :, None]
image = np.where(in_band[:, :, None], band, image)
# a thin darker seam where the sheeting ends, top and bottom
for edge in BAND:
    seam = np.abs(rows - edge) < 0.004
    image = np.where(seam[:, :, None], image * 0.86, image)

# ---- a few thin grey scratches, anywhere below the band
for _ in range(14):
    x = int(rng.integers(0, SIZE))
    y = int(SIZE * (1.0 - rng.uniform(0.03, 0.42)))
    cv2.line(image, (x, y), (x + int(rng.integers(6, 30)), y + int(rng.integers(-8, 9))), (150, 140, 132), 1, cv2.LINE_AA)

image = np.clip(image, 0, 255).astype(np.uint8)
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "explorer_sim", "assets", "cone.png")
cv2.imwrite(os.path.normpath(out), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

# ---- a check, printed: how much of the orange is inside the detector's colour band?
hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
orange_rows = ~in_band[:, 0]
inside = (hsv[..., 0] >= 3) & (hsv[..., 0] <= 24) & (hsv[..., 1] >= 110) & (hsv[..., 2] >= 40)
print("written:", os.path.normpath(out))
print("orange part inside the detector's colour band: %.1f %%" % (100.0 * inside[orange_rows].mean()))
print("white band inside it: %.1f %%" % (100.0 * inside[~orange_rows].mean()))
