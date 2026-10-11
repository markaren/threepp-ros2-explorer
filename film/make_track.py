"""The robot's path through a recorded run, as one pose per film frame.

    python make_track.py <take dir> [seconds]

Reads <take>/sim_frames.csv (the robot's true pose at every frame the simulator recorded, with
the wall-clock time of that frame), fits a smoothing spline through x, y and the unwrapped yaw,
and writes <take>/track.txt: "x y yaw linear angular", 30 lines per second, from the first
recorded frame on. <take>/track.json holds the wall-clock time of line 0.

The spline is there for the stamps, not the poses: a frame is stamped when it has been drawn,
some tens of milliseconds after the physics step it shows, and that delay varies. Left in, it
would shake a camera that rides behind the robot.
"""
import csv
import json
import os
import sys

import numpy as np
from scipy.interpolate import UnivariateSpline

take = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else None
FPS = 30

rows = []
with open(os.path.join(take, "sim_frames.csv")) as f:
    for r in csv.DictReader(f):
        # One stream only: the overview's frames are stamped a whole extra render later than
        # the chase camera's, and mixing the two puts a 40 ms sawtooth on the time axis.
        if r["stream"] == "chase":
            rows.append((float(r["unix_time"]), float(r["x"]), float(r["y"]), float(r["yaw"])))
rows.sort()
t = np.array([r[0] for r in rows])
keep = np.r_[True, np.diff(t) > 1e-4]
t, x, y, yaw = (np.array([r[k] for r in rows])[keep] for k in range(4))
yaw = np.unwrap(yaw)
t0 = t[0]
ts = t - t0
print(f"{len(ts)} poses over {ts[-1]:.1f} s, {len(ts) / ts[-1]:.1f} a second")

# s = N * sigma^2: how far the curve may stay from the samples, per sample
SIGMA_XY, SIGMA_YAW = float(os.environ.get("TRACK_SIGMA_XY", 0.002)), float(os.environ.get("TRACK_SIGMA_YAW", 0.004))
sx = UnivariateSpline(ts, x, k=5, s=len(ts) * SIGMA_XY ** 2)
sy = UnivariateSpline(ts, y, k=5, s=len(ts) * SIGMA_XY ** 2)
syaw = UnivariateSpline(ts, yaw, k=5, s=len(ts) * SIGMA_YAW ** 2)

end = ts[-1] if seconds is None else min(seconds, ts[-1])
tf = np.arange(0.0, end, 1.0 / FPS)
X, Y, YAW = sx(tf), sy(tf), syaw(tf)
VX, VY, W = sx.derivative()(tf), sy.derivative()(tf), syaw.derivative()(tf)
V = VX * np.cos(YAW) + VY * np.sin(YAW)
print(f"residuals: x {np.std(sx(ts) - x) * 1000:.1f} mm (max {np.abs(sx(ts) - x).max() * 1000:.1f}), "
      f"y {np.std(sy(ts) - y) * 1000:.1f} mm (max {np.abs(sy(ts) - y).max() * 1000:.1f}), "
      f"yaw {np.degrees(np.std(syaw(ts) - yaw)):.2f} deg (max {np.degrees(np.abs(syaw(ts) - yaw).max()):.2f})")
print(f"speed up to {np.abs(V).max():.2f} m/s at {tf[np.abs(V).argmax()]:.1f} s, "
      f"turn rate up to {np.abs(W).max():.2f} rad/s at {tf[np.abs(W).argmax()]:.1f} s; "
      f"99th percentile {np.percentile(np.abs(V), 99):.2f} m/s, {np.percentile(np.abs(W), 99):.2f} rad/s")
# What a camera 2.2 m behind the robot, looking 1 m ahead of it, would show of any shake:
# the look point's second difference, in pixels of a 1080p frame at the robot's distance.
look = np.c_[X + np.cos(YAW), Y + np.sin(YAW)]
jerk = np.linalg.norm(np.diff(look, 2, axis=0), axis=1) * 526.0
print(f"look point, second difference: median {np.median(jerk):.2f} px, 99th percentile {np.percentile(jerk, 99):.2f} px, "
      f"max {jerk.max():.2f} px at {tf[jerk.argmax()]:.1f} s")
dt_raw = np.diff(ts)
print(f"sample spacing: median {np.median(dt_raw) * 1000:.0f} ms, 95th percentile {np.percentile(dt_raw, 95) * 1000:.0f} ms, max {dt_raw.max() * 1000:.0f} ms")

with open(os.path.join(take, "track.txt"), "w") as f:
    for row in zip(X, Y, np.arctan2(np.sin(YAW), np.cos(YAW)), V, W):
        f.write(" ".join(f"{v:.6f}" for v in row) + "\n")
with open(os.path.join(take, "track.json"), "w") as f:
    json.dump({"t0": float(t0), "fps": FPS, "frames": int(len(tf))}, f)
print(f"wrote {len(tf)} lines ({len(tf) / FPS:.1f} s) to track.txt, t0 = {t0:.3f}")
