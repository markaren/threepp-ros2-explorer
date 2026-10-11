# The film

`explorer_film.py` makes a two and a half minute film about this demo: what it is, how its six
nodes are connected, where threepp sits in a ROS 2 project, how to run it, and how it was made.

The film is itself a threepp program. Text, cards, the node graph and the video panes are a
`threepp.lesson.Hud` (the same 2D toolkit the dashboard's cards are built on), rendered by
threepp's OpenGL renderer, frame by frame, and piped to ffmpeg. The narration is Kokoro, an
offline text-to-speech model; the pictures are laid out from the measured length of every spoken
line (`explorer_film.speech.json`), so a cut can land on the word that names it.

```
python explorer_film.py --remeasure     say every line once, rewrite explorer_film.speech.json
python explorer_film.py --stills 5,40   single frames
python explorer_film.py --sheet         a contact sheet, one frame every 3 s
python explorer_film.py                 the film, 1920x1080 at 30 fps
```

It needs `threepp` (2026.10.11 or later), `numpy`, `opencv-python`, `imageio-ffmpeg`, and for
the voice Kokoro (`pip install "threepp[narration]"`; without it, `--no-voice`).

## Where the pictures come from

Every moving picture is from **one real run** of the demo: 192 seconds, the whole building
mapped, six of six cones found, the robot home again. The footage is not in the repository
(1.7 GB). `EXPLORER_FOOTAGE` points the script at it; the default is `footage/` next to this file.

| In the film | What it is |
|---|---|
| The dashboard's map, growing | The dashboard's own window, captured while the run was live |
| The robot's camera with green boxes | The `camera/annotated/compressed` topic of that run: the JPEG images the cone spotter published, unchanged |
| The simulator from behind the robot, from above, and through the robot's eye | Drawn **after** the run, by the same simulator, from the robot's recorded poses |
| The two windows in "Run it" | `docs/images/sim_chase.png` and `docs/images/dashboard.png` |
| "The first version" | A screenshot of the simulator from the first hour of the work |

Why the simulator's pictures were drawn afterwards: capturing two 1920x1080 views while the
run was live slowed the simulator from 60 frames a second to about 19, and the film from behind
the robot came out at 9 frames a second. So the live run recorded the robot's true pose with
every frame, and a second pass put the robot back at each of those poses, 30 times per second of
the run, and drew three views of it with no clock involved. The robot, the building, the
lighting and the chase camera's own smoothing are the simulator's; the lidar's dots are cast
again from each pose, with fresh noise. One thing was changed for the eye: in the view from
above the lidar's beams are drawn at four times their usual strength, so the fan of rays can be
seen from across the building.

The run itself was not at its best either: with the capture running, the lidar and the camera
delivered about 6 scans and images a second instead of 10. The robot explored the building all
the same.

The node graph in the film is the one in [`docs/interfaces.md`](../docs/interfaces.md), thinned
out to be read in forty seconds: tf, the odometry and four of the dashboard's subscriptions
(the scan, both camera topics, the safety flag) are left out.

## How it was recorded

Neither the capture nor the second pass is part of the demo. They are a patch,
`record.patch`, meant for a throwaway clone:

```
git clone https://github.com/markaren/threepp-ros2-explorer.git clone
cd clone
git apply film/record.patch
pixi run -e jazzy build
```

With `EXPLORER_RECORD_DIR` set, the patched simulator and dashboard pipe clean frames to ffmpeg
and write the wall-clock time of each. `record_topics.py` saves the camera topics and a time
series of the run (status, poses, detections, cones). `make_track.py` turns the recorded poses
into one pose per film frame, through a smoothing spline: a frame is stamped after it has been
drawn, the delay varies by some milliseconds, and left in, that would shake a camera riding
behind the robot. With `EXPLORER_REPLAY` set to that track, the patched simulator does not
simulate: it draws.

## What is said about how the demo was made

The last part of the film says the demo was built in two to three hours by an AI coding agent
(Claude Code) with a person steering, and shows the first version next to the present one. The
numbers on that card (9 full runs, 68 unit tests, 26 review findings) are counts from that
working session; the run in the film is a tenth. The quoted feedback is the message as it was
typed. The film was made afterwards, the same way.
