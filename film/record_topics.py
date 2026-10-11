"""Record what the film needs from the ROS side of a run: the robot's camera (plain and with
the cone spotter's boxes), and a time series of everything worth cutting on.

    python record_topics.py <out_dir> <max_seconds>

Images are written as the JPEG files they already are. index.json holds, on the wall clock
(time.time()): every image, the exploration status, the robot's true pose, the detections per
image, the cones on the map, and the safety stops. Ends a few seconds after DONE, then writes
<out_dir>/stop, which the patched simulator and dashboard take as "close the videos".
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray
from explorer_interfaces.msg import ExplorationStatus

out_dir = sys.argv[1]
max_seconds = float(sys.argv[2])
for sub in ("camera", "annotated"):
    os.makedirs(os.path.join(out_dir, sub), exist_ok=True)

rclpy.init()
node = Node("film_recorder")
names = {0: "IDLE", 1: "EXPLORING", 2: "RETURNING", 3: "NAVIGATING", 4: "DONE"}
index = {"camera": [], "annotated": [], "status": [], "pose": [], "detections": [], "cones": [], "stops": []}
state = {"status": None, "cones": 0}


def on_image(kind):
    def callback(msg):
        k = len(index[kind])
        with open(os.path.join(out_dir, kind, f"{k:05d}.jpg"), "wb") as f:
            f.write(bytes(msg.data))
        index[kind].append(round(time.time(), 4))
    return callback


def on_status(msg):
    state["status"] = msg
    index["status"].append([round(time.time(), 3), int(msg.state), round(msg.explored_area, 2), round(msg.distance, 2),
                            int(msg.frontier_count), round(msg.elapsed, 2)])


def on_pose(msg):
    p = msg.pose
    yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
    index["pose"].append([round(time.time(), 4), round(p.position.x, 4), round(p.position.y, 4), round(yaw, 4)])


def on_detections(msg):
    boxes = [[round(d.bbox.center.position.x, 1), round(d.bbox.center.position.y, 1), round(d.bbox.size_x, 1),
              round(d.bbox.size_y, 1)] for d in msg.detections]
    index["detections"].append([round(time.time(), 4), boxes])


def on_cones(msg):
    cones = [[round(m.pose.position.x, 2), round(m.pose.position.y, 2)] for m in msg.markers if m.action == 0]
    if len(cones) != state["cones"]:
        state["cones"] = len(cones)
        index["cones"].append([round(time.time(), 3), cones])


latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(CompressedImage, "camera/image/compressed", on_image("camera"), qos_profile_sensor_data)
node.create_subscription(CompressedImage, "camera/annotated/compressed", on_image("annotated"), qos_profile_sensor_data)
node.create_subscription(ExplorationStatus, "exploration_status", on_status, 10)
node.create_subscription(PoseStamped, "ground_truth", on_pose, 50)
node.create_subscription(Detection2DArray, "detections", on_detections, 50)
node.create_subscription(MarkerArray, "cones", on_cones, latched)
node.create_subscription(Bool, "safety_stop", lambda m: index["stops"].append([round(time.time(), 3), bool(m.data)]), latched)

t0 = time.time()
next_print = t0
done_since = None
while time.time() - t0 < max_seconds:
    rclpy.spin_once(node, timeout_sec=0.02)
    now = time.time()
    s = state["status"]
    if now >= next_print:
        next_print = now + 10.0
        if s is None:
            print(f"[{now - t0:5.1f}s] no exploration_status yet", flush=True)
        else:
            print(f"[{now - t0:5.1f}s] {names.get(s.state, s.state):10s} area {s.explored_area:6.1f} m2  dist {s.distance:5.1f} m  "
                  f"frontiers {s.frontier_count}  cones {state['cones']}  images {len(index['camera'])}/{len(index['annotated'])}",
                  flush=True)
    if s is not None and s.state == 4 and now - t0 > 20:
        if done_since is None:
            done_since = now
            print(f"[{now - t0:5.1f}s] DONE: area {s.explored_area:.1f} m2, dist {s.distance:.1f} m, elapsed {s.elapsed:.1f} s", flush=True)
        elif now - done_since > 5.0:
            break

with open(os.path.join(out_dir, "index.json"), "w") as f:
    json.dump(index, f)
with open(os.path.join(out_dir, "stop"), "w") as f:
    f.write("stop\n")
print("cones:", index["cones"][-1][1] if index["cones"] else [], "| images:", len(index["camera"]), len(index["annotated"]), flush=True)
time.sleep(3.0)     # the simulator and the dashboard look for the stop file about once a second
node.destroy_node()
rclpy.shutdown()
