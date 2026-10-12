"""Depth probe: one depth frame -> how much of it is valid, row by row, and the 16-ray
scan built from it. Never moves the robot."""
import os
import sys

# One-off probes live one level below the node. Make the node's modules (ball_policy_node,
# depth_scan, goal_marker, yolo_trt) and kaya_lab/ball_detector importable without copying.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(_HERE, os.pardir),
                os.path.join(_HERE, os.pardir, os.pardir, "kaya_lab")]
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from ball_policy_node import tf_matrix
from depth_scan import DepthScan

rclpy.init()
n = Node("probe_depth")
got = {}
q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Image, "/camera/depth/image_raw", lambda m: got.__setitem__("d", m), q)
n.create_subscription(CameraInfo, "/camera/depth/camera_info", lambda m: got.setdefault("k", m), 10)
tf = tf2_ros.Buffer()
tf2_ros.TransformListener(tf, n)
t0 = time.time()
while time.time() - t0 < 10 and not ("d" in got and "k" in got
                                     and tf.can_transform("base_link", "camera_depth_optical_frame", Time())):
    rclpy.spin_once(n, timeout_sec=0.2)
m = got["d"]
d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(float) / 1000
h, w = d.shape
c = d[h // 4:3 * h // 4, w // 3:2 * w // 3]
med = np.median(c[c > 0]) if (c > 0).any() else float("nan")
print(f"depth {w}x{h}: valid {100 * (d > 0).mean():.0f}% of image, centre valid {100 * (c > 0).mean():.0f}%, "
      f"centre median {med:.3f} m")
for row in (80, 160, 200, 240, 280, 320, 400):
    r = d[row, w // 3:2 * w // 3]
    v = r[r > 0]
    print(f"  row {row:3d}: valid {100 * len(v) / len(r):3.0f}%  median {np.median(v) if len(v) else float('nan'):.3f} m")
Td = tf_matrix(tf.lookup_transform("base_link", "camera_depth_optical_frame", Time()))
print("scan (m, left to right):", np.round(DepthScan(np.array(got["k"].k).reshape(3, 3), Td).scan(d), 2))
