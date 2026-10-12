"""Save the left IR image and a depth picture, to see why depth is empty. Never moves."""
import os
import sys

# One-off probes live one level below the node. Make the node's modules (ball_policy_node,
# depth_scan, goal_marker, yolo_trt) and kaya_lab/ball_detector importable without copying.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(_HERE, os.pardir),
                os.path.join(_HERE, os.pardir, os.pardir, "kaya_lab")]
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

rclpy.init()
n = Node("probe_ir")
got = {}
q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Image, "/camera/left_ir/image_raw", lambda m: got.__setitem__("ir", m), q)
n.create_subscription(Image, "/camera/depth/image_raw", lambda m: got.__setitem__("d", m), q)
t0 = time.time()
while time.time() - t0 < 10 and not ("ir" in got and "d" in got):
    rclpy.spin_once(n, timeout_sec=0.2)
ir = got["ir"]
a = np.frombuffer(ir.data, np.uint8)[:ir.height * ir.width].reshape(ir.height, ir.width)
print(f"IR {ir.encoding}: mean {a.mean():.0f}, p1 {np.percentile(a, 1):.0f}, p99 {np.percentile(a, 99):.0f}, "
      f"saturated {100 * (a >= 250).mean():.1f}%")
d = got["d"]
z = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(float)
vis = np.zeros((d.height, d.width, 3), np.uint8)
ok = z > 0
vis[ok] = cv2.applyColorMap(np.clip(255 - z[ok] / 2000 * 255, 0, 255).astype(np.uint8).reshape(-1, 1),
                            cv2.COLORMAP_JET).reshape(-1, 3)
cv2.imwrite("probe_ir.png", np.hstack([cv2.cvtColor(a, cv2.COLOR_GRAY2BGR), vis]))
print(f"depth valid {100 * ok.mean():.1f}%")
