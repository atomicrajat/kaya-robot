"""Drive a fixed small move, then stop: python3 nudge.py vx vy w seconds."""
import os
import sys

# One-off probes live one level below the node. Make the node's modules (ball_policy_node,
# depth_scan, goal_marker, yolo_trt) and kaya_lab/ball_detector importable without copying.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(_HERE, os.pardir),
                os.path.join(_HERE, os.pardir, os.pardir, "kaya_lab")]
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

vx, vy, w, sec = (float(v) for v in sys.argv[1:5])
rclpy.init()
n = Node("nudge")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
t = Twist()
t.linear.x, t.linear.y, t.angular.z = vx, vy, w
t0 = time.time()
while time.time() - t0 < sec:
    pub.publish(t)
    time.sleep(0.05)
for _ in range(10):
    pub.publish(Twist())
    time.sleep(0.05)
print("nudged and stopped")
