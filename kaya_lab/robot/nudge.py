"""Drive a fixed small move, then stop: python3 nudge.py vx vy w seconds."""
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
