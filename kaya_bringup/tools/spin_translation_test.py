"""Command a pure in-place rotation and measure how far the robot translates.

An omni base should rotate about its own centre. If commanded rotation leaks
into translation, the wheel geometry in kaya_drive is wrong and no amount of
Nav2 tuning will fix the robot wandering when it corrects heading at a goal.

Measured in the ODOM frame: this is raw VIO, no map and no relocalisation, so
the number is about the robot and not about localisation.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

WZ = 0.5          # rad/s, the same cap Nav2 now uses
TURN = 2 * math.pi
DUR = TURN / WZ


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rp_of(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    return math.degrees(roll), math.degrees(math.asin(sinp))


rclpy.init()
n = Node("spin_test")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
buf = tf2_ros.Buffer()
lis = tf2_ros.TransformListener(buf, n)


def pose():
    t = buf.lookup_transform("odom", "base_footprint", Time()).transform
    r, pi = rp_of(t.rotation)
    return t.translation.x, t.translation.y, yaw_of(t.rotation), t.translation.z, r, pi


t0 = time.time()
while time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.2)
    try:
        start = pose()
        break
    except Exception:
        start = None
if start is None:
    sys.exit("no odom->base_footprint")

print("start  x %+.3f  y %+.3f  yaw %+.1f deg" % (start[0], start[1], math.degrees(start[2])), flush=True)
print("commanding wz = %.2f rad/s for %.1f s (one full turn), vx = vy = 0" % (WZ, DUR), flush=True)

msg = Twist()
msg.angular.z = WZ
t0 = time.time()
worst = 0.0
worst_r = worst_p = worst_z = 0.0
while time.time() - t0 < DUR:
    pub.publish(msg)
    rclpy.spin_once(n, timeout_sec=0.02)
    try:
        p = pose()
        d = math.hypot(p[0] - start[0], p[1] - start[1])
        worst = max(worst, d)
        worst_r = max(worst_r, abs(p[4])); worst_p = max(worst_p, abs(p[5]))
        worst_z = max(worst_z, abs(p[3]))
    except Exception:
        pass
    time.sleep(0.05)

pub.publish(Twist())          # stop
for _ in range(10):
    pub.publish(Twist())
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.05)

t0 = time.time()
while time.time() - t0 < 3:
    rclpy.spin_once(n, timeout_sec=0.2)
    time.sleep(0.1)
end = pose()
drift = math.hypot(end[0] - start[0], end[1] - start[1])
turned = math.degrees((end[2] - start[2] + math.pi) % (2 * math.pi) - math.pi)
print("end    x %+.3f  y %+.3f  yaw %+.1f deg" % (end[0], end[1], math.degrees(end[2])), flush=True)
print("", flush=True)
print("  net translation after one full turn : %.3f m  (%.1f cm)" % (drift, drift * 100), flush=True)
print("  worst excursion during the turn     : %.3f m  (%.1f cm)" % (worst, worst * 100), flush=True)
print("  heading closed to                   : %+.1f deg from start" % turned, flush=True)
print("", flush=True)
print("  WORST |roll| during the turn        : %.1f deg" % worst_r, flush=True)
print("  WORST |pitch| during the turn       : %.1f deg" % worst_p, flush=True)
print("  WORST |z| during the turn           : %.3f m" % worst_z, flush=True)
print("  final roll / pitch                  : %+.1f / %+.1f deg" % (end[4], end[5]), flush=True)
