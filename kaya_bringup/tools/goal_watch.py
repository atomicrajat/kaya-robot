"""Watch for a Nav2 goal and measure where the robot actually ended up.

Reports the error against the goal, and Z separately -- on a flat floor Z
should read zero, so any departure is drift you can trust.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class W(Node):
    def __init__(self):
        super().__init__("goal_watch")
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.goal = None
        self.last = None
        self.still_since = None
        self.t_goal = None
        print("watching /goal_pose ...", flush=True)

    def pose(self):
        try:
            t = self.buf.lookup_transform("map", "base_footprint", Time()).transform
            q = t.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            return t.translation, yaw
        except Exception:
            return None, None

    def on_goal(self, m):
        p, yaw = self.pose()
        self.goal = m.pose.position
        self.t_goal = time.time()
        self.still_since = None
        if p:
            print("GOAL  (%.3f, %.3f)  from (%.3f, %.3f)  -> %.2f m away"
                  % (self.goal.x, self.goal.y, p.x, p.y,
                     math.hypot(self.goal.x - p.x, self.goal.y - p.y)), flush=True)

    def tick(self):
        p, yaw = self.pose()
        if p is None or self.goal is None:
            return
        if self.last is not None:
            moved = math.hypot(p.x - self.last[0], p.y - self.last[1])
            if moved < 0.005:
                if self.still_since is None:
                    self.still_since = time.time()
                elif (time.time() - self.still_since > 4.0
                      and time.time() - self.t_goal > 6.0):
                    err = math.hypot(self.goal.x - p.x, self.goal.y - p.y)
                    print("ARRIVED at (%.3f, %.3f, z=%+.3f) yaw %+.1f deg  |  "
                          "error %.3f m (%.1f cm)  |  took %.0f s"
                          % (p.x, p.y, p.z, math.degrees(yaw), err, err * 100,
                             time.time() - self.t_goal), flush=True)
                    self.goal = None
                    self.still_since = None
            else:
                self.still_since = None
        self.last = (p.x, p.y)


rclpy.init()
w = W()
t0 = time.time()
while time.time() - t0 < 900:
    rclpy.spin_once(w, timeout_sec=0.2)
    w.tick()
