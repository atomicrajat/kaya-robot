#!/usr/bin/env python3
"""B5: the trained ball policy on the real Kaya.

    colour image --> ball_detector (HSV, blob, sphere fit) --> ball in base_link
                                         |  (TF at the image's stamp)
    cuVSLAM odometry -------------------> memory: last seen ball in odom + its age
    start mark + measured box position -> goal point and direction
                                         v
                 the SAME 15 observations as training (ball_goal_det_env.py)
                                         v
              numpy policy (policy.npz) -> twist -> /cmd_vel -> drive_node

Run (with the B5 stack up: camera, cuVSLAM, drive, robot description):

    python3 ball_policy_node.py --ros-args -p policy:=policy.npz \
        -p goal_x:=2.0 -p goal_y:=0.0 -p goal_yaw_deg:=0.0 -p dry_run:=true

The robot must start ON THE MARK: goal_x / goal_y are the box mouth's centre
measured from the robot's centre at start (x ahead, y left, metres), and
goal_yaw_deg the direction INTO the box (0 = the box faces the robot straight
on). dry_run:=true (the default) computes and logs everything but never moves.

Stops (zero twist, node exits) on: scored, max_time, robot leaving the arena,
Ctrl-C. drive_node also stops the wheels by itself if /cmd_vel goes quiet.
"""

import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from ball_detector import BallDetector
from numpy_policy import NumpyPolicy

MAX_LIN, MAX_ANG = 0.15, 1.0          # drive node limits; the policy was trained on these
AGE_CAP_S = 5.0                       # as in training
BASE_LINK_Z = 0.037                   # base_link above the floor (wheel radius)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def tf_matrix(t):
    q, p = t.transform.rotation, t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class BallPolicyNode(Node):
    def __init__(self):
        super().__init__("kaya_ball_policy")
        P = self.declare_parameter
        self.policy = NumpyPolicy(P("policy", "policy.npz").value)
        self.goal_xy = np.array([P("goal_x", 2.0).value, P("goal_y", 0.0).value])
        self.goal_yaw = math.radians(P("goal_yaw_deg", 0.0).value)
        self.half_w = P("goal_width", 0.46).value / 2
        self.r = P("ball_radius", 0.070).value
        self.dry_run = P("dry_run", True).value
        self.max_time = P("max_time", 90.0).value
        self.arena = (P("arena_back", 1.0).value, P("arena_side", 2.0).value)
        self.image_topic = P("image_topic", "/camera/color/image_raw").value
        # phantom rejection (first live run, 2026-09-28: yellow/orange things 4-5 m to the
        # side were taken for the ball for ~15 s). A detection must be round (sphere-fit
        # residual) and inside the play area. Run 2 set the threshold: the real ball close
        # up and moving fits at 1.8-3.1 deg (static frames 0.24-0.86), phantoms (a wooden
        # desk edge) at 12-17 deg. 1.5 deg rejected the real ball while it was being pushed.
        self.max_fit_deg = P("max_fit_deg", 5.0).value
        self.save_rejects = P("save_rejects", True).value
        self.n_reject = {"fit": 0, "outside": 0}

        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.image_topic, self.on_image, best_effort)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, 10)
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self.on_odom, best_effort)

        self.K = self.D = None
        self.detector = None
        self.latest_img = None
        self.twist_b = np.zeros(3)                    # vx, vy, w from odometry
        self.start = None                             # (x, y, yaw) in odom at start
        self.est_w = np.zeros(2)                      # remembered ball, odom frame
        self.age = AGE_CAP_S
        self.last_det_t = 0.0
        self.seen_now = False
        self.ever_seen = False
        self.action = np.zeros(3)
        self.t0 = None
        self.done = False
        self.n_det = 0
        self.last_fit = (0.0, 0)
        self.det_ms = []
        self.create_timer(0.1, self.step)             # 10 Hz, as trained
        self.get_logger().info(f"policy loaded; goal mouth at {self.goal_xy} m, into-box yaw "
                               f"{math.degrees(self.goal_yaw):.0f} deg, width {2*self.half_w:.2f} m; "
                               f"{'DRY RUN - will not move' if self.dry_run else 'LIVE - will drive'}")

    # --------------------------------------------------------------- inputs
    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d)

    def on_odom(self, msg):
        t = msg.twist.twist
        self.twist_b = np.array([t.linear.x, t.linear.y, t.angular.z])

    def on_image(self, msg):
        self.latest_img = msg

    def pose(self, stamp=None):
        """base_link in odom: (x, y, yaw), at stamp if given."""
        t = self.tf.lookup_transform("odom", "base_link", Time() if stamp is None else Time.from_msg(stamp),
                                     timeout=rclpy.duration.Duration(seconds=0.05))
        p = t.transform.translation
        return np.array([p.x, p.y]), yaw_of(t.transform.rotation)

    def ensure_ready(self):
        if self.detector is None and self.K is not None:
            try:
                T = tf_matrix(self.tf.lookup_transform("base_link", "camera_color_optical_frame", Time()))
            except tf2_ros.TransformException:
                return False
            self.detector = BallDetector(self.K, self.D, T, self.r, self.r - BASE_LINK_Z)
            self.get_logger().info(f"detector ready; colour lens in base_link at {np.round(T[:3, 3], 3)}")
        if self.start is None:
            try:
                p, yaw = self.pose()
            except tf2_ros.TransformException:
                return False
            self.start = (p, yaw)
            self.get_logger().info(f"start pose in odom {np.round(p, 3)}, yaw {math.degrees(yaw):.1f} deg "
                                   "(the robot must be ON THE MARK now)")
        return self.detector is not None and self.start is not None

    # ------------------------------------------------------------ the loop
    def detect(self):
        """Newest frame -> (new_frame, ball in odom or None)."""
        msg, self.latest_img = self.latest_img, None
        if msg is None:
            return False, None
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)[..., :3]
        if msg.encoding.lower().startswith("bgr"):
            img = img[..., ::-1]
        t = time.perf_counter()
        d = self.detector.detect(np.ascontiguousarray(img))
        self.det_ms.append((time.perf_counter() - t) * 1000)
        if not d.ok:
            return True, None
        if d.rms_deg > self.max_fit_deg:
            return True, self._reject("fit", img, d, None)
        try:
            p, yaw = self.pose(msg.header.stamp)            # where the robot was when the frame was taken
        except tf2_ros.TransformException:
            p, yaw = self.pose()
        ball_w = p + rot(yaw) @ np.array([d.x, d.y])
        sp, syaw = self.start
        b_s = rot(-syaw) @ (ball_w - sp)                   # in the start frame
        if b_s[0] < -self.arena[0] or b_s[0] > self.goal_xy[0] + 0.4 or abs(b_s[1]) > self.arena[1] + 0.5:
            return True, self._reject("outside", img, d, b_s)
        self.n_det += 1
        self.last_fit = (d.rms_deg, d.area_px)
        return True, ball_w

    def _reject(self, why, img, d, b_s):
        """Log a rejected detection and keep a snapshot of it: what fooled the detector?"""
        self.n_reject[why] += 1
        where = f" at start-frame ({b_s[0]:+.2f},{b_s[1]:+.2f})" if b_s is not None else ""
        self.get_logger().warn(f"rejected ({why}): fit {d.rms_deg:.2f} deg, area {d.area_px} px{where}")
        if self.save_rejects and self.n_reject[why] <= 20:
            import cv2
            m = self.detector.mask(img)
            vis = img[..., ::-1].copy()
            vis[m > 0] = (0.5 * vis[m > 0] + [127, 0, 127]).astype(np.uint8)
            cv2.imwrite(f"reject_{why}_{self.n_reject[why]:02d}.png", vis)
        return None

    def step(self):
        if self.done or not self.ensure_ready():
            return
        if self.t0 is None:
            self.t0 = time.time()
        # "seen" = the latest camera FRAME contained the ball; a step with no new frame
        # yet (the 70 ms detector and the 15 Hz camera do not line up with the 10 Hz
        # loop) keeps the last verdict. Counting it as "not seen" made seen/memory
        # alternate every step in the first dry run, a pattern training never showed.
        # age = real time since the last detection, as in training.
        new_frame, ball_w = self.detect()
        now = time.time()
        if ball_w is not None:
            self.est_w, self.last_det_t, self.seen_now, self.ever_seen = ball_w, now, True, True
        elif new_frame:
            self.seen_now = False
        self.age = min(now - self.last_det_t, AGE_CAP_S) if self.ever_seen else AGE_CAP_S

        p, yaw = self.pose()
        sp, syaw = self.start
        # goal (box mouth centre) and its into-box direction, in odom
        g_w = sp + rot(syaw) @ self.goal_xy
        gdir = syaw + self.goal_yaw
        # the ball in the goal's frame (x into the box, y across the mouth)
        b_g = rot(-gdir) @ (self.est_w - g_w)
        # the point to push towards: one radius inside the mouth, level with the ball
        # but within the posts (training: _goal_point_env)
        half = max(self.half_w - self.r, 0.0)
        gp_g = np.array([self.r, np.clip(b_g[1] if self.ever_seen else 0.0, -half, half)])
        gp_w = g_w + rot(gdir) @ gp_g

        to_b = rot(-yaw)
        est_b = to_b @ (self.est_w - p) if self.ever_seen else np.zeros(2)
        obs = np.concatenate([
            est_b / 2.0,
            [self.age / AGE_CAP_S],
            [1.0 if self.seen_now else 0.0],
            (to_b @ (gp_w - p)) / 3.0,
            [math.cos(gdir - yaw), math.sin(gdir - yaw)],
            [self.half_w / 0.23],
            self.twist_b[:2] / MAX_LIN,
            [self.twist_b[2] / MAX_ANG],
            self.action,
        ])
        self.action = np.clip(self.policy(obs), -1.0, 1.0)

        # stop conditions
        t = time.time() - self.t0
        p_s = rot(-syaw) @ (p - sp)                        # robot in the start frame
        scored = self.ever_seen and self.age < 1.0 and b_g[0] > self.r and abs(b_g[1]) < self.half_w
        left = p_s[0] < -self.arena[0] or abs(p_s[1]) > self.arena[1] or p_s[0] > self.goal_xy[0] + 0.5
        reason = "SCORED" if scored else "time limit" if t > self.max_time else "left the arena" if left else None

        twist = Twist()
        if reason is None and not self.dry_run:
            twist.linear.x = float(self.action[0] * MAX_LIN)
            twist.linear.y = float(self.action[1] * MAX_LIN)
            twist.angular.z = float(self.action[2] * MAX_ANG)
        if not self.dry_run or reason:
            self.cmd.publish(twist)
        if int(t * 10) % 5 == 0 or reason:
            ball_s = rot(-syaw) @ (self.est_w - sp)
            self.get_logger().info(
                f"t {t:5.1f}s robot ({p_s[0]:+.2f},{p_s[1]:+.2f}) | ball {'SEEN' if self.seen_now else f'mem {self.age:.1f}s' if self.ever_seen else 'never seen'}"
                f" at ({ball_s[0]:+.2f},{ball_s[1]:+.2f}) goal-frame ({b_g[0]:+.2f},{b_g[1]:+.2f}) | action "
                f"({self.action[0]:+.2f},{self.action[1]:+.2f},{self.action[2]:+.2f}) | det {np.mean(self.det_ms[-20:]) if self.det_ms else 0:.0f} ms"
                f" fit {self.last_fit[0]:.2f} deg")
        if reason:
            self.done = True
            self.get_logger().warn(f"STOP: {reason} after {t:.1f} s ({self.n_det} detections, rejected {self.n_reject})")
            raise SystemExit

    def stop(self):
        for _ in range(3):
            self.cmd.publish(Twist())
            time.sleep(0.05)


def main():
    rclpy.init()
    node = BallPolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not node.dry_run:
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
