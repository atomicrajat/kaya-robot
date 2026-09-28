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
from sensor_msgs.msg import CameraInfo, Image, Imu

from ball_detector import BallDetector
from numpy_policy import NumpyPolicy

MAX_LIN, MAX_ANG = 0.15, 1.0          # drive node limits; the policy was trained on these
AGE_CAP_S = 5.0                       # as in training
STALL_MAX_TURN = 0.5                  # [rad/s] impacts are not trusted while turning faster
ROBOT_IN_BOX_M = 0.12                 # robot centre this far past the mouth line = in the box
COLOUR_TRUST_S = 5.0                  # a colour-only ball detection needs YOLO within this long
BOX_FRESH_S = 10.0                    # a goal only counts within this long of a marker sighting
FUSE_JUMP_M = 0.05                    # per 0.1 s step: 0.5 m/s of disagreement = a visual jump
FUSE_JUMP_RAD = math.radians(8.0)
# Stall detection (stall_probe measurements, 2026-09-28; the wheels keep spinning at the
# commanded speed when blocked, so the encoders cannot see a stall):
#   wall hit          IMU -1.27 m/s^2 spike, loads +10%
#   ball, open floor  no spike (within +-0.3 noise)
#   ball pinned       no clear spike, loads +5% -> caught by the depth scan instead
STALL_IMPACT = 0.6                    # [m/s^2] deceleration against the wheels' direction,
STALL_ACC_WIN_S = 0.1                 #   averaged over this window
STALL_LATCH_S = 3.0                   # after an impact: stalled while still pushing the same way, max 3 s
STALL_WIN_S = 0.5                     # soft stall: wheels moved >= 3 cm forward in this window,
STALL_WHEEL_M = 0.03                  #   yet the forward depth closed by < 1 cm
STALL_CLOSE_M = 0.01
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
        # the top speed the policy was TRAINED with (0.15, or 0.10 for the slow fine-tune);
        # scaling a policy down on the robot instead of retraining would break what it
        # learned about how its own commands move it
        global MAX_LIN
        MAX_LIN = P("max_lin_vel", MAX_LIN).value
        self.image_topic = P("image_topic", "/camera/color/image_raw").value
        # where the goal box is: "mark" = measured from the start mark (goal_x/goal_y/goal_yaw_deg),
        # "marker" = read off the ArUco markers on the box (goal_marker.py), carried by odometry
        # between sightings and falling back to the mark until the first sighting
        self.goal_source = P("goal_source", "mark").value
        self.marker_size = P("marker_size", 0.146).value           # measured print: 146 mm
        self.markers = None
        self.goal_w = None                                          # (xy, into-box yaw) in odom
        self.goal_seen_t = 0.0
        self.n_marker = 0
        self.last_marker_ids, self.marker_counts = [], {}
        self.goal_yaw_w = None                                      # trusted, from ID 0 only
        self.marker_rejected = 0
        self.reject_run = []                          # consecutive rejected box placements (world xy)
        self.marker_reanchored = 0
        # phantom rejection (first live run, 2026-09-28: yellow/orange things 4-5 m to the
        # side were taken for the ball for ~15 s). A detection must be round (sphere-fit
        # residual) and inside the play area. Run 2 set the threshold: the real ball close
        # up and moving fits at 1.8-3.1 deg (static frames 0.24-0.86), phantoms (a wooden
        # desk edge) at 12-17 deg. 1.5 deg rejected the real ball while it was being pushed.
        self.max_fit_deg = P("max_fit_deg", 5.0).value
        self.test_push = P("test_push", 0.0).value               # >0: drive straight at this speed (stall tests)
        # step 1 by YOLO11s on the GPU (TensorRT); "" = colour only
        engine = P("yolo_engine", "").value
        self.yolo = None
        if engine:
            from yolo_trt import YoloTRT
            self.yolo = YoloTRT(engine)
            self.get_logger().info(f"YOLO engine loaded: {engine}")
        self.save_rejects = P("save_rejects", True).value
        self.n_reject = {"fit": 0, "outside": 0, "unconfirmed": 0}
        self.yolo_t, self.yolo_w = None, None               # the last YOLO-backed ball sighting
        self.round_buf = []                                 # recent "round" sightings (t, xy in odom)

        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.image_topic, self.on_image, best_effort)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, 10)
        self.odom_source = P("odom_source", "wheel").value          # "wheel" or "vslam"
        self.odom_hist = []                                          # (stamp ns, xy, yaw), wheel odometry
        wheel = self.odom_source in ("wheel", "fused")
        odom_topic = "/odom" if wheel else "/visual_slam/tracking/odometry"
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10 if wheel else best_effort)
        self.fused, self.fused_hist, self.n_fuse = None, [], {"vslam": 0, "wheel": 0}
        # gyro yaw (the camera's IMU): turning is where the wheels drift most, and the gyro
        # measures it directly and never jumps. Bias from the first second standing still.
        self.use_gyro = self.odom_source == "fused" and P("gyro_yaw", True).value
        self.R_base_imu, self.gyro_yaw, self.gyro_t, self.gyro_bias, self.gyro_cal = None, 0.0, None, None, []
        self.acc_cal, self.acc_bias, self.acc_buf = [], np.zeros(2), []
        self.gyro_rate = 0.0
        self.stall_until, self.stall_reason, self.stall_dir = 0.0, "", np.array([1.0, 0.0])
        self.stall_hist = []                      # (t, wheel forward travel so far, forward range, yaw)
        self.wheel_fwd = 0.0
        self.n_stalled = 0
        if self.use_gyro:
            # every sample, in order: depth 1 kept only the newest while a 50-170 ms detection held
            # the executor, and the dropped rotation left run 32's heading ~65 deg off after its
            # search spin (the box then "moved" 1.8 m and the robot pushed the ball the wrong way)
            imu_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=400, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Imu, "/camera/gyro_accel/sample", self.on_imu, imu_qos)
        self.vio_z0, self.vio_dead = None, False
        # cuVSLAM loses tracking near plain walls and can then report "no motion" while still
        # alive (the pinned-ball stall test) - and with the IR projector on for depth, its
        # images carry moving dots. For the ball game: gyro + wheels + stall detection.
        self.use_vio = P("use_vio", True).value
        # v2 policies also get a 16-ray depth scan (depth_scan.py); needs the depth stream on
        self.use_scan = P("use_scan", False).value
        self.box_in_scan = P("box_in_scan", True).value     # fill the scan's box part from the box pose
        self.box_part = None
        self.scanner = None
        self.depth_K = None
        self.latest_depth = None
        self.last_scan = np.full(16, 3.0)
        if self.use_scan:
            self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, best_effort)
            self.create_subscription(CameraInfo, "/camera/depth/camera_info", self.on_depth_info, 10)

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
        self.last_src = ""
        self.prev_p = None
        self.seen_in_front = False                    # the ball was in front of the mouth, within the posts
        self.inside_n, self.inside_t0 = 0, None       # consecutive frames that SAW the ball inside the box
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
        if self.odom_source in ("wheel", "fused"):
            q, p = msg.pose.pose.orientation, msg.pose.pose.position
            self.odom_hist.append((Time.from_msg(msg.header.stamp).nanoseconds, np.array([p.x, p.y]), yaw_of(q)))
            if len(self.odom_hist) > 200:
                self.odom_hist.pop(0)

    def on_image(self, msg):
        self.latest_img = msg

    def on_depth_info(self, msg):
        if self.depth_K is None:
            self.depth_K = np.array(msg.k).reshape(3, 3)

    def on_depth(self, msg):
        self.latest_depth = msg

    def depth_scan(self):
        """The 16 distances (m) from the newest depth frame; the last scan if none is new."""
        msg, self.latest_depth = self.latest_depth, None
        if msg is None or self.scanner is None:
            return self.last_scan
        dtype = np.uint16 if "16" in msg.encoding else np.float32
        d = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width).astype(np.float64)
        if dtype == np.uint16:
            d = d / 1000.0                                         # 16UC1 is millimetres
        self.last_scan = self.scanner.scan(d)
        return self.last_scan

    def on_imu(self, msg):
        if self.R_base_imu is None:
            try:
                self.R_base_imu = tf_matrix(self.tf.lookup_transform("base_link", msg.header.frame_id, Time()))[:3, :3]
            except tf2_ros.TransformException:
                return
        w = self.R_base_imu @ np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        a = self.R_base_imu @ np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.gyro_bias is None:                      # still at the start: learn the biases
            self.acc_cal.append(a[:2])
            self.gyro_cal.append(w[2])
            if len(self.gyro_cal) >= 150:
                self.gyro_bias = float(np.mean(self.gyro_cal))
                self.acc_bias = np.mean(self.acc_cal, axis=0)
                self.get_logger().info(f"gyro bias {math.degrees(self.gyro_bias):+.3f} deg/s from {len(self.gyro_cal)} samples")
            self.gyro_t = t
            return
        if self.gyro_t is not None and 0 < t - self.gyro_t < 0.5:
            # trapezoid over the gap; a gap this long now means a lost burst, still better counted
            self.gyro_yaw += 0.5 * (w[2] - self.gyro_bias + self.gyro_rate) * (t - self.gyro_t)
            self.gyro_rate = w[2] - self.gyro_bias
        self.gyro_t = t
        # impact: horizontal deceleration against the direction the wheels are driving
        # average over ~0.1 s, the window the thresholds were measured on (stall_probe). A 3-sample
        # (~20 ms) window read omni-roller vibration on tile as 1-2 m/s^2 "impacts" and froze the
        # odometry for 552 of 589 steps of run 19.
        self.acc_buf.append((t, a[:2] - self.acc_bias))
        while self.acc_buf and t - self.acc_buf[0][0] > STALL_ACC_WIN_S:
            self.acc_buf.pop(0)
        v = self.twist_b[:2]
        if np.linalg.norm(v) > 0.03:
            if len(self.acc_buf) < 8:
                return
            decel = -float(np.mean([b[1] for b in self.acc_buf], axis=0) @ (v / np.linalg.norm(v)))
            # not while turning hard: the IMU sits off the robot's centre, so a spin in place reads
            # as a steady sideways acceleration (run 29: 592 of 808 steps "stalled" while it spun)
            if decel > STALL_IMPACT and abs(self.gyro_rate) < STALL_MAX_TURN:
                # latched while the robot keeps pushing the same way (the depth is blind against a
                # plain wall, so nothing else can see that it is still blocked; check 2 of the
                # stall tests: a 0.7 s hold let the spinning wheels creep 15 cm through the wall)
                self.stall_until = max(self.stall_until, time.time() + STALL_LATCH_S)
                self.stall_reason = f"impact {decel:.2f} m/s2"
                self.stall_dir = v / np.linalg.norm(v)

    def stalled(self):
        """Stalled until stall_until, unless the command has turned away from the blocked direction."""
        if time.time() >= self.stall_until:
            return False
        cmd = self.action[:2] * MAX_LIN
        if np.linalg.norm(cmd) < 0.02 or float(cmd @ self.stall_dir) / np.linalg.norm(cmd) < 0.7:
            self.stall_until = 0.0                       # driving away from the obstacle: free again
            return False
        return True

    def check_soft_stall(self):
        """Soft / pinned stall: the wheels moved the robot forward >= 3 cm in the last 0.5 s
        while barely turning, but the forward depth readings did not close in."""
        c = self.last_scan[6:10]
        rng = float(np.median(c)) if (c < 2.9).sum() >= 2 else None
        now = time.time()
        self.stall_hist.append((now, self.wheel_fwd, rng, self.gyro_yaw))
        while self.stall_hist and now - self.stall_hist[0][0] > STALL_WIN_S + 0.05:
            self.stall_hist.pop(0)
        t0, f0, r0, y0 = self.stall_hist[0]
        if now - t0 < STALL_WIN_S - 0.1 or rng is None or r0 is None or rng > 1.5:
            return
        if self.wheel_fwd - f0 >= STALL_WHEEL_M and abs(self.gyro_yaw - y0) < math.radians(5) \
                and (r0 - rng) < STALL_CLOSE_M:
            self.stall_until = max(self.stall_until, now + 0.3)
            self.stall_reason = f"depth: wheels +{(self.wheel_fwd - f0) * 100:.0f} cm, range {(r0 - rng) * 100:+.1f} cm"
            self.stall_dir = np.array([1.0, 0.0])

    def update_fused(self):
        """Fused odometry, once per control step. Wheels drift when the omni rollers slip
        under hard strafing (20-30 cm after ~40 s, runs 12c and 13: the robot did not know
        the ball was already in the box). cuVSLAM is accurate but can JUMP with the camera
        facing a plain wall (runs 4-5). So: each step, compare the two sources' motion in
        the robot's own frame; trust cuVSLAM, unless it disagrees with the wheels by more
        than FUSE_JUMP_M, which only a jump can do; then take the wheels' step."""
        if not self.odom_hist or (self.use_gyro and self.gyro_bias is None):
            return False
        _, pw, yw = self.odom_hist[-1]
        gy = self.gyro_yaw
        try:
            if not self.use_vio:
                raise tf2_ros.TransformException("cuVSLAM not used (use_vio:=false)")
            t = self.tf.lookup_transform("odom", "base_link", Time())
            pv, yv = np.array([t.transform.translation.x, t.transform.translation.y]), yaw_of(t.transform.rotation)
            # Kaya lives on the floor: cuVSLAM's height leaving it means tracking is LOST
            # (after run 13 it read z = 20 km, integrating IMU noise). Lost may also drift in
            # small steps that pass the jump test, so it is dropped for the rest of the run.
            z = t.transform.translation.z
            if self.vio_z0 is None:
                self.vio_z0 = z
            if abs(z - self.vio_z0) > 0.10 and not self.vio_dead:
                self.vio_dead = True
                self.get_logger().warn(f"cuVSLAM height moved {z - self.vio_z0:+.2f} m: tracking lost, wheels only")
            if self.vio_dead:
                pv = None
        except tf2_ros.TransformException:
            pv = None
        if self.fused is None:
            self.fused = (np.zeros(2), 0.0)
            self.prev_w, self.prev_v = (pw, yw), ((pv, yv) if pv is not None else None)
            self.prev_g = gy
        else:
            dw = rot(-self.prev_w[1]) @ (pw - self.prev_w[0])
            dyw = math.remainder(yw - self.prev_w[1], 2 * math.pi)
            use_v = pv is not None and self.prev_v is not None
            if use_v:
                dv = rot(-self.prev_v[1]) @ (pv - self.prev_v[0])
                dyv = math.remainder(yv - self.prev_v[1], 2 * math.pi)
                use_v = np.linalg.norm(dv - dw) < FUSE_JUMP_M and abs(dyv - dyw) < FUSE_JUMP_RAD
            d, dy = (dv, dyv) if use_v else (dw, dyw)
            self.n_fuse["vslam" if use_v else "wheel"] += 1
            self.wheel_fwd += float(dw[0])
            if self.stalled():
                # blocked: the wheels spin, the robot does not move - in ANY direction. Run 20's
                # video: pinned against the box's side panel for 25 s while the wheels, strafing,
                # "carried" it 1.5 m along the wall. (Keeping the sideways part, tried after run 21,
                # would have kept that fiction.) Drift that gets through is fixed by the markers.
                d = np.zeros(2)
                self.n_stalled += 1
            if self.use_gyro:
                dy = gy - self.prev_g                    # turning: always the gyro
                self.prev_g = gy
            p, yaw = self.fused
            self.fused = (p + rot(yaw) @ d, yaw + dy)
            self.prev_w, self.prev_v = (pw, yw), ((pv, yv) if pv is not None else None)
        self.fused_hist.append((self.get_clock().now().nanoseconds, self.fused[0].copy(), self.fused[1]))
        if len(self.fused_hist) > 100:
            self.fused_hist.pop(0)
        return True

    def update_goal_from_markers(self, img, stamp):
        """The box pose from the markers, robust to ArUco's planar-pose flip.

        Run 16: seen side-on, the left-panel marker's ORIENTATION flipped, and the box
        estimate jumped ~1 m towards the robot (a false goal against a phantom box).
        A marker's centre POSITION stays reliable at any angle. So:
          * the box's direction comes only from the front marker (ID 0), seen near head-on;
            the box never moves, so it is measured once and then only refined;
          * every marker then places the box from its centre position + that direction;
          * a sighting implying a jump > 15 cm (or a turn > 20 deg) is discarded."""
        from goal_marker import BOX_FROM_MARKER
        res = self.markers.detect(img)
        if not res:
            return
        try:
            p, ryaw = self.pose(stamp)
        except tf2_ros.TransformException:
            return
        R = rot(ryaw)
        accepted = []
        for mid, box_xy_b, box_yaw_b, dist, mc_b in res:
            if mid == 0:
                yaw_w = ryaw + box_yaw_b
                if self.goal_w is None or abs(math.remainder(yaw_w - self.goal_w[1], 2 * math.pi)) < math.radians(20):
                    self.goal_yaw_w = yaw_w if self.goal_yaw_w is None else \
                        self.goal_yaw_w + 0.3 * math.remainder(yaw_w - self.goal_yaw_w, 2 * math.pi)
            if self.goal_yaw_w is None:
                continue                                    # no trusted direction yet: wait for ID 0
            # box origin = marker centre - (marker's offset in the box frame, rotated by the box direction)
            c_box = BOX_FROM_MARKER[mid][:2, 3]
            g = p + R @ mc_b - rot(self.goal_yaw_w) @ c_box
            if self.goal_w is not None and np.linalg.norm(g - self.goal_w[0]) > 0.15:
                # a phantom (a flip) is a one-off; three rejected sightings in a row that agree with
                # each other mean the box has not moved but OUR odometry has (run 21: 14 good
                # sightings rejected after the drift): re-anchor the box to them
                self.marker_rejected += 1
                self.reject_run.append(g)
                self.reject_run = self.reject_run[-3:]
                if len(self.reject_run) == 3 and max(np.linalg.norm(r - np.mean(self.reject_run, axis=0))
                                                     for r in self.reject_run) < 0.05:
                    new = np.mean(self.reject_run, axis=0)
                    self.get_logger().warn(f"box re-anchored by {np.linalg.norm(new - self.goal_w[0]):.2f} m (odometry drift)")
                    self.goal_w = (new, self.goal_yaw_w)
                    self.marker_reanchored += 1
                    self.reject_run = []
                continue
            self.reject_run = []
            accepted.append((mid, g))
        if not accepted:
            return
        g = np.mean([a[1] for a in accepted], axis=0)
        if self.goal_w is not None:
            g = 0.7 * self.goal_w[0] + 0.3 * g
        self.goal_w = (g, self.goal_yaw_w)
        self.goal_seen_t = time.time()
        self.n_marker += 1
        self.last_marker_ids = sorted({a[0] for a in accepted})
        for a in accepted:
            self.marker_counts[a[0]] = self.marker_counts.get(a[0], 0) + 1

    def pose(self, stamp=None):
        """base_link in odom: (x, y, yaw), at stamp if given. From the wheel encoders
        (drive_node /odom: drifts slowly, never jumps), from cuVSLAM via TF (jumped
        0.3-2 m twice with the camera up against a plain wall, runs 4-5), or fused."""
        if self.odom_source == "fused":
            if not self.fused_hist:
                raise tf2_ros.TransformException("no fused odometry yet")
            if stamp is None:
                _, p, yaw = self.fused_hist[-1]
            else:
                ns = Time.from_msg(stamp).nanoseconds
                _, p, yaw = min(self.fused_hist, key=lambda h: abs(h[0] - ns))
            return p.copy(), yaw
        if self.odom_source == "wheel":
            if not self.odom_hist:
                raise tf2_ros.TransformException("no wheel odometry yet")
            if stamp is None:
                _, p, yaw = self.odom_hist[-1]
            else:
                ns = Time.from_msg(stamp).nanoseconds
                _, p, yaw = min(self.odom_hist, key=lambda h: abs(h[0] - ns))
            return p.copy(), yaw
        t = self.tf.lookup_transform("odom", "base_link", Time() if stamp is None else Time.from_msg(stamp),
                                     timeout=rclpy.duration.Duration(seconds=0.05))
        p = t.transform.translation
        return np.array([p.x, p.y]), yaw_of(t.transform.rotation)

    def ensure_ready(self):
        if self.odom_source == "fused" and not self.update_fused():
            return False
        if self.detector is None and self.K is not None:
            try:
                T = tf_matrix(self.tf.lookup_transform("base_link", "camera_color_optical_frame", Time()))
            except tf2_ros.TransformException:
                return False
            self.detector = BallDetector(self.K, self.D, T, self.r, self.r - BASE_LINK_Z, yolo=self.yolo)
            if self.goal_source == "marker":
                from goal_marker import GoalMarkers
                self.markers = GoalMarkers(self.K, self.D, T, size=self.marker_size)
            self.get_logger().info(f"detector ready; colour lens in base_link at {np.round(T[:3, 3], 3)}")
        if self.use_scan and self.scanner is None:
            if self.depth_K is None:
                return False
            try:
                Td = tf_matrix(self.tf.lookup_transform("base_link", "camera_depth_optical_frame", Time()))
            except tf2_ros.TransformException:
                return False
            from depth_scan import DepthScan
            self.scanner = DepthScan(self.depth_K, Td)
            self.get_logger().info(f"depth scan ready; depth lens in base_link at {np.round(Td[:3, 3], 3)}")
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
        if self.markers is not None:
            self.update_goal_from_markers(np.ascontiguousarray(img), msg.header.stamp)
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
        # a ball behind the start is real (run 29: YOLO saw it 1.3 m behind, the check threw it
        # away 69 times and the robot spun for 90 s), so behind, allow the arena + 1 m
        # past the goal wall: judged from the BOX when the markers have placed it (run 33: the start-
        # frame limit, 60 cm off after drift, rejected the real ball beside the box 124 times)
        if self.goal_w is not None:
            beyond = (rot(-self.goal_w[1]) @ (ball_w - self.goal_w[0]))[0] > 0.30 + 0.15
        else:
            beyond = b_s[0] > self.goal_xy[0] + 0.4
        if b_s[0] < -self.arena[0] - 1.0 or beyond or abs(b_s[1]) > self.arena[1] + 0.5:
            return True, self._reject("outside", img, d, b_s)
        # Colour-only detections ("round", "near") need YOLO's backing: run 28 chased yellow-brown
        # cardboard blobs for 27 s while the ball sat behind the robot - YOLO confirmed none of
        # them. So one counts only within COLOUR_TRUST_S of a YOLO sighting, and near where the
        # ball was then (it can have rolled since: the gate grows with the time)
        # A "round" blob can also START a track on its own, if it stays put: 3 sightings within
        # 1.5 s agreeing to 15 cm. The real ball 1.8 m away, too small for YOLO, did (run 30:
        # -1.46/-1.58/-1.53, -1.00/-1.03/-1.04); run 28's cardboard "balls" jumped 0.5-1 m
        # between frames, because a patch of wall fitted as a sphere moves with the viewpoint.
        now = time.time()
        if d.source == "round":
            self.round_buf = [(t, w) for t, w in self.round_buf if now - t < 1.5] + [(now, ball_w)]
            pts = np.array([w for _, w in self.round_buf[-3:]])
            if len(pts) == 3 and np.linalg.norm(pts - pts.mean(0), axis=1).max() < 0.15 and \
                    (self.yolo_t is None or now - self.yolo_t > COLOUR_TRUST_S):
                self.yolo_t, self.yolo_w = now, ball_w          # a steady round blob: trust it
        if d.source == "yolo":
            self.yolo_t, self.yolo_w = now, ball_w
        elif self.yolo_t is None or now - self.yolo_t > COLOUR_TRUST_S or \
                np.linalg.norm(ball_w - self.yolo_w) > 0.25 + 0.5 * (now - self.yolo_t):
            return True, self._reject("unconfirmed", img, d, b_s)
        else:                                                # a continuous track from a YOLO sighting:
            self.yolo_t, self.yolo_w = now, ball_w           # follow it while YOLO can't (a cut-off ball)
        self.n_det += 1
        self.last_fit = (d.rms_deg, d.area_px)
        self.last_src = d.source
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
        # (ensure_ready already advanced the fused odometry for this step)
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
        # odometry sanity: Kaya cannot move 0.25 m in one 0.1 s step (2.5 m/s). Run 4 ended
        # on a silent cuVSLAM jump of 2 m in ~2 s (camera staring at a featureless wall up
        # close); acting on it is worse than stopping.
        if self.prev_p is not None and np.linalg.norm(p - self.prev_p) > 0.25:
            self.done = True
            self.cmd.publish(Twist())
            self.get_logger().error(f"STOP: odometry jumped {np.linalg.norm(p - self.prev_p):.2f} m in one step")
            raise SystemExit
        self.prev_p = p
        sp, syaw = self.start
        # goal (box mouth centre) and its into-box direction, in odom
        if self.goal_w is not None:                      # from the markers
            g_w, gdir = self.goal_w
        else:                                            # from the start mark
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
        # Own speed: blocked, the real wheels keep spinning at the commanded speed, so the policy
        # was told "moving forward" while pinning a ball (the user: it keeps pressing). The sim
        # would say 0 here, so when stalled it gets 0; turning comes from the gyro.
        obs = np.concatenate([
            est_b / 2.0,
            [self.age / AGE_CAP_S],
            [1.0 if self.seen_now else 0.0],
            (to_b @ (gp_w - p)) / 3.0,
            [math.cos(gdir - yaw), math.sin(gdir - yaw)],
            [self.half_w / 0.23],
            (np.zeros(2) if self.stalled() else self.twist_b[:2]) / MAX_LIN,
            [(self.gyro_rate if self.use_gyro else self.twist_b[2]) / MAX_ANG],
            self.action,
        ])
        if self.use_scan:                                  # v2: + the 16-ray depth scan, / 3 m as in training
            scan = self.depth_scan()
            if self.box_in_scan:
                # the box's part of the scan from its known pose: the real depth is blind to the
                # panel the ball gets pinned against (see depth_scan.box_scan)
                from depth_scan import box_scan
                o_g = rot(-gdir) @ (p + rot(yaw) @ np.array([0.024, 0.024]) - g_w)
                self.box_part = box_scan(o_g, yaw - gdir, self.half_w)
                scan = np.minimum(scan, self.box_part)
            obs = np.concatenate([obs, scan / 3.0])
            if self.use_gyro:
                self.check_soft_stall()
        self.action = np.clip(self.policy(obs), -1.0, 1.0)

        # stop conditions
        t = time.time() - self.t0
        p_s = rot(-syaw) @ (p - sp)                        # robot in the start frame
        # Scored = inside the box AND in through the mouth. Inside, the ball's centre is at most
        # half_w - r (16 cm) off-centre; against the OUTSIDE of a side panel it is >= ~31 cm.
        # The first rule (< half_w, 23 cm) sat between the two, and in run 12b a ball pushed
        # along the wall into the outside of a side panel counted as a goal.
        in_lane = abs(b_g[1]) < self.half_w - self.r + 0.03
        if self.ever_seen and b_g[0] < 0.0 and in_lane:
            self.seen_in_front = True
        # ... and SEEN inside in 3 frames in a row over >= 0.5 s: in run 21 a single bad "near"
        # detection (fit 4.35 deg, the ball 23 cm further on than 0.3 s before) called a false goal
        if ball_w is not None:                             # a fresh detection this step
            if b_g[0] > self.r and in_lane:
                self.inside_n += 1
                self.inside_t0 = self.inside_t0 or time.time()
            else:
                self.inside_n, self.inside_t0 = 0, None
        # ... and with the box's position fresh: run 22 "scored" 21 s after the last marker, the
        # box estimate carried by wheel odometry through 16 s of pinned wheel-spin (video: the
        # ball was 50 cm right of the box)
        box_fresh = self.goal_source != "marker" or time.time() - self.goal_seen_t < BOX_FRESH_S
        # ... and with the robot itself not IN the box: run 26 "scored" with the robot inside it.
        # Pushing the ball deep, the robot's centre does enter the mouth a little (run 32: ~9 cm,
        # the real goal went uncalled at 0.0 and the robot kept pushing), so allow 12 cm; fully
        # inside, its centre is >= 15 cm in (half its width).
        robot_out = (rot(-gdir) @ (p - g_w))[0] < ROBOT_IN_BOX_M
        scored = (self.inside_n >= 3 and time.time() - self.inside_t0 >= 0.5 and self.seen_in_front
                  and box_fresh and robot_out)
        left = p_s[0] < -self.arena[0] or abs(p_s[1]) > self.arena[1] or p_s[0] > self.goal_xy[0] + 0.5
        reason = "SCORED" if scored else "time limit" if t > self.max_time else "left the arena" if left else None

        twist = Twist()
        if reason is None and not self.dry_run:
            twist.linear.x = float(self.action[0] * MAX_LIN)
            twist.linear.y = float(self.action[1] * MAX_LIN)
            twist.angular.z = float(self.action[2] * MAX_ANG)
            if self.test_push > 0:                       # stall test: ignore the policy, push straight on
                twist.linear.x, twist.linear.y, twist.angular.z = self.test_push, 0.0, 0.0
                self.action = np.array([self.test_push / MAX_LIN, 0.0, 0.0])
        if not self.dry_run or reason:
            self.cmd.publish(twist)
        if int(t * 10) % 5 == 0 or reason:
            ball_s = rot(-syaw) @ (self.est_w - sp)
            self.get_logger().info(
                f"t {t:5.1f}s robot ({p_s[0]:+.2f},{p_s[1]:+.2f}) | ball {'SEEN' if self.seen_now else f'mem {self.age:.1f}s' if self.ever_seen else 'never seen'}"
                f" at ({ball_s[0]:+.2f},{ball_s[1]:+.2f}) goal-frame ({b_g[0]:+.2f},{b_g[1]:+.2f}) | action "
                f"({self.action[0]:+.2f},{self.action[1]:+.2f},{self.action[2]:+.2f}) | det {np.mean(self.det_ms[-20:]) if self.det_ms else 0:.0f} ms"
                f" fit {self.last_fit[0]:.2f} deg {self.last_src}"
                + (f" | scan L->R {np.round(self.last_scan[::3], 2)}" if self.use_scan else "")
                + (f" box {np.round(self.box_part[::3], 2)}" if self.box_part is not None else "")
                + (f" | goal {'marker ' + str(self.last_marker_ids) + ' ' + format(time.time() - self.goal_seen_t, '.1f') + 's' if self.goal_w is not None else 'MARK'}"
                   if self.goal_source == "marker" else "")
                + (f" | STALLED ({self.stall_reason})" if self.stalled() else ""))
        if reason:
            self.done = True
            self.get_logger().warn(f"STOP: {reason} after {t:.1f} s ({self.n_det} detections, {self.n_marker} marker sightings by id {self.marker_counts}, {self.marker_rejected} marker jumps rejected ({self.marker_reanchored} re-anchors), rejected {self.n_reject}"
                                   + (f", odometry steps from {self.n_fuse}, stalled steps {self.n_stalled}" if self.odom_source == "fused" else "") + ")")
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
