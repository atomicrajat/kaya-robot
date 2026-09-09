"""Follow something named in words: "follow the red backpack".

    source /opt/ros/jazzy/setup.bash
    ../perception_venv/bin/python perception/kaya_follower.py

RUNS ON THE WORKSTATION, and does not need to run anywhere else. The robot
publishes images and executes goals; everything between is arithmetic and a
ViT, and the Jetson has ~600 MB free with the saved map loaded. The workstation
sees every robot topic over DDS -- the same path the dashboard already drives
the robot through.

WHAT IT DOES

    target named in words   -> OWLv2 finds a box          (perception/detector.py)
    box                     -> median depth inside it     (37% of pixels return)
    pixel + depth           -> 3D point, camera frame
    TF                      -> that point in the MAP frame
    stand off ~1 m          -> a goal Nav2 can drive to
    republish as it moves

Nav2 does the driving and the obstacle avoidance, which it already does well:
following is a goal-republishing problem, not a new controller.

BEHAVIOUR THAT MATTERS, and why each rule is here

  STANDOFF, NOT PURSUIT. The goal is a point `standoff_m` SHORT of the target,
  on the robot's side. Driving at the detection itself means arriving at a
  person's feet.

  A LOST TARGET STOPS. If the target has not been seen for `lost_after_s`, the
  robot is sent a goal at its own current pose -- it halts where it is. It does
  NOT extrapolate the target's motion. Guessing where a person went and driving
  there is how a follower hurts someone.

  NO DEPTH IS NOT A DISTANCE. Only ~37% of depth pixels carry a return in this
  room, so a box regularly has none. That is treated as "not seen", not as
  distance zero.

  THE CAMERA FACES FORWARD ONLY. A target that leaves the ~90 deg FOV is simply
  gone. The goal heading always points at the target, so Nav2 turns the robot
  to keep it in view; that is the whole behaviour, and it is why heading is set
  from the target rather than left at zero.

  SPEED IS CAPPED IN nav2.yaml (0.15 m/s) and nothing here raises it.
"""

import json
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from detector import OwlDetector, box_depth                       # noqa: E402
from geometry_msgs.msg import PointStamped, PoseStamped           # noqa: E402
import rclpy                                                      # noqa: E402
from rclpy.node import Node                                       # noqa: E402
from rclpy.qos import (HistoryPolicy, QoSProfile,                 # noqa: E402
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image    # noqa: E402
from std_msgs.msg import String                                   # noqa: E402
import tf2_ros                                                    # noqa: E402
from tf2_geometry_msgs import do_transform_point                  # noqa: E402

SENSOR = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT)


class Follower(Node):

    def __init__(self):
        super().__init__('kaya_follower')
        self.declare_parameter('standoff_m', 1.0)
        self.declare_parameter('lost_after_s', 2.0)
        self.declare_parameter('republish_min_move_m', 0.15)
        self.declare_parameter('goal_rate_hz', 2.0)
        self.declare_parameter('min_score', 0.12)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        # Off by default. Naming a target starts it; nothing about loading this
        # node should make the robot drive.
        self.declare_parameter('target', '')

        self.standoff = self.get_parameter('standoff_m').value
        self.lost_after = self.get_parameter('lost_after_s').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.lock = threading.Lock()
        self.rgb = None
        self.depth = None
        self.info = None
        self.target = self.get_parameter('target').value
        self.last_seen = 0.0
        self.last_goal = None
        self.halted = False
        self.det = None          # latest accepted detection
        self.enabled = bool(self.target)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CompressedImage,
                                 '/camera/color/image_raw/compressed',
                                 self.on_rgb, SENSOR)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self.on_depth, SENSOR)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self.on_info, SENSOR)
        self.create_subscription(String, '/kaya/track_target',
                                 self.on_target, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/kaya/track_status', 10)

        self.get_logger().info('loading OWLv2 (first run downloads weights)...')
        self.detector = OwlDetector(threshold=self.get_parameter('min_score').value)
        self.get_logger().info('detector ready on %s' % self.detector.device)

        threading.Thread(target=self.detect_loop, daemon=True).start()
        self.create_timer(1.0 / self.get_parameter('goal_rate_hz').value, self.tick)

    # ------------------------------------------------------------ ROS inputs

    def on_rgb(self, m):
        import cv2
        a = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if a is None:
            return
        with self.lock:
            self.rgb = a[:, :, ::-1].copy()           # BGR -> RGB

    def on_depth(self, m):
        if m.encoding != '16UC1':
            return
        a = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
        with self.lock:
            self.depth = a.astype(np.float32) / 1000.0

    def on_info(self, m):
        with self.lock:
            self.info = (m.k[0], m.k[4], m.k[2], m.k[5], m.header.frame_id)

    def on_target(self, m):
        t = m.data.strip()
        with self.lock:
            self.target = t
            self.enabled = bool(t)
            self.det = None
            self.last_goal = None
            self.halted = False
        self.get_logger().info('target set to %r' % t if t else 'tracking stopped')

    # -------------------------------------------------------- detection loop

    def detect_loop(self):
        """Detection runs in its own thread; the ViT must not stall the timers."""
        while rclpy.ok():
            with self.lock:
                rgb, target, on = self.rgb, self.target, self.enabled
            if not on or rgb is None or not target:
                time.sleep(0.1)
                continue
            try:
                dets = self.detector.detect(rgb, [target])
            except Exception as e:                    # noqa: BLE001
                self.get_logger().error('detect failed: %r' % e)
                time.sleep(1.0)
                continue
            with self.lock:
                if dets:
                    self.det = dets[0]
                    self.last_seen = time.time()
                self.det_ms = self.detector.last_ms

    # ---------------------------------------------------------- goal from box

    def target_point_map(self, det):
        """Box -> 3D point in the map frame, or None if anything is missing."""
        with self.lock:
            depth, info = self.depth, self.info
        if depth is None or info is None:
            return None
        fx, fy, cx, cy, frame = info
        z = box_depth(depth, det['box'])
        if z is None:
            return None
        x0, y0, x1, y1 = det['box']
        u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        p = PointStamped()
        p.header.frame_id = frame
        p.point.x = (u - cx) * z / fx
        p.point.y = (v - cy) * z / fy
        p.point.z = z
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        m = do_transform_point(p, tf).point
        return (m.x, m.y, z)

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()).transform
        except tf2_ros.TransformException:
            return None
        return (t.translation.x, t.translation.y)

    def publish_goal(self, x, y, yaw):
        g = PoseStamped()
        g.header.frame_id = self.map_frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = float(x), float(y)
        g.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(g)

    def status(self, state, **kw):
        kw.update({'state': state, 'target': self.target,
                   'detect_ms': getattr(self, 'det_ms', None)})
        self.status_pub.publish(String(data=json.dumps(kw)))

    def tick(self):
        with self.lock:
            on, det, seen = self.enabled, self.det, self.last_seen
        if not on:
            self.status('idle')
            return

        age = time.time() - seen
        if det is None or age > self.lost_after:
            # Lost: halt where we are, once, and say so. Never extrapolate.
            if not self.halted:
                rp = self.robot_xy()
                if rp:
                    self.publish_goal(rp[0], rp[1], 0.0)
                self.halted = True
                self.get_logger().warn(
                    'target %r lost (%.1f s); holding position' % (self.target, age))
            self.status('lost', age_s=round(age, 2))
            return

        pt = self.target_point_map(det)
        if pt is None:
            self.status('no_depth', score=round(det['score'], 3))
            return

        tx, ty, rng = pt
        rp = self.robot_xy()
        if rp is None:
            self.status('no_tf')
            return
        dx, dy = tx - rp[0], ty - rp[1]
        d = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        if d <= self.standoff:
            # Close enough: hold station, but keep facing the target so it does
            # not leave the forward-only FOV while we sit still.
            gx, gy = rp
        else:
            k = (d - self.standoff) / d
            gx, gy = rp[0] + dx * k, rp[1] + dy * k

        moved = (self.last_goal is None or
                 math.hypot(gx - self.last_goal[0], gy - self.last_goal[1])
                 > self.get_parameter('republish_min_move_m').value)
        if moved:
            self.publish_goal(gx, gy, yaw)
            self.last_goal = (gx, gy)
        self.halted = False
        self.status('tracking', score=round(det['score'], 3),
                    box=[round(v, 1) for v in det['box']],
                    range_m=round(rng, 2), goal=[round(gx, 3), round(gy, 3)])


def main():
    rclpy.init()
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                             # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
