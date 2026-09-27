"""Phase 9: remember what places looked like, and where they were.

    source /opt/ros/jazzy/setup.bash
    python3 perception/keyframe_store.py            # drive the robot; it records

Saves a colour keyframe plus its MAP pose whenever the robot has moved far
enough, turned far enough, or the view has actually changed. That store is what
Phase 10 reasons over: "show me where I can cook" becomes "which of these
remembered places looks like a kitchen", and the answer is a pose Nav2 can
already drive to at 6 cm.

RUNS ON THE WORKSTATION, off the THROTTLED feed
It subscribes to /kaya/perception/color, the 2 Hz republish, not the camera's
own topic. That is not a detail: a remote subscriber on the full-rate topics
starved the robot badly enough that collision_monitor stopped the wheels and TF
fell behind -- see kaya_bringup/perception_throttle_node.py. Writing JPEGs is
nearly free, but only if the frames were cheap to get.

WHEN IT SAVES, and why three triggers rather than one

  DISTANCE   every `min_move_m` of travel. The roadmap's rule, and it is what
             gives even coverage of a space that is being driven through.
  ROTATION   every `min_turn_deg`. A robot that turns on the spot sees an
             entirely new room without moving a centimetre, and a
             distance-only trigger would miss all of it.
  APPEARANCE when the view changes even though the robot has not. Someone
             opens a door, or the robot is parked while the scene changes.
             Measured cheaply on a 32x24 greyscale thumbnail -- no GPU, no
             model, a few microseconds -- because the semantic layer must
             never compete with navigation.

WHAT IT REFUSES TO SAVE
A keyframe whose pose is not trustworthy is worse than no keyframe: it teaches
the store that a place is somewhere it is not, and Phase 10 would then send the
robot there. So nothing is written unless the map frame is anchored and the
pose passes the same flat-floor sanity the rest of this project uses. This
robot has already been seen driving on a pose that had diverged to 13 km.
"""

import argparse
import json
import math
import os
import time

import numpy as np

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import tf2_ros

SENSOR = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class KeyframeStore(Node):

    def __init__(self, args):
        super().__init__('keyframe_store')
        self.dir = os.path.expanduser(args.dir)
        os.makedirs(os.path.join(self.dir, 'images'), exist_ok=True)
        self.index_path = os.path.join(self.dir, 'index.json')
        self.index = []
        if os.path.isfile(self.index_path):
            try:
                with open(self.index_path) as f:
                    self.index = json.load(f)
                self.get_logger().info('resuming a store of %d keyframes'
                                       % len(self.index))
            except ValueError:
                self.get_logger().warn('index.json unreadable; starting fresh')

        self.min_move = args.min_move_m
        self.min_turn = math.radians(args.min_turn_deg)
        self.appearance = args.appearance
        self.min_appearance_move = args.min_appearance_move_m
        self.map_frame, self.base_frame = args.map_frame, args.base_frame

        self.rgb = None
        self.last = None            # (x, y, yaw) of the last saved keyframe
        self.last_thumb = None
        self.anchored = False
        self.saved_this_run = 0

        self.create_subscription(CompressedImage, args.image_topic,
                                 self.on_rgb, SENSOR)
        self.create_subscription(Bool, '/cuvslam_origin_anchor/anchored',
                                 self.on_anchored, LATCHED)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            'store at %s; saving every %.1f m, %.0f deg%s'
            % (self.dir, self.min_move, args.min_turn_deg,
               ', or on a changed view' if self.appearance else ''))

    def on_rgb(self, m):
        a = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if a is not None:
            self.rgb = a

    def on_anchored(self, m):
        if m.data and not self.anchored:
            self.get_logger().info('map frame anchored; recording enabled')
        elif not m.data and self.anchored:
            self.get_logger().warn(
                'map frame NOT anchored; recording paused -- a keyframe with a '
                'wrong pose teaches the store that a place is somewhere it is not')
        self.anchored = m.data

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()).transform
        except tf2_ros.TransformException:
            return None
        q = t.rotation
        # Same flat-floor sanity the anchor and the pose guard apply. A robot
        # that believes it is half a metre underground is not somewhere worth
        # remembering.
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                          1 - 2 * (q.x * q.x + q.y * q.y))
        if abs(t.translation.z) > 0.30 or abs(math.degrees(roll)) > 15:
            return None
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    @staticmethod
    def thumb(img):
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return cv2.resize(g, (32, 24)).astype(np.float32)

    def why_save(self, p, img):
        if self.last is None:
            return 'first'
        d = math.hypot(p[0] - self.last[0], p[1] - self.last[1])
        if d >= self.min_move:
            return 'moved %.2f m' % d
        turn = abs((p[2] - self.last[2] + math.pi) % (2 * math.pi) - math.pi)
        if turn >= self.min_turn:
            return 'turned %.0f deg' % math.degrees(turn)
        # The appearance trigger needs a floor under it. MEASURED on a first
        # real drive: with no minimum spacing it fired 427 times out of 428
        # keyframes and the distance and rotation triggers never fired at all.
        # A 15 minute drive produced 428 frames that thin to 62 distinct
        # places -- the store was not richer for it, just heavier, and a
        # shortlist built from near-duplicates wastes a reasoning model's
        # attention on the same wall eight times.
        if (self.appearance and self.last_thumb is not None
                and d >= self.min_appearance_move):
            t = self.thumb(img)
            change = float(np.abs(t - self.last_thumb).mean())
            if change >= self.appearance:
                return 'view changed (%.1f)' % change
        return None

    def tick(self):
        if self.rgb is None or not self.anchored:
            return
        p = self.pose()
        if p is None:
            return
        img = self.rgb
        why = self.why_save(p, img)
        if not why:
            return
        stamp = time.time()
        name = 'kf_%d_%06d.jpg' % (int(stamp), len(self.index))
        cv2.imwrite(os.path.join(self.dir, 'images', name), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 85])
        self.index.append({
            'file': name,
            'x': round(p[0], 3), 'y': round(p[1], 3),
            'yaw_deg': round(math.degrees(p[2]), 1),
            'stamp': stamp,
            'iso': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(stamp)),
            'why': why,
        })
        # Written every time, not at shutdown: a store that only survives a
        # clean exit is a store this project would have lost to a flat battery
        # more than once.
        with open(self.index_path, 'w') as f:
            json.dump(self.index, f, indent=1)
        self.last = p
        self.last_thumb = self.thumb(img)
        self.saved_this_run += 1
        self.get_logger().info(
            'keyframe %d at (%+.2f, %+.2f) %+.0f deg -- %s'
            % (len(self.index), p[0], p[1], math.degrees(p[2]), why))


def main():
    ap = argparse.ArgumentParser()
    # default: <workspace>/keyframes, the workspace being the folder that
    # contains this repo (KAYA_WORKSPACE overrides) - see scene_memory.py
    ap.add_argument('--dir', default=os.path.join(os.environ.get(
        'KAYA_WORKSPACE', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       os.pardir, os.pardir)), 'keyframes'))
    ap.add_argument('--image-topic', default='/kaya/perception/color')
    ap.add_argument('--min-move-m', type=float, default=1.0)
    ap.add_argument('--min-turn-deg', type=float, default=45.0)
    ap.add_argument('--min-appearance-move-m', type=float, default=0.25,
                    help='a changed view only counts as a new keyframe once '
                         'the robot has moved at least this far; without it '
                         'the appearance trigger fires continuously')
    ap.add_argument('--appearance', type=float, default=25.0,
                    help='mean abs difference on a 32x24 grey thumbnail; '
                         '0 disables the view-change trigger')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='base_footprint')
    args = ap.parse_args()

    rclpy.init()
    node = KeyframeStore(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('saved %d keyframes this run; %d in the store'
                               % (node.saved_this_run, len(node.index)))
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                             # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
