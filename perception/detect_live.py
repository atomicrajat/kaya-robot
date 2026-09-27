"""Run open-vocabulary detection on the robot's live stream. Publishes NOTHING.

Safe by construction: it subscribes, detects, projects to the map frame and
prints. It has no publisher, so it cannot move the robot. Use it to confirm the
perception chain before letting kaya_follower.py act on it.

    ../perception_venv/bin/python perception/detect_live.py "a red backpack" ...
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2                                                    # noqa: E402
from detector import OwlDetector, box_depth                   # noqa: E402
from geometry_msgs.msg import PointStamped                    # noqa: E402
import rclpy                                                  # noqa: E402
from rclpy.node import Node                                   # noqa: E402
from rclpy.qos import (HistoryPolicy, QoSProfile,             # noqa: E402
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image   # noqa: E402
import tf2_ros                                                # noqa: E402
from tf2_geometry_msgs import do_transform_point              # noqa: E402

SENSOR = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT)
PROMPTS = sys.argv[1:] or ['a person', 'a chair', 'a box', 'a bag']

rclpy.init()
n = Node('detect_live')
state = {'rgb': None, 'depth': None, 'info': None}


def on_rgb(m):
    a = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    if a is not None:
        state['rgb'] = a[:, :, ::-1].copy()


def on_depth(m):
    if m.encoding == '16UC1':
        state['depth'] = (np.frombuffer(m.data, dtype=np.uint16)
                          .reshape(m.height, m.width).astype(np.float32) / 1000.0)


def on_info(m):
    state['info'] = (m.k[0], m.k[4], m.k[2], m.k[5], m.header.frame_id)


n.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', on_rgb, SENSOR)
n.create_subscription(Image, '/camera/depth/image_raw', on_depth, SENSOR)
n.create_subscription(CameraInfo, '/camera/color/camera_info', on_info, SENSOR)
buf = tf2_ros.Buffer()
lis = tf2_ros.TransformListener(buf, n)

print('waiting for camera...', flush=True)
def ready():
    # `all(state.values())` is wrong here: the values are numpy arrays, and
    # truth-testing one raises. Check for None explicitly.
    return all(v is not None for v in state.values())


t0 = time.time()
while time.time() - t0 < 25 and not ready():
    rclpy.spin_once(n, timeout_sec=0.2)
if not ready():
    sys.exit('missing: ' + ', '.join(k for k, v in state.items() if v is None))

print('loading OWLv2...', flush=True)
det = OwlDetector(threshold=0.10)
print('ready on %s. prompts: %s\n' % (det.device, PROMPTS), flush=True)

for run in range(6):
    for _ in range(12):
        rclpy.spin_once(n, timeout_sec=0.1)
    rgb, depth, info = state['rgb'], state['depth'], state['info']
    dets = det.detect(rgb, PROMPTS)
    print('--- pass %d (%.0f ms, %d detections) ---' % (run + 1, det.last_ms, len(dets)), flush=True)
    for d in dets[:4]:
        z = box_depth(depth, d['box'])
        where = ''
        if z is not None:
            fx, fy, cx, cy, frame = info
            x0, y0, x1, y1 = d['box']
            p = PointStamped()
            p.header.frame_id = frame
            p.point.x = ((x0 + x1) / 2 - cx) * z / fx
            p.point.y = ((y0 + y1) / 2 - cy) * z / fy
            p.point.z = z
            try:
                tf = buf.lookup_transform('map', frame, rclpy.time.Time())
                m = do_transform_point(p, tf).point
                where = '  range %.2f m  ->  map (%+.2f, %+.2f)' % (z, m.x, m.y)
            except Exception:
                where = '  range %.2f m  (no TF)' % z
        else:
            where = '  no depth in box'
        print('   %-22s %.3f%s' % (d['label'], d['score'], where), flush=True)
    time.sleep(1.0)
