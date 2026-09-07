"""Measure what the camera actually has to work with, alongside pose drift.

cuVSLAM exposes no feature count (its status message carries only vo_state and
timings, and the landmark visualisation topics were not publishing), so this
measures the input instead -- which is the more honest thing to measure anyway.

  brightness  mean pixel value of the left IR image. Very low means the
              scene is under-lit and features will be noisy.
  texture     mean absolute gradient. This is the one that matters: a bright
              blank wall has high brightness and near-zero texture, and VIO
              cannot track it. Low texture predicts drift far better than
              low brightness does.

Logged against pose so a drift stretch can be read against what the camera
could see at the time.
"""
import csv
import math
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import tf2_ros

OUT = '/tmp/scene_quality.csv'


class S(Node):
    def __init__(self):
        super().__init__('scene_quality')
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/left_ir/image_raw', self.img, qos)
        self.bright = 0.0
        self.tex = 0.0
        self.f = open(OUT, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'z', 'yaw', 'path', 'brightness', 'texture'])
        self.t0 = time.time()
        self.prev = None
        self.path = 0.0
        self.n = 0
        self.tex_hist = []
        self.create_timer(0.2, self.tick)

    def img(self, m):
        a = np.frombuffer(m.data, dtype=np.uint8)
        if a.size < m.height * m.width:
            return
        a = a[:m.height * m.width].reshape(m.height, m.width)
        # Subsample: full-res gradient every frame is wasteful and we only
        # need a scene-level statistic.
        s = a[::4, ::4].astype(np.int16)
        self.bright = float(s.mean())
        gx = np.abs(np.diff(s, axis=1)).mean()
        gy = np.abs(np.diff(s, axis=0)).mean()
        self.tex = float((gx + gy) / 2.0)

    def tick(self):
        el = time.time() - self.t0
        try:
            t = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z)))
        if self.prev is not None:
            self.path += math.hypot(tr.x - self.prev[0], tr.y - self.prev[1])
        self.prev = (tr.x, tr.y)
        self.tex_hist.append(self.tex)
        self.w.writerow(['%.2f' % el, '%.4f' % tr.x, '%.4f' % tr.y, '%.4f' % tr.z,
                         '%.2f' % yaw, '%.3f' % self.path,
                         '%.1f' % self.bright, '%.2f' % self.tex])
        self.n += 1
        if self.n % 50 == 0:
            self.f.flush()
            print('t=%4.0fs x=%+.2f y=%+.2f Z=%+.3f yaw=%+6.1f path=%5.1fm | '
                  'brightness=%5.1f texture=%5.2f'
                  % (el, tr.x, tr.y, tr.z, yaw, self.path, self.bright, self.tex),
                  flush=True)

    def report(self):
        self.f.flush(); self.f.close()
        if not self.tex_hist or self.prev is None:
            print('NO DATA'); return
        tx = sorted(self.tex_hist)
        print('\n=== RUN SUMMARY ===')
        print('path         : %.2f m over %.0f s' % (self.path, time.time() - self.t0))
        print('final pose   : x=%+.3f y=%+.3f  -> %.3f m from origin'
              % (self.prev[0], self.prev[1], math.hypot(*self.prev)))
        print('texture      : min %.2f  p10 %.2f  median %.2f  max %.2f'
              % (tx[0], tx[int(len(tx) * 0.1)], tx[len(tx) // 2], tx[-1]))
        poor = sum(1 for v in self.tex_hist if v < 3.0)
        print('samples with texture < 3.0 (little to track): %d / %d (%.1f%%)'
              % (poor, len(tx), 100.0 * poor / len(tx)))
        print('log: %s' % OUT)


rclpy.init()
n = S()
signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
try:
    rclpy.spin(n)
except (KeyboardInterrupt, Exception):
    pass
n.report()
