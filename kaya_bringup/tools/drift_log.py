"""Measure cuVSLAM drift over a there-and-back loop, with Z called out.

Z is the point of this run. A 32-minute mapping session ended with the robot
reading -0.451 m in map frame while standing on a flat floor, which put
nvblox's obstacle band 55 cm above the ground the robot was on. Either that
was drift, or the robot did not finish where we assumed. A short loop back to
a MARKED start distinguishes them: on flat ground the honest answer for Z is
zero, and any departure is drift accumulated over a known path length.
"""
import csv
import math
import signal
import time

import rclpy
from rclpy.node import Node
import tf2_ros

OUT = '/tmp/drift_log.csv'


def rpy(q):
    r = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    sp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    p = math.asin(sp)
    y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return math.degrees(r), math.degrees(p), math.degrees(y)


class D(Node):
    def __init__(self):
        super().__init__('drift_log')
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.f = open(OUT, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'path'])
        self.t0 = time.time()
        self.prev = None
        self.path = 0.0
        self.zmin = self.zmax = None
        self.n = 0
        self.create_timer(0.2, self.tick)

    def tick(self):
        el = time.time() - self.t0
        try:
            t = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return
        tr = t.transform.translation
        r, p, y = rpy(t.transform.rotation)
        if self.prev is not None:
            self.path += math.hypot(tr.x - self.prev[0], tr.y - self.prev[1])
        self.prev = (tr.x, tr.y)
        self.zmin = tr.z if self.zmin is None else min(self.zmin, tr.z)
        self.zmax = tr.z if self.zmax is None else max(self.zmax, tr.z)
        self.w.writerow(['%.2f' % el] + ['%.4f' % v for v in
                                         (tr.x, tr.y, tr.z, r, p, y, self.path)])
        self.n += 1
        if self.n % 50 == 0:
            self.f.flush()
            print('t=%5.0fs  x=%+.3f y=%+.3f  Z=%+.3f  roll=%+5.1f pitch=%+5.1f '
                  'yaw=%+6.1f  path=%.1fm'
                  % (el, tr.x, tr.y, tr.z, r, p, y, self.path), flush=True)

    def report(self):
        self.f.flush()
        self.f.close()
        if self.prev is None:
            print('NO TF DATA')
            return
        print('\n=== DRIFT SUMMARY ===')
        print('path driven      : %.2f m over %.0f s' % (self.path, time.time() - self.t0))
        print('Z excursion      : %+.3f .. %+.3f m  (span %.3f m)'
              % (self.zmin, self.zmax, self.zmax - self.zmin))
        print('log              : %s' % OUT)
        print('\nCompare the FINAL row against the start you marked.')


rclpy.init()
n = D()
signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
try:
    rclpy.spin(n)
except (KeyboardInterrupt, Exception):
    pass
n.report()
