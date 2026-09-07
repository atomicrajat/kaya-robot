"""Measure image SHARPNESS, not just brightness.

Brightness and gradient say a scene is fine while the robot stands still.
Motion blur is invisible to both at rest and destroys tracking the moment the
robot drives -- which is exactly the symptom here: 1.4 mm drift over 94 s
stationary, 15-32% under motion. NVIDIA's own guidance says exposure must be
short enough to avoid blur, and auto-exposure in a dim room does the opposite.

  sharpness  variance of the Laplacian. The standard focus/blur metric.
             Collapses under motion blur while brightness stays put.
  texture    mean absolute gradient, kept for comparison with earlier runs.
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class B(Node):
    def __init__(self, label, dur):
        super().__init__('blur_check')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/left_ir/image_raw', self.cb, qos)
        self.s = []
        self.t = []
        self.b = []
        self.label = label
        self.dur = dur
        self.create_timer(0.5, self.tick)
        self.n = 0

    def cb(self, m):
        a = np.frombuffer(m.data, dtype=np.uint8)
        if a.size < m.height * m.width:
            return
        im = a[:m.height * m.width].reshape(m.height, m.width).astype(np.float32)
        # 4-neighbour Laplacian; its variance is the classic blur metric.
        lap = (im[:-2, 1:-1] + im[2:, 1:-1] + im[1:-1, :-2] + im[1:-1, 2:]
               - 4.0 * im[1:-1, 1:-1])
        self.s.append(float(lap.var()))
        g = im[::4, ::4]
        self.t.append(float((np.abs(np.diff(g, axis=1)).mean()
                             + np.abs(np.diff(g, axis=0)).mean()) / 2))
        self.b.append(float(im.mean()))
        self.n += 1

    def tick(self):
        self.dur -= 0.5
        if self.dur > 0:
            return
        if not self.s:
            print('%s: NO IMAGES' % self.label)
            rclpy.shutdown()
            return
        s = sorted(self.s); t = sorted(self.t); b = sorted(self.b)
        print('%-34s sharpness %8.1f   texture %6.2f   brightness %5.1f  (n=%d)'
              % (self.label, s[len(s) // 2], t[len(t) // 2], b[len(b) // 2], self.n))
        rclpy.shutdown()


rclpy.init()
try:
    rclpy.spin(B(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 12.0))
except Exception:
    pass
