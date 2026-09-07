"""Measure image-timestamp jitter and IMU-vs-image clock consistency.

cuVSLAM fuses a 200 Hz IMU with 30 Hz images, and that fusion depends on the
two sharing a consistent time base. Jitter in the IMAGE stamps relative to the
IMU is unmodelled rotation between frames -- invisible when the robot is still,
and directly proportional to error once it moves.

  frame jitter   spread of image inter-frame intervals about the nominal
                 33.3 ms. Hardware timestamps should be near-perfect; host
                 arrival timestamps carry USB scheduling jitter.
  imu jitter     same for the IMU stream about its own nominal period.
  skew           difference between image and IMU stamp lag. If the two
                 streams are stamped from different clocks this is non-zero
                 and drifts, which is what breaks visual-inertial fusion.
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu


def st(m):
    return m.header.stamp.sec + m.header.stamp.nanosec / 1e9


class S(Node):
    def __init__(self, label, dur):
        super().__init__('imu_cam_sync')
        q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200,
                       reliability=ReliabilityPolicy.BEST_EFFORT)
        self.img = []
        self.imu = []
        self.img_lag = []
        self.imu_lag = []
        self.create_subscription(Image, '/camera/left_ir/image_raw', self.ci, q)
        self.create_subscription(Imu, '/camera/gyro_accel/sample', self.cu, q)
        self.label = label
        self.dur = dur
        self.create_timer(0.5, self.tick)

    def _lag(self, m):
        return (self.get_clock().now().nanoseconds / 1e9) - st(m)

    def ci(self, m):
        self.img.append(st(m)); self.img_lag.append(self._lag(m))

    def cu(self, m):
        self.imu.append(st(m)); self.imu_lag.append(self._lag(m))

    def tick(self):
        self.dur -= 0.5
        if self.dur > 0:
            return
        if len(self.img) < 100 or len(self.imu) < 500:
            print('%s: insufficient data (img=%d imu=%d)'
                  % (self.label, len(self.img), len(self.imu)))
            rclpy.shutdown(); return
        di = np.diff(sorted(self.img)) * 1000.0
        du = np.diff(sorted(self.imu)) * 1000.0
        il = np.array(self.img_lag); ul = np.array(self.imu_lag)
        print('=== %s ===' % self.label)
        print('  image interval : mean %7.3f ms  std %6.3f ms  max dev %6.3f ms'
              % (di.mean(), di.std(), np.abs(di - di.mean()).max()))
        print('  imu   interval : mean %7.3f ms  std %6.3f ms  max dev %6.3f ms'
              % (du.mean(), du.std(), np.abs(du - du.mean()).max()))
        print('  image lag      : %7.3f s   imu lag: %7.3f s' % (il.mean(), ul.mean()))
        print('  IMAGE-IMU SKEW : %7.1f ms   <-- fusion needs this small AND stable'
              % ((il.mean() - ul.mean()) * 1000.0))
        rclpy.shutdown()


rclpy.init()
try:
    rclpy.spin(S(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 15.0))
except Exception:
    pass
