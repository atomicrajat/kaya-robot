"""How far behind ROS time is each transform's own header stamp?

A transform published at 25 Hz can still be rejected as "too old" if it is
STAMPED in the past. Nav2 compares the stamp against now, not against arrival.
"""
import collections, time
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image

class R(Node):
    def __init__(self):
        super().__init__('stamp_lag')
        self.lag = collections.defaultdict(list)
        self.create_subscription(TFMessage, '/tf', self.cb, 200)
        self.create_subscription(Image, '/camera/left_ir/image_raw', self.img, 10)
        self.t0 = time.time()
        self.create_timer(0.5, self.tick)

    def _lag(self, key, stamp):
        now = self.get_clock().now().nanoseconds
        s = stamp.sec * 10**9 + stamp.nanosec
        self.lag[key].append((now - s) / 1e9)

    def cb(self, m):
        for t in m.transforms:
            self._lag('%s -> %s' % (t.header.frame_id, t.child_frame_id), t.header.stamp)

    def img(self, m):
        self._lag('IMAGE /camera/left_ir', m.header.stamp)

    def tick(self):
        if time.time() - self.t0 < 15.0:
            return
        print('%-42s %7s %9s %9s %9s' % ('topic/transform', 'n', 'min lag', 'median', 'max lag'))
        for k in sorted(self.lag):
            v = sorted(self.lag[k])
            print('%-42s %7d %8.3fs %8.3fs %8.3fs'
                  % (k, len(v), v[0], v[len(v)//2], v[-1]))
        rclpy.shutdown()

rclpy.init()
try: rclpy.spin(R())
except Exception: pass
