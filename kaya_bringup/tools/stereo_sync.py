"""Is the IR stereo pair still delivering MATCHED frames?

cuVSLAM triangulates from left/right pairs. If frames arrive unpaired -- one
eye dropping frames the other keeps -- triangulation degrades and the scale of
the whole trajectory drifts, while vo_state happily still reports 1. That is
the failure signature we are chasing: 15% position drift with tracking that
looks healthy.

Verified good earlier in this project: 0.178 ms median stereo sync.
"""
import collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec / 1e9


class S(Node):
    def __init__(self):
        super().__init__('stereo_sync')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.L = []
        self.R = []
        self.create_subscription(Image, '/camera/left_ir/image_raw',
                                 lambda m: self.L.append(stamp(m)), qos)
        self.create_subscription(Image, '/camera/right_ir/image_raw',
                                 lambda m: self.R.append(stamp(m)), qos)
        self.create_timer(1.0, self.tick)

    def tick(self):
        if len(self.L) < 400 or len(self.R) < 400:
            return
        L, R = sorted(self.L), sorted(self.R)
        print('left frames  : %d' % len(L))
        print('right frames : %d' % len(R))
        print('difference   : %d  (%.1f%% of right)'
              % (len(R) - len(L), 100.0 * abs(len(R) - len(L)) / len(R)))

        # For each left frame find nearest right stamp.
        import bisect
        d = []
        unmatched = 0
        for t in L:
            i = bisect.bisect_left(R, t)
            best = min((abs(t - R[j]) for j in (i - 1, i, i + 1) if 0 <= j < len(R)),
                       default=None)
            if best is None:
                continue
            d.append(best * 1000.0)
            if best * 1000.0 > 5.0:
                unmatched += 1
        d.sort()
        print('\nstereo pairing (each left frame to nearest right):')
        print('  median %.3f ms   p90 %.3f ms   max %.3f ms'
              % (d[len(d)//2], d[int(len(d)*0.9)], d[-1]))
        print('  left frames with NO right within 5 ms: %d (%.1f%%)'
              % (unmatched, 100.0 * unmatched / len(d)))
        print('  (verified good earlier in this project: 0.178 ms median)')
        rclpy.shutdown()


rclpy.init()
try:
    rclpy.spin(S())
except Exception:
    pass
