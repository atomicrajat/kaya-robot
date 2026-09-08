"""Write a nav_msgs/OccupancyGrid to the standard ROS map format (PGM + YAML).

nav2's map_saver_cli refuses these topics -- it expects a LATCHED
(transient_local) map publisher and nvblox publishes volatile, so it dies with
"Failed to spin map subscription" even though the topic is publishing happily
at ~4.9 Hz. Subscribing with matching QoS and writing the two files ourselves
is simpler than fighting that, and the output is the same standard format any
ROS tool or image viewer will open.

    python3 save_2d_map.py /nvblox_node/static_occupancy_grid out/place_2d
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

# Standard map_server pixel values.
FREE, OCCUPIED, UNKNOWN = 254, 0, 205


class Saver(Node):
    def __init__(self, topic, stem):
        super().__init__('save_2d_map')
        self.stem = stem
        self.done = False
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(OccupancyGrid, topic, self.cb, qos)
        self.tries = 0
        self.create_timer(0.5, self.tick)

    def tick(self):
        self.tries += 1
        if not self.done and self.tries > 40:
            print('no map received on that topic')
            rclpy.shutdown()

    def cb(self, m):
        if self.done:
            return
        self.done = True
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        # OccupancyGrid row 0 is the BOTTOM (lowest y); PGM is written
        # top-down, so rows are emitted in reverse.
        rows = []
        counts = {'free': 0, 'occ': 0, 'unk': 0}
        for iy in range(h - 1, -1, -1):
            row = bytearray(w)
            base = iy * w
            for ix in range(w):
                v = m.data[base + ix]
                if v < 0:
                    row[ix] = UNKNOWN
                    counts['unk'] += 1
                elif v >= 65:
                    row[ix] = OCCUPIED
                    counts['occ'] += 1
                else:
                    row[ix] = FREE
                    counts['free'] += 1
            rows.append(bytes(row))

        pgm = self.stem + '.pgm'
        with open(pgm, 'wb') as f:
            f.write(b'P5\n')
            f.write(b'# saved from %s\n' % self.get_name().encode())
            f.write(b'%d %d\n255\n' % (w, h))
            for r in rows:
                f.write(r)

        with open(self.stem + '.yaml', 'w') as f:
            f.write('image: %s\n' % pgm.split('/')[-1])
            f.write('mode: trinary\n')
            f.write('resolution: %.6f\n' % res)
            f.write('origin: [%.6f, %.6f, 0.000000]\n' % (ox, oy))
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.25\n')

        print('wrote %s (%d x %d @ %.3f m/px = %.1f x %.1f m)'
              % (pgm, w, h, res, w * res, h * res))
        print('  free=%d  occupied=%d  unknown=%d' % (
            counts['free'], counts['occ'], counts['unk']))
        print('wrote %s.yaml' % self.stem)
        rclpy.shutdown()


rclpy.init()
try:
    rclpy.spin(Saver(sys.argv[1], sys.argv[2]))
except Exception:
    pass
