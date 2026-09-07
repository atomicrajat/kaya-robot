"""Render nvblox's 2D costmap slice around the ROBOT'S ACTUAL POSE.

The earlier version assumed the robot sat at map (0,0). That held while every
test started from the origin, and silently produced nonsense the moment it did
not -- it probed empty space near the origin and reported "0 observed cells",
which looks exactly like a broken costmap. This one looks the pose up from TF.

  #  inside/at an obstacle (d <= 0.05)   +  close (<= 0.15)
  .  free                                   blank = unobserved
  R  robot
"""
import math
import sys

import rclpy
from rclpy.node import Node
from nvblox_msgs.msg import DistanceMapSlice
import tf2_ros


class R(Node):
    def __init__(self):
        super().__init__('slice_render2')
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.msg = None
        self.create_subscription(DistanceMapSlice, '/nvblox_node/static_map_slice',
                                 self.cb, 10)
        self.create_timer(0.5, self.tick)
        self.tries = 0

    def cb(self, m):
        self.msg = m

    def tick(self):
        self.tries += 1
        if self.msg is None:
            if self.tries > 40:
                print('no slice received')
                rclpy.shutdown()
            return
        try:
            t = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            if self.tries > 40:
                print('no TF map->base_footprint')
                rclpy.shutdown()
            return

        m = self.msg
        rx = t.transform.translation.x
        ry = t.transform.translation.y
        rz = t.transform.translation.z
        w, h, res, unk = m.width, m.height, m.resolution, m.unknown_value
        ox, oy = m.origin.x, m.origin.y
        d = list(m.data)

        rcx = int(round((rx - ox) / res))
        rcy = int(round((ry - oy) / res))

        print('slice %dx%d @ %.2f m, origin (%.2f, %.2f, %.3f)'
              % (w, h, res, ox, oy, m.origin.z))
        print('ROBOT at map (%.3f, %.3f, %.3f) -> cell (%d, %d)'
              % (rx, ry, rz, rcx, rcy))
        inside = 0 <= rcx < w and 0 <= rcy < h
        print('robot inside published slice: %s' % ('YES' if inside else 'NO'))
        # The slice is a fixed band in MAP z. If the robot's own z has drifted
        # away from the z the band was chosen for, the band no longer sits over
        # the floor the robot is on -- that is what this line is checking.
        print('slice reported at map z=%.3f; robot z=%.3f -> band sits %.3f m '
              'above the robot\'s own floor plane' % (m.origin.z, rz, m.origin.z - rz))

        def ch(ix, iy):
            if ix == rcx and iy == rcy:
                return 'R'
            if not (0 <= ix < w and 0 <= iy < h):
                return ' '
            v = d[iy * w + ix]
            if v == unk:
                return ' '
            if v <= 0.05:
                return '#'
            if v <= 0.15:
                return '+'
            return '.'

        lo = max(0, rcx - 24)
        hi = min(w, rcx + 25)
        print()
        for ix in range(hi - 1, lo - 1, -1):
            row = ''.join(ch(ix, iy) for iy in range(h - 1, -1, -1))
            mark = '  <-- robot row' if ix == rcx else ''
            print('%+6.2f |%s|%s' % (ox + ix * res, row, mark))

        # Probe the corridor ahead OF THE ROBOT, in its own heading frame.
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        n = obs = 0
        for fwd in [x * 0.05 for x in range(8, 31)]:
            for lat in [y * 0.05 for y in range(-8, 9)]:
                gx = rx + fwd * math.cos(yaw) - lat * math.sin(yaw)
                gy = ry + fwd * math.sin(yaw) + lat * math.cos(yaw)
                ix = int(round((gx - ox) / res))
                iy = int(round((gy - oy) / res))
                if not (0 <= ix < w and 0 <= iy < h):
                    continue
                v = d[iy * w + ix]
                n += 1
                if v != unk:
                    obs += 1
        print('\nCORRIDOR 0.4-1.5 m ahead of the ROBOT (its own heading):')
        print('  %d cells in slice bounds, %d observed (%.1f%%)'
              % (n, obs, 100.0 * obs / n if n else 0.0))
        rclpy.shutdown()


rclpy.init()
try:
    rclpy.spin(R())
except Exception:
    pass
