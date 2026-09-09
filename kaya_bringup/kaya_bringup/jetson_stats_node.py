"""Publish the Jetson's own load, so the dashboard can show what the robot costs.

Reads sysfs rather than shelling out to tegrastats: tegrastats is a streaming
binary that has to be parsed and kept alive, while every number below is a file
read. Measured cost of this node is nothing worth reporting, which matters --
a monitor that perturbs what it measures is useless, and this project has
already been bitten by a "free" addition twice.

Published as JSON on a std_msgs/String so the dashboard, which runs on the
workstation where nvblox_msgs and the Isaac types are NOT installed, can read
it without needing any custom message package.

    ros2 run kaya_bringup jetson_stats

GPU load on Orin is per-mille in /sys/devices/platform/gpu.0/load (0-1000),
not a percentage; reporting it raw would show "537% GPU" and be believed.
"""

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

GPU_LOAD = '/sys/devices/platform/gpu.0/load'
THERMAL = '/sys/devices/virtual/thermal'


def read(path, default=None):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


class JetsonStats(Node):

    def __init__(self):
        super().__init__('jetson_stats')
        self.declare_parameter('rate_hz', 1.0)
        self.pub = self.create_publisher(String, '/kaya/jetson_stats', 10)
        self.prev_cpu = None
        hz = self.get_parameter('rate_hz').value
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info('publishing /kaya/jetson_stats at %.1f Hz' % hz)

    def cpu_percent(self):
        """Whole-CPU busy fraction between calls, from /proc/stat deltas."""
        line = read('/proc/stat', '').split('\n')[0].split()
        if len(line) < 5:
            return None
        vals = [int(v) for v in line[1:]]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        total = sum(vals)
        prev, self.prev_cpu = self.prev_cpu, (idle, total)
        if prev is None:
            return None
        d_idle, d_total = idle - prev[0], total - prev[1]
        if d_total <= 0:
            return None
        return round(100.0 * (1.0 - d_idle / d_total), 1)

    def mem(self):
        info = {}
        for line in read('/proc/meminfo', '').split('\n'):
            if ':' in line:
                k, v = line.split(':', 1)
                info[k.strip()] = v.strip().split()[0]
        try:
            total = int(info['MemTotal']) / 1024.0
            avail = int(info['MemAvailable']) / 1024.0
            return {'used_mb': round(total - avail), 'total_mb': round(total),
                    'percent': round(100.0 * (total - avail) / total, 1)}
        except (KeyError, ValueError, ZeroDivisionError):
            return None

    def temps(self):
        out = {}
        try:
            for zone in sorted(os.listdir(THERMAL)):
                if not zone.startswith('thermal_zone'):
                    continue
                name = read(os.path.join(THERMAL, zone, 'type'))
                raw = read(os.path.join(THERMAL, zone, 'temp'))
                if name and raw and raw.lstrip('-').isdigit():
                    out[name.replace('-thermal', '')] = round(int(raw) / 1000.0, 1)
        except OSError:
            pass
        return out

    def tick(self):
        raw = read(GPU_LOAD)
        # Per-mille, not percent. See the module docstring.
        gpu = round(int(raw) / 10.0, 1) if raw and raw.isdigit() else None
        self.pub.publish(String(data=json.dumps({
            'cpu_percent': self.cpu_percent(),
            'gpu_percent': gpu,
            'mem': self.mem(),
            'temps_c': self.temps(),
        })))


def main(args=None):
    rclpy.init(args=args)
    node = JetsonStats()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                             # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
