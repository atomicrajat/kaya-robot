"""Kaya web dashboard: live map, live robot position, click to navigate.

RUNS ON THE WORKSTATION, NOT THE ROBOT
Both machines run ROS 2 Jazzy on ROS_DOMAIN_ID 0 on the same subnet, so the
workstation sees all 151 of the robot's topics directly over DDS -- verified.
Everything here therefore costs the Jetson nothing beyond the DDS traffic it
already publishes, which is the design rule the roadmap keeps returning to:
navigation is real-time and must stay so, and the semantic layer must not
compete with it.

    source /opt/ros/jazzy/setup.bash
    python3 dashboard/kaya_dashboard.py        # then open http://localhost:8080

WHY NOT FOXGLOVE
foxglove_bridge is wired into nav.launch.py (foxglove:=true) and works for TF
and images, but Foxglove Studio 3.0.0 cannot render the map: it reports
"Failed to parse channel schema ... Unsupported message encoding" for every
nvblox topic -- static_map_slice, combined_map_slice, mesh, tsdf_layer,
color_layer -- and for /visual_slam/status. Those carry custom nvblox_msgs
types. So the map was unrenderable there whatever the panel settings, which is
what sent this project to a custom dashboard earlier than the roadmap planned.

WHICH MAP THIS SHOWS, AND WHY IT IS TWO LAYERS
nvblox_msgs is not installed on the workstation, so the custom slice topics
cannot be subscribed to from here at all. Both layers below are plain
nav_msgs/OccupancyGrid or a file, and neither needs anything Isaac-specific:

  BASE   the saved 2D grid from the mapping run, 312x369 at 0.05 m. This is the
         whole 202 m^2 and it never changes, so it is loaded once from disk.
  LIVE   /global_costmap/costmap, a 400x400 rolling 20 m window around the
         robot. This is what Nav2 is actually planning on right now, including
         obstacles that were not there when the map was built.

Showing only the live layer would be misleading -- it is a 20 m window, so the
rest of the building simply would not exist. Showing only the base layer would
be worse: it is what the world looked like in the afternoon, and a person
standing in a doorway would be invisible.
"""

import json
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO

from PIL import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
import tf2_ros

# The Isaac types live only on the robot -- neither isaac_ros_visual_slam_
# interfaces nor nvblox_msgs is installed on the workstation. Rather than
# reporting a health number derived from something else and calling it VIO,
# the dashboard says the figure is unavailable. A dashboard that invents a
# green light is the exact failure this project keeps having to undo.
try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    HAVE_VSLAM_MSGS = True
except ImportError:
    VisualSlamStatus = None
    HAVE_VSLAM_MSGS = False

HERE = os.path.dirname(os.path.abspath(__file__))
# The saved map lives outside the repo -- it is 719 MB of map store, not source.
BASE_MAP = os.path.expanduser(
    '~/work_projects/kaya_robot/robot_maps/home_final/home_2d')
PORT = int(os.environ.get('KAYA_DASH_PORT', '8080'))

# Latched publishers (costmaps, the anchor verdict) are TRANSIENT_LOCAL; asking
# for VOLATILE here would connect to nothing and show an empty dashboard while
# reporting no error at all.
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def load_base_map(stem):
    """Load the saved .pgm/.yaml pair written by tools/save_2d_map.py."""
    try:
        meta = {}
        with open(stem + '.yaml') as f:
            for line in f:
                if ':' not in line:
                    continue
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
        origin = json.loads(meta['origin'].replace("'", '"'))
        img = Image.open(stem + '.pgm')
        buf = BytesIO()
        # The browser draws it; PNG so it can be an <img> without a decoder.
        img.convert('L').save(buf, format='PNG')
        return {
            'png': buf.getvalue(),
            'width': img.size[0], 'height': img.size[1],
            'resolution': float(meta['resolution']),
            'origin_x': float(origin[0]), 'origin_y': float(origin[1]),
        }
    except Exception as e:                            # noqa: BLE001
        print('base map not loaded (%s): %s' % (stem, e))
        return None


class Dash(Node):

    def __init__(self):
        super().__init__('kaya_dashboard')
        self.lock = threading.Lock()
        self.costmap = None
        self.costmap_png = None
        self.vio = None
        self.anchored = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_costmap, LATCHED)
        if HAVE_VSLAM_MSGS:
            self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                     self.on_vio, 10)
        self.create_subscription(Bool, '/cuvslam_origin_anchor/anchored',
                                 self.on_anchored, LATCHED)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.base = load_base_map(BASE_MAP)

    # ------------------------------------------------------------ ROS inputs

    def on_costmap(self, m):
        a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        # OccupancyGrid is -1 unknown, 0 free, 100 occupied. Render it as
        # RGBA so unknown can be transparent and the base map show through --
        # that distinction is the whole point of having two layers.
        rgba = np.zeros((m.info.height, m.info.width, 4), dtype=np.uint8)
        occ = a >= 65
        free = (a >= 0) & (a < 65)
        rgba[occ] = (239, 83, 80, 255)
        rgba[free] = (38, 50, 56, 90)
        buf = BytesIO()
        # Row 0 of an OccupancyGrid is the BOTTOM row in map coordinates, so it
        # is flipped here rather than in the browser, where it would have to be
        # undone again for every click.
        Image.fromarray(rgba[::-1], 'RGBA').save(buf, format='PNG')
        with self.lock:
            self.costmap_png = buf.getvalue()
            self.costmap = {
                'width': m.info.width, 'height': m.info.height,
                'resolution': m.info.resolution,
                'origin_x': m.info.origin.position.x,
                'origin_y': m.info.origin.position.y,
            }

    def on_vio(self, m):
        with self.lock:
            self.vio = {'vo_state': int(m.vo_state),
                        'track_ms': m.track_execution_time_mean * 1000.0}

    def on_anchored(self, m):
        with self.lock:
            self.anchored = bool(m.data)

    # --------------------------------------------------------------- queries

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except tf2_ros.TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return {'x': tr.x, 'y': tr.y, 'z': tr.z, 'yaw': yaw}

    def send_goal(self, x, y, yaw):
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        g.pose.orientation.z = math.sin(float(yaw) / 2.0)
        g.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self.goal_pub.publish(g)
        self.get_logger().info('goal published: (%.2f, %.2f) yaw %.0f deg'
                               % (x, y, math.degrees(yaw)))


def make_handler(node):

    class H(BaseHTTPRequestHandler):

        def log_message(self, *a):
            pass                                      # quiet; ROS logs instead

        def _send(self, code, body, ctype, cache=False):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            if not cache:
                self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path in ('/', '/index.html'):
                with open(os.path.join(HERE, 'index.html'), 'rb') as f:
                    self._send(200, f.read(), 'text/html; charset=utf-8')
            elif path == '/api/state':
                with node.lock:
                    body = {
                        'pose': node.pose(),
                        'vio': node.vio,
                        'vio_available': HAVE_VSLAM_MSGS,
                        'anchored': node.anchored,
                        'costmap': node.costmap,
                        'base': ({k: v for k, v in node.base.items() if k != 'png'}
                                 if node.base else None),
                    }
                self._send(200, json.dumps(body).encode(), 'application/json')
            elif path == '/api/base.png' and node.base:
                self._send(200, node.base['png'], 'image/png', cache=True)
            elif path == '/api/costmap.png':
                with node.lock:
                    png = node.costmap_png
                if png is None:
                    self._send(404, b'no costmap yet', 'text/plain')
                else:
                    self._send(200, png, 'image/png')
            else:
                self._send(404, b'not found', 'text/plain')

        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0))
            req = json.loads(self.rfile.read(n) or b'{}')
            if self.path == '/api/goal':
                node.send_goal(req['x'], req['y'], req.get('yaw', 0.0))
                self._send(200, json.dumps({'ok': True}).encode(),
                           'application/json')
            elif self.path == '/api/ask':
                # Phase 10 lands here: shortlist keyframes for the question,
                # let a reasoning model pick one, send its pose as a goal.
                # Deliberately not faked -- an endpoint that invents an answer
                # is worse than one that says it is not built.
                self._send(501, json.dumps({
                    'error': 'Phase 10 not built yet. This endpoint is where '
                             'the question will be answered: keyframes are '
                             'Phase 9, the reasoning model is Phase 10.',
                }).encode(), 'application/json')
            else:
                self._send(404, b'not found', 'text/plain')

    return H


def main():
    rclpy.init()
    node = Dash()
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), make_handler(node))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info('dashboard on http://localhost:%d  '
                           '(base map %s)'
                           % (PORT, 'loaded' if node.base else 'MISSING'))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                             # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
