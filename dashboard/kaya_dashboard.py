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
import sys
import urllib.parse
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO

from PIL import Image
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import MarkerArray
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
import tf2_ros

# perception/ sits beside dashboard/ in the repo, not on the path.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, 'perception'))
# Data that is not source (keyframes, the 719 MB map store) lives in the
# workspace folder that CONTAINS this repo - ~/work_projects/kaya_robot on the
# dev machine. KAYA_WORKSPACE overrides it on any other machine.
WORKSPACE = os.environ.get('KAYA_WORKSPACE', os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir)))
KEYFRAME_DIR = os.path.join(WORKSPACE, 'keyframes')

try:
    import scene_memory
except Exception as _e:                                   # noqa: BLE001
    scene_memory = None
    print('scene_memory unavailable (%s); /api/ask will report why' % _e)

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
# KAYA_BASE_MAP: the 2D grid's path stem (the .pgm/.yaml pair without the extension),
# as written by kaya_save_map. Default: the project's own map in the workspace folder.
BASE_MAP = os.environ.get('KAYA_BASE_MAP',
                          os.path.join(WORKSPACE, 'robot_maps', 'home_final', 'home_2d'))
PORT = int(os.environ.get('KAYA_DASH_PORT', '8080'))
# Beyond this the robot is treated as gone, not merely quiet.
STALE_S = 5.0

# Latched publishers (costmaps, the anchor verdict) are TRANSIENT_LOCAL; asking
# for VOLATILE here would connect to nothing and show an empty dashboard while
# reporting no error at all.
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Sensor streams are published BEST_EFFORT; a RELIABLE subscription silently
# matches nothing and the panel stays empty with no error anywhere.
SENSOR = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT)

# Large, fragmented messages need reliability to survive the wire.
BULK = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                  reliability=ReliabilityPolicy.RELIABLE)

# Paths are drawn, not analysed, so a few hundred points is plenty and keeps
# the JSON small enough to poll twice a second.
MAX_PATH_PTS = 300
# The rollout cloud is ~7.6k states; this many reads the same at screen size.
MAX_CANDIDATE_PTS = 700
# A plan that stopped arriving is not a plan; Nav2 sends no empty Path on abort.
PLAN_STALE_S = 2.0
# A held teleop command expires this fast. The browser re-sends while a key or
# button is down; if it stops for any reason the robot stops with it.
TELEOP_TIMEOUT_S = 0.5
# Well under the 0.15 m/s Nav2 uses: a human driving from a camera view has far
# less situational awareness than the planner does.
TELEOP_VX = 0.10
TELEOP_WZ = 0.4


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
        self.plan = None
        self.plan_t = 0.0
        self.optimal = None
        self.optimal_t = 0.0
        self.candidates = None
        self.candidates_t = 0.0
        self.jpeg = None
        self.jpeg_t = 0.0
        self.jpeg_src = None
        self.jetson = None
        self.track = None
        self.manual = False
        self.teleop_t = 0.0
        self.teleop_cmd = (0.0, 0.0, 0.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_costmap, LATCHED)
        if HAVE_VSLAM_MSGS:
            self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                     self.on_vio, 10)
        self.create_subscription(Bool, '/cuvslam_origin_anchor/anchored',
                                 self.on_anchored, LATCHED)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(Path, '/optimal_trajectory', self.on_optimal, 10)
        # MPPI's candidate rollouts -- the cloud that makes the planner legible.
        # RELIABLE on purpose, unlike the camera: one message carries ~7.6k
        # markers, which fragments into many UDP datagrams, and BEST_EFFORT
        # drops the entire sample if any one fragment is lost. Subscribed
        # best-effort it delivered NOTHING across a whole drive while a
        # reliable probe on the same topic received every message.
        self.create_subscription(MarkerArray, '/trajectories',
                                 self.on_candidates, BULK)
        # Colour when it is running, IR otherwise. Colour is OFF whenever the
        # robot is navigating -- at 5 fps it halves the IR rate cuVSLAM needs
        # -- so IR is the normal live view, not the fallback.
        self.create_subscription(CompressedImage,
                                 '/camera/color/image_raw/compressed',
                                 self.on_jpeg, SENSOR)
        self.create_subscription(CompressedImage, '/kaya/perception/ir',
                                 self.on_ir, SENSOR)
        self.create_subscription(String, '/kaya/jetson_stats',
                                 self.on_jetson, 10)
        self.create_subscription(String, '/kaya/track_status', self.on_track, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # Naming a target here is what starts following; the follower node
        # does nothing until it receives one.
        self.target_pub = self.create_publisher(String, '/kaya/track_target', 10)
        # Teleop publishes into cmd_vel_smoothed, NOT cmd_vel, so
        # collision_monitor still sits between the operator and the wheels.
        # Driving by hand is exactly when a last-resort safety layer earns its
        # place, and this project already has one configured.
        self.teleop_pub = self.create_publisher(Twist, '/cmd_vel_smoothed', 10)
        # DEADMAN: a browser that closes, freezes or loses wifi must not leave
        # the robot driving.
        self.create_timer(0.1, self.teleop_tick)
        self.base = load_base_map(BASE_MAP)

    # ------------------------------------------------------------ ROS inputs

    def on_costmap(self, m):
        a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        # OccupancyGrid is -1 unknown, 0 free, 100 occupied. Render it as
        # RGBA so unknown can be transparent and the base map show through --
        # that distinction is the whole point of having two layers.
        rgba = np.zeros((m.info.height, m.info.width, 4), dtype=np.uint8)
        # Four bands, not two. An earlier version painted everything >= 65 as
        # solid red and every other KNOWN cell as a dark fill -- which was
        # nearly invisible while the costmap held only the camera cone, and
        # became a red blob hiding the base map entirely once the static layer
        # made the whole 202 m^2 known. Lethal and "inflated but driveable"
        # are different things and must not look the same.
        lethal = a >= 99                       # inscribed: the robot cannot be here
        near = (a >= 50) & (a < 99)            # graded cost: driveable, discouraged
        low = (a > 0) & (a < 50)               # barely any cost
        rgba[lethal] = (239, 83, 80, 210)
        rgba[near] = (255, 167, 38, 70)
        rgba[low] = (79, 195, 247, 26)
        # cost 0 and unknown stay fully transparent so the saved map shows
        # through -- that is the layer that makes the room legible.
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

    @staticmethod
    def _thin(pts):
        step = max(1, len(pts) // MAX_PATH_PTS)
        return [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
                for p in pts[::step]]

    def on_plan(self, m):
        with self.lock:
            self.plan = self._thin(m.poses)
            self.plan_t = time.time()

    def on_optimal(self, m):
        with self.lock:
            self.optimal = self._thin(m.poses)
            self.optimal_t = time.time()

    def on_candidates(self, m):
        """MPPI's candidate rollouts, as a cloud of sampled states.

        Nav2's TrajectoryVisualizer does NOT publish one LINE_STRIP per
        trajectory. It publishes one SPHERE marker per sampled state, with the
        position in marker.pose.position and points[] EMPTY -- measured here at
        7656 markers in a single message. A first version filtered for
        len(points) >= 2 and silently dropped every one of them.

        They are drawn as a cloud rather than reassembled into curves: the
        markers carry no trajectory id, so grouping them means guessing at run
        lengths, and a wrong guess would draw confident lines through states
        that belong to different rollouts.
        """
        pts = [mk.pose.position for mk in m.markers if mk.action == 0]
        step = max(1, len(pts) // MAX_CANDIDATE_PTS)
        out = [[round(p.x, 3), round(p.y, 3)] for p in pts[::step]]
        with self.lock:
            self.candidates = out or None
            self.candidates_t = time.time()

    def on_jpeg(self, m):
        # Throttled: the browser polls at a few Hz, so keeping every frame off
        # the wire to it costs nothing and keeps this cheap.
        now = time.time()
        if now - self.jpeg_t < 0.15:
            return
        with self.lock:
            self.jpeg = bytes(m.data)
            self.jpeg_t = now
            self.jpeg_src = 'colour'

    def on_ir(self, m):
        # IR only fills in when colour is not arriving. Colour is the better
        # picture; IR is the one that is always there, because it is what
        # cuVSLAM runs on and it is never turned off.
        now = time.time()
        if now - self.jpeg_t < 0.4:
            return
        with self.lock:
            self.jpeg = bytes(m.data)
            self.jpeg_t = now
            self.jpeg_src = 'infrared'

    def teleop_tick(self):
        if not self.manual:
            return
        vx, vy, wz = self.teleop_cmd
        if time.time() - self.teleop_t > TELEOP_TIMEOUT_S:
            vx = vy = wz = 0.0
            self.teleop_cmd = (0.0, 0.0, 0.0)
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        self.teleop_pub.publish(t)

    def cancel_navigation(self):
        """Stop Nav2 through its own action, not by fighting for /cmd_vel."""
        try:
            from action_msgs.srv import CancelGoal
            cli = self.create_client(CancelGoal,
                                     '/navigate_to_pose/_action/cancel_goal')
            if cli.service_is_ready():
                cli.call_async(CancelGoal.Request())
        except Exception as e:                        # noqa: BLE001
            self.get_logger().warn('could not cancel navigation: %r' % e)

    def fresh(self, value, stamp, max_age=None):
        """Return a value only while it is still arriving.

        Nav2 publishes no empty Path when a goal aborts, so a plan that
        stopped coming stays in hand forever. Drawn unaged it animated a dead
        route while the robot sat still against a wall, which reads as "still
        going". Everything the robot streams is aged.
        """
        if value is None or time.time() - stamp > (max_age or PLAN_STALE_S):
            return None
        return value

    def on_track(self, m):
        try:
            with self.lock:
                self.track = json.loads(m.data)
                self.track['t'] = time.time()
        except ValueError:
            pass

    def on_jetson(self, m):
        try:
            with self.lock:
                self.jetson = json.loads(m.data)
        except ValueError:
            pass

    def on_vio(self, m):
        with self.lock:
            self.vio = {'vo_state': int(m.vo_state),
                        'track_ms': m.track_execution_time_mean * 1000.0}

    def on_anchored(self, m):
        with self.lock:
            self.anchored = bool(m.data)

    # --------------------------------------------------------------- queries

    def pose(self):
        """Pose AND how old it is.

        A TF buffer keeps handing back the last transform it saw, forever. When
        the robot's battery died mid-session this dashboard went on reporting a
        confident pose and "map frame anchored" for a machine that had been off
        the network for minutes -- the same trap as a tracker reporting
        vo_state=1 while drifting metres. Nothing here may be shown without its
        age beside it.
        """
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except tf2_ros.TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = Time.from_msg(t.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return {'x': tr.x, 'y': tr.y, 'z': tr.z, 'yaw': yaw, 'age_s': age}

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
                    pose = node.pose()
                    # Anything the robot published is only as good as the last
                    # time it published anything at all.
                    age = pose['age_s'] if pose else None
                    link = ('offline' if age is None or age > STALE_S
                            else 'stale' if age > 1.5 else 'live')
                    body = {
                        'pose': pose,
                        'link': link,
                        'stale_after_s': STALE_S,
                        'vio': node.vio,
                        'vio_available': HAVE_VSLAM_MSGS,
                        # A goal published to nobody goes nowhere and reports
                        # no error, which is exactly what it looks like when
                        # the robot has quietly gone away.
                        'goal_subscribers': node.goal_pub.get_subscription_count(),
                        'camera_source': node.jpeg_src,
                        'anchored': node.anchored if link != 'offline' else None,
                        # Nav2 does not publish an empty Path when a goal is
                        # aborted, so a plan that stopped arriving stays in
                        # hand forever. Drawn unaged, the overlay animated a
                        # dead plan along a route the robot had already given
                        # up on -- which reads as "still going" while it sits
                        # against a wall. Age everything the robot streams.
                        'plan': node.fresh(node.plan, node.plan_t),
                        'optimal': node.fresh(node.optimal, node.optimal_t),
                        'candidates': node.fresh(node.candidates,
                                                 node.candidates_t),
                        'navigating': (time.time() - node.optimal_t) < PLAN_STALE_S,
                        'jetson': node.jetson if link != 'offline' else None,
                        'track': node.fresh(node.track,
                                            (node.track or {}).get('t', 0), 3.0),
                        'follower': node.target_pub.get_subscription_count() > 0,
                        'manual': node.manual,
                        'camera': (node.jpeg is not None
                                   and time.time() - node.jpeg_t < 3.0),
                        'costmap': node.costmap,
                        'base': ({k: v for k, v in node.base.items() if k != 'png'}
                                 if node.base else None),
                    }
                self._send(200, json.dumps(body).encode(), 'application/json')
            elif path == '/api/base.png' and node.base:
                self._send(200, node.base['png'], 'image/png', cache=True)
            elif path.startswith('/static/'):
                # three.js, its loaders and the robot GLB are vendored rather
                # than pulled from a CDN, so the dashboard still works when the
                # robot is on a network with no route out.
                name = os.path.basename(path)
                f = os.path.join(HERE, 'static', name)
                if not os.path.isfile(f):
                    self._send(404, b'not found', 'text/plain')
                    return
                ctype = ('model/gltf-binary' if name.endswith('.glb')
                         else 'application/javascript' if name.endswith('.js')
                         else 'application/octet-stream')
                with open(f, 'rb') as fh:
                    self._send(200, fh.read(), ctype, cache=True)
            elif path == '/api/camera.jpg':
                with node.lock:
                    j = node.jpeg
                if j is None:
                    self._send(404, b'no frame', 'text/plain')
                else:
                    self._send(200, j, 'image/jpeg')
            elif path == '/api/costmap.png':
                with node.lock:
                    png = node.costmap_png
                if png is None:
                    self._send(404, b'no costmap yet', 'text/plain')
                else:
                    self._send(200, png, 'image/png')
            elif self.path.startswith('/api/keyframe.jpg'):
                q = urllib.parse.parse_qs(
                    urllib.parse.urlparse(self.path).query)
                name = (q.get('file') or [''])[0]
                # basename only: this serves a directory to a browser, and
                # '../../etc/passwd' is a file name too.
                name = os.path.basename(name)
                path = os.path.join(KEYFRAME_DIR, 'images', name)
                if not name or not os.path.isfile(path):
                    self._send(404, b'no such keyframe', 'text/plain')
                    return
                with open(path, 'rb') as f:
                    self._send(200, f.read(), 'image/jpeg')
            else:
                self._send(404, b'not found', 'text/plain')

        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0))
            req = json.loads(self.rfile.read(n) or b'{}')
            if self.path == '/api/goal':
                n_sub = node.goal_pub.get_subscription_count()
                if n_sub == 0:
                    # Publishing anyway would look identical to success.
                    self._send(503, json.dumps({
                        'ok': False,
                        'error': 'nothing is subscribed to /goal_pose. Nav2 is '
                                 'not running, or the robot is off the network.',
                    }).encode(), 'application/json')
                    return
                node.send_goal(req['x'], req['y'], req.get('yaw', 0.0))
                self._send(200, json.dumps({'ok': True}).encode(),
                           'application/json')
            elif self.path == '/api/mode':
                want = bool(req.get('manual'))
                node.manual = want
                node.teleop_cmd = (0.0, 0.0, 0.0)
                node.teleop_t = 0.0
                if want:
                    # Entering manual cancels whatever Nav2 is doing, so the
                    # planner and the operator never drive at the same time.
                    node.cancel_navigation()
                node.get_logger().info(
                    'manual control %s' % ('ENGAGED' if want else 'released'))
                self._send(200, json.dumps({'ok': True, 'manual': want}).encode(),
                           'application/json')
            elif self.path == '/api/teleop':
                if not node.manual:
                    self._send(409, json.dumps({
                        'ok': False, 'error': 'not in manual mode'}).encode(),
                        'application/json')
                    return
                node.teleop_cmd = (
                    max(-1.0, min(1.0, float(req.get('vx', 0.0)))) * TELEOP_VX,
                    max(-1.0, min(1.0, float(req.get('vy', 0.0)))) * TELEOP_VX,
                    max(-1.0, min(1.0, float(req.get('wz', 0.0)))) * TELEOP_WZ)
                node.teleop_t = time.time()
                self._send(200, json.dumps({'ok': True}).encode(),
                           'application/json')
            elif self.path == '/api/track':
                if node.target_pub.get_subscription_count() == 0:
                    self._send(503, json.dumps({
                        'ok': False,
                        'error': 'the follower node is not running. Start '
                                 'perception/kaya_follower.py on the '
                                 'workstation.',
                    }).encode(), 'application/json')
                    return
                t = (req.get('target') or '').strip()
                node.target_pub.publish(String(data=t))
                self._send(200, json.dumps({'ok': True, 'target': t}).encode(),
                           'application/json')
            elif self.path == '/api/ask':
                if scene_memory is None:
                    self._send(501, json.dumps({
                        'error': 'perception/scene_memory.py could not be '
                                 'imported, so questions cannot be answered.',
                    }).encode(), 'application/json')
                    return
                q = (req.get('question') or '').strip()
                if not q:
                    self._send(400, json.dumps({'error': 'no question'}).encode(),
                               'application/json')
                    return
                r = scene_memory.ask(q)
                # ANSWERING AND DRIVING ARE SEPARATE. The question returns a
                # place and the photo the robot remembers of it; the operator
                # then presses Go. A model that is confidently wrong about a
                # room should cost a glance at a picture, not a drive across
                # the flat -- and this store already contains rooms captioned
                # from a single doorway glimpse.
                node.get_logger().info(
                    'asked "%s" -> %s' % (q, r.get('why', r.get('error', ''))))
                self._send(200 if r.get('ok') else 503,
                           json.dumps(r).encode(), 'application/json')
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
