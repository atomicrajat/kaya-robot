"""Load a saved .nvblx volume into a running nvblox_node, once it is ready.

WHY THIS EXISTS
A saved map in this project is a DIRECTORY, and it holds two independent things
that are easy to mistake for one:

    home_final/cuvslam/data.mdb   cuVSLAM's relocalisation database -- where
                                  the robot IS. This is what map_path loads.
    home_final/home.nvblx         nvblox's TSDF/ESDF volume -- what the SPACE
                                  looks like. Nothing loaded this.

So `nav.launch.py map_path:=.../home_final/cuvslam` relocalised correctly into
the saved coordinate system and then rebuilt the room from scratch off the live
camera, because that is all nvblox had. On a stationary robot that means a
costmap consisting of the cone the camera can currently see -- measured at 371
of 391 cells in the corridor ahead, and nothing at all behind. Reloading the
.nvblx took the same corridor to 391 of 391 and restored the whole 202 m^2.

The failure is quiet in the worst way: the robot knows exactly where it is, on a
map that has forgotten the building. Planning works, and plans through walls it
has not seen yet.

WHY A NODE AND NOT A TIMER
nvblox allocates its voxel layers on the GPU at startup and that takes roughly
two minutes on this board -- measured, not estimated. Its services do not exist
before then, and the delay is not predictable enough to hard-code, which is the
same mistake origin_anchor_node.py documents having made with a TimerAction. So
this waits for the service to appear rather than guessing when it will.

ON TRUSTING THE RESPONSE
`load_map` is SYNCHRONOUS -- the call returns after the work is done, measured
at 11 s for a 101 MB volume, and its success flag is the real outcome. That is
unlike cuVSLAM's `LocalizeInMap`, which returns the moment the request is
accepted and reports the actual result only to its log; see the long note in
origin_anchor_node.py. The two are not the same shape and must not be treated
the same way.
"""

from nvblox_msgs.srv import FilePath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Bool


class NvbloxMapLoader(Node):

    def __init__(self):
        super().__init__('nvblox_map_loader')
        self.declare_parameter('nvblox_map', '')
        # nvblox needs ~2 minutes to allocate its layers before its services
        # appear; this is the ceiling on waiting for that, not a target.
        self.declare_parameter('timeout_s', 240.0)

        self.map_file = self.get_parameter('nvblox_map').value
        self.timeout_s = self.get_parameter('timeout_s').value
        self.t0 = self.get_clock().now()
        self.done = False

        self.loaded_pub = self.create_publisher(
            Bool, '~/loaded',
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        if not self.map_file:
            self.get_logger().info(
                'no nvblox_map given; nvblox will build its map from live '
                'depth only, and will not know anything the camera has not '
                'seen this run')
            self.done = True
            return

        self.cli = self.create_client(FilePath, '/nvblox_node/load_map')
        self.get_logger().info(
            'waiting for /nvblox_node/load_map, then will load %s '
            '(nvblox takes ~2 min to allocate its layers first)' % self.map_file)
        self.create_timer(2.0, self.tick)

    def tick(self):
        if self.done:
            return
        if self.cli.service_is_ready():
            self.done = True
            req = FilePath.Request()
            req.file_path = self.map_file
            self.cli.call_async(req).add_done_callback(self.on_done)
            return
        el = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        if el > self.timeout_s:
            self.done = True
            self.fail('/nvblox_node/load_map never appeared within %.0f s'
                      % self.timeout_s)

    def on_done(self, fut):
        try:
            ok = fut.result().success
        except Exception as e:                        # noqa: BLE001
            self.fail('call failed: %r' % e)
            return
        if not ok:
            self.fail('nvblox refused %s. Check the file exists and was '
                      'written by a compatible nvblox version.' % self.map_file)
            return
        self.get_logger().info(
            'loaded %s; the costmap now holds the whole saved space, not just '
            'what the camera has seen this run' % self.map_file)
        self.loaded_pub.publish(Bool(data=True))

    def fail(self, why):
        self.get_logger().error(
            'nvblox map NOT loaded: %s. The robot will still localize, but it '
            'will plan on a map rebuilt from the live camera alone.' % why)
        self.loaded_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = NvbloxMapLoader()
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
