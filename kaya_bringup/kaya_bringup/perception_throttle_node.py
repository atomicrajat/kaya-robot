"""Republish the camera at a low rate, so off-board perception cannot starve the robot.

WHY THIS EXISTS
kaya_follower runs on the workstation and subscribes to the robot's camera
across wifi. Even using the COMPRESSED topics that pulled enough bandwidth to
starve the robot's own pipeline: collision_monitor began rejecting its only
source with

    [depth_cloud]: Latest source and current collision monitor node
    timestamps differ on 6.67 seconds. Ignoring the source.

thousands of times in a run, against ZERO in every run without the follower.
collision_monitor's documented response to losing its source is to stop the
robot, and TF fell far enough behind that RotationShimController could not
transform a pose into the base frame either. The follower tracked its target
perfectly, published correct goals, and the wheels never turned. It broke the
robot by watching it.

The fix is to throttle AT THE SOURCE, on the robot, rather than to ask a remote
subscriber to be polite -- DDS gives the subscriber no way to be. Perception
does not need the full rate anyway: the roadmap's own rule is detection at
1-2 Hz, because a room does not change fast, and OWLv2 takes 35 ms so it is not
the bottleneck either.

At 2 Hz this is about 360 KB/s across the link instead of tens of MB/s.

    /camera/left_ir/image_raw/compressed      ->  /kaya/perception/ir
    /camera/color/image_raw/compressed        ->  /kaya/perception/color
    /camera/depth/image_raw/compressedDepth   ->  /kaya/perception/depth
    /camera/color/camera_info                 ->  /kaya/perception/camera_info

The camera_info is republished alongside so a remote consumer needs nothing
from the full-rate topics at all.

WHY THE IR STREAM IS HERE
It is the dashboard's live view when colour is off, and colour is off whenever
the robot is navigating. MEASURED, colour at only 5 fps starves the pair
cuVSLAM runs on:

    colour off     left_ir 28.4 Hz   map -> odom stamp lag 0.425 s
    colour 5 fps   left_ir 12.9 Hz   map -> odom stamp lag 0.539 s

and a late map -> odom is what makes Nav2 discard the goal and report success
without arriving. The IR stream is ALREADY RUNNING for cuVSLAM, so showing it
costs the camera nothing. Greyscale, but it is genuinely what the robot sees --
and the alternative was a live view that breaks navigation to exist.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage

SENSOR = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT)


class PerceptionThrottle(Node):

    def __init__(self):
        super().__init__('perception_throttle')
        self.declare_parameter('rate_hz', 2.0)
        rate = float(self.get_parameter('rate_hz').value)
        self.period = 1.0 / rate if rate > 0 else 0.5

        self.latest = {'color': None, 'depth': None, 'info': None, 'ir': None}
        self.sent = {'color': 0.0, 'depth': 0.0, 'info': 0.0, 'ir': 0.0}

        self.pub = {
            'color': self.create_publisher(CompressedImage,
                                           '/kaya/perception/color', SENSOR),
            'depth': self.create_publisher(CompressedImage,
                                           '/kaya/perception/depth', SENSOR),
            'info': self.create_publisher(CameraInfo,
                                          '/kaya/perception/camera_info', SENSOR),
            'ir': self.create_publisher(CompressedImage,
                                        '/kaya/perception/ir', SENSOR),
        }
        self.create_subscription(CompressedImage,
                                 '/camera/color/image_raw/compressed',
                                 self._keep('color'), SENSOR)
        self.create_subscription(CompressedImage,
                                 '/camera/depth/image_raw/compressedDepth',
                                 self._keep('depth'), SENSOR)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._keep('info'), SENSOR)
        self.create_subscription(CompressedImage,
                                 '/camera/left_ir/image_raw/compressed',
                                 self._keep('ir'), SENSOR)
        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            'republishing camera on /kaya/perception/* at %.1f Hz' % rate)

    def _keep(self, key):
        # Only the newest frame matters; anything older is already stale by the
        # time a remote consumer would see it.
        def cb(msg):
            self.latest[key] = msg
        return cb

    def tick(self):
        for key, msg in self.latest.items():
            if msg is not None:
                self.pub[key].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionThrottle()
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
