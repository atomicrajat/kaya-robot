"""Anchor cuVSLAM's map frame once, after tracking is genuinely healthy.

There are two ways to establish where "map" is, and they are the same job:

  NO SAVED MAP -- reset the tracker, pinning the origin to wherever the robot
      currently stands. Every run starts a fresh coordinate system.

  SAVED MAP    -- localize in it, so the robot adopts the coordinate system it
      used last time. Goals, waypoints and anything else recorded against that
      map stay meaningful across restarts. This is the whole point of saving a
      map, and it is what makes "go to the kitchen" survive a reboot.

Only one of these may happen, and doing the wrong one silently destroys the
other: resetting after loading a map throws away the localization and drops
the robot back at a meaningless (0,0). The launch file therefore drives both
from this single node rather than leaving them as two things to remember.

WHY IT WAITS ON THE TRACKER, NOT A TIMER
An earlier version was a launch TimerAction with a fixed 12 s delay, and that
was wrong. The delay ran from launch start rather than tracker readiness, and
those times are not predictable: the component container has taken 14 s to
instantiate the node, camera init has ranged 1.6-10 s, and in one run the
tracker did not report ready until 67 s after launch. The timer fired during
initialisation and the tracker never came up at all. So this waits for the
condition that actually matters -- healthy tracking, sustained for settle_s --
and gives up loudly rather than acting on a half-initialised tracker.

WHY THE ORIGIN NEEDS PINNING AT ALL
cuVSLAM's origin is wherever the tracker believed it was when it initialised,
which is when it is least trustworthy. Measured on this robot, stationary on
the floor throughout: by the time tracking looked healthy, map -> base_footprint
read (0.285, -0.061, -0.360) m with -9.0 deg pitch. The -0.36 m in Z is the
tell -- the robot cannot descend 36 cm, so that pose was integrated, not
travelled. It is not drift: once settled the same tracker held 0.87 cm/min and
0.05 deg/min stationary. Left uncorrected it poisons every nvblox voxel and
every Nav2 goal, since both are expressed in that frame.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from isaac_ros_visual_slam_interfaces.srv import LocalizeInMap, Reset

# vo_state 0 and 1 are both "tracking"; anything else is lost or initialising,
# and acting then is exactly what this node exists to avoid.
TRACKING_STATES = (0, 1)


class OriginAnchor(Node):

    def __init__(self):
        super().__init__('cuvslam_origin_anchor')
        self.declare_parameter('settle_s', 5.0)
        self.declare_parameter('timeout_s', 120.0)
        # Empty means "no saved map": reset instead of localizing.
        self.declare_parameter('map_path', '')
        # Pose hint for localization, map_T_robot. Only a HINT -- verified on
        # hardware that cuVSLAM recovers from a bad one: given a hint 1.8 m
        # away from truth it still converged to within 7 mm. Set it if you
        # know roughly where the robot is starting; leave it at the origin if
        # you do not.
        self.declare_parameter('hint_x', 0.0)
        self.declare_parameter('hint_y', 0.0)
        self.declare_parameter('hint_yaw', 0.0)

        self.settle_s = self.get_parameter('settle_s').value
        self.timeout_s = self.get_parameter('timeout_s').value
        self.map_path = self.get_parameter('map_path').value

        self.healthy_since = None
        self.done = False
        self.t0 = self.get_clock().now()

        if self.map_path:
            self.cli = self.create_client(LocalizeInMap, '/visual_slam/localize_in_map')
            what = 'localize in %s' % self.map_path
        else:
            self.cli = self.create_client(Reset, '/visual_slam/reset')
            what = 're-zero the origin here (no map_path given)'

        self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                 self.on_status, 10)
        self.create_timer(1.0, self.check_timeout)
        self.get_logger().info(
            'waiting for %.1f s of healthy tracking, then will %s'
            % (self.settle_s, what))

    def on_status(self, msg):
        if self.done:
            return
        now = self.get_clock().now()
        if msg.vo_state not in TRACKING_STATES:
            if self.healthy_since is not None:
                self.get_logger().warn(
                    'tracking dropped (vo_state=%d); restarting settle window'
                    % msg.vo_state)
            self.healthy_since = None
            return
        if self.healthy_since is None:
            self.healthy_since = now
            return
        if (now - self.healthy_since).nanoseconds / 1e9 >= self.settle_s:
            self.fire()

    def fire(self):
        self.done = True
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '%s unavailable; map frame NOT anchored' % self.cli.srv_name)
            return
        if self.map_path:
            req = LocalizeInMap.Request()
            req.map_folder_path = self.map_path
            req.pose_hint = self._hint()
        else:
            req = Reset.Request()
        self.cli.call_async(req).add_done_callback(self.on_done)

    def _hint(self):
        import math
        p = Pose()
        p.position.x = self.get_parameter('hint_x').value
        p.position.y = self.get_parameter('hint_y').value
        yaw = self.get_parameter('hint_yaw').value
        p.orientation.z = math.sin(yaw / 2.0)
        p.orientation.w = math.cos(yaw / 2.0)
        return p

    def on_done(self, fut):
        try:
            ok = fut.result().success
        except Exception as e:                        # noqa: BLE001
            self.get_logger().error('call failed: %r' % e)
            return
        if not ok:
            self.get_logger().error(
                'service reported failure; map frame NOT anchored. If '
                'localizing, the robot may not be anywhere in the saved map, '
                'or the scene has changed too much to recognise.')
            return
        if self.map_path:
            self.get_logger().info(
                'localized in %s; the map frame matches the saved session, so '
                'poses recorded against it are still valid' % self.map_path)
        else:
            self.get_logger().info(
                "origin re-zeroed at the robot's current pose; map frame is "
                'now anchored here')

    def check_timeout(self):
        if self.done:
            return
        el = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        if el > self.timeout_s:
            self.done = True
            self.get_logger().error(
                'cuVSLAM never reported %.1f s of healthy tracking within '
                '%.0f s. Map frame NOT anchored -- treat it with suspicion. '
                'Check the camera is streaming and the scene has texture.'
                % (self.settle_s, self.timeout_s))


def main(args=None):
    rclpy.init(args=args)
    node = OriginAnchor()
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
