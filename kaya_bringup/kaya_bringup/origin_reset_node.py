"""Re-zero cuVSLAM's origin once, after tracking is genuinely healthy.

WHY THIS EXISTS
cuVSLAM's origin is wherever the tracker believed it was when it initialised
-- which is exactly when it is least trustworthy. Measured on this robot,
stationary on the floor the whole time: by the time tracking looked healthy,
map -> base_footprint read (0.285, -0.061, -0.360) m with -9.0 deg pitch. The
-0.36 m in Z is the tell; the robot cannot descend 36 cm, so that pose was
integrated, not travelled. The log showed frames dropping over the same
window, which is the likely cause.

It is not drift. Once settled, the same tracker held 0.87 cm/min and
0.05 deg/min while stationary, and returned a 16.4 m driven loop to 2.2 cm
and 1.1 deg. It is a one-off startup artifact -- but an uncorrected one
poisons everything downstream, because every nvblox voxel and every Nav2 goal
is expressed in that frame.

WHY IT IS A NODE AND NOT A launch TIMER
It was a TimerAction with a 12 s delay first, and that was wrong. The delay
ran from launch start, not from tracker readiness, and the numbers are not
stable: the component container has taken 14 s to instantiate the node, and
camera init has measured anywhere from 1.6 s to 5.2 s (slower right after a
USB re-enumeration). The 12 s timer fired *during* initialisation, and the
tracker never came up at all.

Any fixed delay is a guess about a machine that is busy in a way you cannot
predict. So this waits for the condition that actually matters -- the tracker
reporting healthy tracking, continuously, for settle_s -- and only then
resets. If that never happens it gives up loudly rather than resetting into a
half-initialised tracker.
"""

import rclpy
from rclpy.node import Node

from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from isaac_ros_visual_slam_interfaces.srv import Reset

# vo_state 0 and 1 are both "tracking"; anything else means lost or
# initialising, and resetting then is what we are trying to avoid.
TRACKING_STATES = (0, 1)


class OriginReset(Node):

    def __init__(self):
        super().__init__('cuvslam_origin_reset')
        self.declare_parameter('settle_s', 5.0)
        self.declare_parameter('timeout_s', 120.0)

        self.settle_s = self.get_parameter('settle_s').value
        self.timeout_s = self.get_parameter('timeout_s').value

        self.healthy_since = None
        self.done = False
        self.t0 = self.get_clock().now()

        self.cli = self.create_client(Reset, '/visual_slam/reset')
        self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                 self.on_status, 10)
        self.create_timer(1.0, self.check_timeout)
        self.get_logger().info(
            'waiting for cuVSLAM to track healthily for %.1f s before re-zeroing'
            % self.settle_s)

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
                '/visual_slam/reset unavailable; origin NOT re-zeroed')
            return
        fut = self.cli.call_async(Reset.Request())
        fut.add_done_callback(self.on_reset)

    def on_reset(self, fut):
        try:
            ok = fut.result().success
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error('reset call failed: %r' % e)
            return
        if ok:
            self.get_logger().info(
                'origin re-zeroed at the robot\'s current pose; map frame is '
                'now anchored here')
        else:
            self.get_logger().error('reset service reported failure')

    def check_timeout(self):
        if self.done:
            return
        el = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        if el > self.timeout_s:
            self.done = True
            self.get_logger().error(
                'cuVSLAM never reported %.1f s of healthy tracking within %.0f s. '
                'Origin NOT re-zeroed -- it may carry an initialisation offset, '
                'so treat the map frame with suspicion. Check that the camera is '
                'streaming and the scene has texture.'
                % (self.settle_s, self.timeout_s))


def main(args=None):
    rclpy.init(args=args)
    node = OriginReset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                            # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
