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

WHY LOCALIZATION NEEDS A VERDICT OF ITS OWN
`LocalizeInMap` is ASYNCHRONOUS. Its response field is documented upstream as
"indicate successful run of the service" and is set the moment the request is
accepted -- the node logs "Async localization started, waiting for callback
during normal tracking" and returns true. The real outcome lands one to two
seconds later.

An earlier version of this node took that response as the answer, so it
reported

    localized in <map>; the map frame matches the saved session, so
    poses recorded against it are still valid

on every run -- including runs where the very next line from cuVSLAM was
"Failed to localize in map". The one component whose job is to say whether the
map frame can be trusted always said yes. That is the same class of bug as the
time_domain regression: a confident health report over a broken estimator.

There is no API for the outcome. Checked against isaac_ros_visual_slam 4.6.0:
LocalizeInMap, Reset, SetSlamPose and FilePath all respond with a bare
"service ran" bool; GetAllPoses returns only the current session's trajectory,
not the loaded map; and VisualSlamStatus carries vo_state and timings with no
localization field. cuVSLAM reports the outcome to its logger and nowhere else.

So this node reads it from `/rosout`, which is where ROS 2 publishes every
node's log records, and treats an unrecognised or absent verdict as UNVERIFIED
rather than as success. If a future release reworks those strings the node goes
quiet and says so; it does not go back to claiming victory.

WHY THE VERDICT ALONE IS NOT ENOUGH
cuVSLAM can also be wrong, and says so in exactly the same words it uses when
it is right. Measured on this robot standing at the saved map's origin, at
night against a map built in afternoon daylight, on an otherwise idle board:

    hint 0.0 m from truth    localized to 1-2 cm        correct
    hint 2.0 m from truth    localized to 2.3 cm        correct
    hint 3.5 m from truth    localized to 1.5 cm        correct
    hint 5.0 m from truth    "Localization failed"      correct

So the recovery radius that afternoon spanned at least 5 m -- it recovered from
a 5 m hint to 5.9 cm -- and at night sits between 3.5 m and 5 m. Nothing in the
stack changed; the room did. Relocalisation is matching a daytime map against a
night scene, and the weaker the match, the smaller the radius. IR texture in
that room measured 4.33, against ~11.8 for a scene with features and ~4.6 for
the dark room in tools/README.md.

The same 3.5 m hint on a LOADED board is where it turns dangerous. With ten
orphaned nvblox_node processes left over from earlier launches, ~54 MB of the
7.5 GB free and load average 49, the same request returned

    "Successfully localized at {1.82, -1.66, -0.69}"

twice -- a pose 2.4 m from truth and 69 cm below the floor, reported as a
success. Nav2 would have planned against it. Memory pressure on this board is
not hypothetical: the nav stack already holds 3.7 GB, and Phase 11 adds a
detector on the same GPU.

The Z is what gives it away: this robot drives on a flat floor, so a map pose
69 cm underground is not a pose, it is a failure wearing one. That is the same
tell as the -0.36 m above, and the same reason tools/drift_log.py calls Z out
separately.

So a claimed success is checked against the geometry the robot is physically
capable of before it is believed.

AND THE CHECK ITSELF HAS TO BE HONEST
The first version of that check read the pose once, a second after the verdict,
and rejected a good 2 cm localization: the single lookup came back (0.179,
-0.117, 0.288) from a transform buffered before localization, while the settled
pose was (-0.002, 0.003, 0.003). Sampled at 5 Hz the pose was in fact steady
from the verdict onward, so there was no transient to wait out -- the reading
was simply stale. A check that invents failures is no better than one that
invents successes, so the pose is now polled until consecutive fresh samples
agree, and only a settled pose is judged.
"""

import math

from geometry_msgs.msg import Pose
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from isaac_ros_visual_slam_interfaces.srv import LocalizeInMap, Reset
from rcl_interfaces.msg import Log
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool
import tf2_ros

# VisualSlamStatus documents vo_state as 0 = Unknown, 1 = Success, 2 = Failed.
# An earlier version of this node accepted 0 as well, on the belief that both
# 0 and 1 meant tracking; the message definition says otherwise, and acting
# while the tracker calls its own state unknown is exactly what this node
# exists to avoid. Sampled over a full startup-to-localization run on this
# robot, vo_state was 1 on all 247 messages and 0 never appeared, so requiring
# 1 costs nothing in practice.
TRACKING_STATES = (1,)

# How cuVSLAM words the outcome, as of isaac_ros_visual_slam 4.6.0. Matched as
# substrings against /rosout records from VSLAM_LOGGER only. Anything else --
# including silence -- is UNVERIFIED, never success.
VSLAM_LOGGER = 'visual_slam_node'
VERDICT_OK = 'Successfully localized'
VERDICT_FAIL = ('Failed to localize', 'Localization failed')


class OriginAnchor(Node):

    def __init__(self):
        super().__init__('cuvslam_origin_anchor')
        self.declare_parameter('settle_s', 5.0)
        self.declare_parameter('timeout_s', 120.0)
        # Empty means "no saved map": reset instead of localizing.
        self.declare_parameter('map_path', '')
        # Pose hint for localization, map_T_robot. Only a HINT, but not a free
        # one -- see the measured recovery radius in the module docstring. Set
        # it if you know roughly where the robot is starting.
        self.declare_parameter('hint_x', 0.0)
        self.declare_parameter('hint_y', 0.0)
        self.declare_parameter('hint_yaw', 0.0)
        # How long to wait for cuVSLAM's async outcome before giving up on it.
        # Observed at 1.4-1.5 s on this robot; 20 s is slack, not a target.
        self.declare_parameter('verdict_timeout_s', 20.0)
        # Reading the pose the verdict produced. A single lookup right after
        # the verdict is not safe: it can return a transform buffered from
        # BEFORE localization, which on this robot read (0.179, -0.117, 0.288)
        # while the settled pose was (-0.002, 0.003, 0.003) -- the node then
        # rejected a perfectly good 2 cm localization on stale data. So the
        # pose is polled until consecutive samples agree, and only a settled
        # pose is judged.
        self.declare_parameter('verify_period_s', 0.2)
        self.declare_parameter('settle_window_s', 1.5)
        self.declare_parameter('settle_tol_m', 0.02)
        self.declare_parameter('verify_timeout_s', 15.0)
        # What the robot is physically capable of. It drives on a flat floor,
        # so a localized pose outside these is a false positive, not a pose.
        self.declare_parameter('max_z_m', 0.05)
        self.declare_parameter('max_tilt_deg', 5.0)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')

        self.settle_s = self.get_parameter('settle_s').value
        self.timeout_s = self.get_parameter('timeout_s').value
        self.map_path = self.get_parameter('map_path').value
        self.verdict_timeout_s = self.get_parameter('verdict_timeout_s').value

        self.healthy_since = None
        self.done = False
        self.awaiting_verdict = False
        self.requested_at = None
        self.samples = []
        self.verify_started = None
        self.t0 = self.get_clock().now()

        # Latched, so anything that starts later still learns whether the map
        # frame can be trusted. Nothing downstream should plan against a map
        # frame this node has not confirmed.
        self.anchored_pub = self.create_publisher(
            Bool, '~/anchored',
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.map_path:
            self.cli = self.create_client(LocalizeInMap, '/visual_slam/localize_in_map')
            what = 'localize in %s' % self.map_path
            # Only the localization path has an outcome worth chasing; Reset
            # completes within its own response.
            self.create_subscription(Log, '/rosout', self.on_log, 100)
        else:
            self.cli = self.create_client(Reset, '/visual_slam/reset')
            what = 're-zero the origin here (no map_path given)'

        self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                 self.on_status, 10)
        self.create_timer(1.0, self.check_timeout)
        self.get_logger().info(
            'waiting for %.1f s of healthy tracking, then will %s'
            % (self.settle_s, what))

    # ------------------------------------------------------------- settling

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
            self.fail('%s unavailable' % self.cli.srv_name)
            return
        if self.map_path:
            req = LocalizeInMap.Request()
            req.map_folder_path = self.map_path
            req.pose_hint = self._hint()
            # Set before the call: the verdict can beat the future's callback.
            self.awaiting_verdict = True
            self.requested_at = self.get_clock().now()
        else:
            req = Reset.Request()
        self.cli.call_async(req).add_done_callback(self.on_done)

    def _hint(self):
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
            self.awaiting_verdict = False
            self.fail('call failed: %r' % e)
            return
        if not ok:
            self.awaiting_verdict = False
            self.fail(
                'the service itself refused the request. If localizing, check '
                'the map path exists and contains a cuVSLAM database.')
            return
        if not self.map_path:
            # Reset is synchronous: this response IS the outcome.
            self.succeed(
                "origin re-zeroed at the robot's current pose; map frame is "
                'now anchored here')
            return
        # Localization is not done, only accepted. Say exactly that, and wait.
        self.get_logger().info(
            'localization request accepted for %s; waiting up to %.0f s for '
            'cuVSLAM to report the outcome'
            % (self.map_path, self.verdict_timeout_s))

    # -------------------------------------------------------------- verdict

    def on_log(self, msg):
        if not self.awaiting_verdict or msg.name != VSLAM_LOGGER:
            return
        if VERDICT_OK in msg.msg:
            self.awaiting_verdict = False
            self.get_logger().info('cuVSLAM reports: %s' % msg.msg.strip())
            self.samples = []
            self.verify_started = self.get_clock().now()
            self.verify_timer = self.create_timer(
                self.get_parameter('verify_period_s').value, self.verify_tick)
            return
        if any(p in msg.msg for p in VERDICT_FAIL):
            self.awaiting_verdict = False
            self.fail(
                'cuVSLAM could not find the robot in %s. Either it is not '
                'anywhere in that map, the pose hint is too far from truth, '
                'or the scene no longer looks like it did when the map was '
                'built -- illumination counts, and the recovery radius shrinks '
                'as the match weakens.' % self.map_path)

    def verify_tick(self):
        """Poll map -> base until it holds still, then judge the settled pose.

        Non-blocking on purpose: a lookup timeout here would stall the single
        threaded executor, and the TF listener shares it -- the node would be
        waiting for transforms it has stopped being able to receive.
        """
        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        waited = (self.get_clock().now() - self.verify_started).nanoseconds / 1e9

        try:
            tf = self.tf_buffer.lookup_transform(
                map_frame, base_frame, Time(), timeout=Duration(seconds=0.0))
            stamp = Time.from_msg(tf.header.stamp).nanoseconds
            # Only count genuinely new transforms towards "settled"; a repeated
            # stale one would otherwise look like perfect stability.
            if not self.samples or stamp != self.samples[-1][0]:
                self.samples.append((stamp, tf))
        except tf2_ros.TransformException:
            tf = None

        window = self.get_parameter('settle_window_s').value
        period = self.get_parameter('verify_period_s').value
        needed = max(3, int(window / period))
        if len(self.samples) >= needed:
            recent = [s[1].transform.translation for s in self.samples[-needed:]]
            spread = max(max(abs(a.x - b.x), abs(a.y - b.y), abs(a.z - b.z))
                         for a in recent for b in recent)
            if spread <= self.get_parameter('settle_tol_m').value:
                self.verify_timer.cancel()
                self.judge(self.samples[-1][1])
                return

        if waited > self.get_parameter('verify_timeout_s').value:
            self.verify_timer.cancel()
            if tf is None and not self.samples:
                self.fail(
                    'cuVSLAM claimed success but %s -> %s never became '
                    'readable, so the claim is unverified'
                    % (map_frame, base_frame))
            else:
                self.fail(
                    'cuVSLAM claimed success but %s -> %s never held still '
                    'within %.0f s, so the pose it produced cannot be checked. '
                    'An unsettled pose is not one to plan against.'
                    % (map_frame, base_frame,
                       self.get_parameter('verify_timeout_s').value))

    def judge(self, tf):
        """Check the settled pose against what this robot can physically be."""
        t = tf.transform.translation
        roll, pitch = self._roll_pitch(tf.transform.rotation)
        max_z = self.get_parameter('max_z_m').value
        max_tilt = math.radians(self.get_parameter('max_tilt_deg').value)

        bad = []
        if abs(t.z) > max_z:
            bad.append('z = %+.3f m (limit %.3f)' % (t.z, max_z))
        if abs(roll) > max_tilt or abs(pitch) > max_tilt:
            bad.append('roll = %+.1f deg, pitch = %+.1f deg (limit %.1f)'
                       % (math.degrees(roll), math.degrees(pitch),
                          self.get_parameter('max_tilt_deg').value))
        if bad:
            self.fail(
                'cuVSLAM reported success at (%.3f, %.3f, %.3f) but the pose '
                'is not one this robot can hold on a flat floor: %s. Treat it '
                'as a FAILED localization -- a wrong pose that claims to be '
                'right is worse than no pose.'
                % (t.x, t.y, t.z, '; '.join(bad)))
            return

        self.succeed(
            'localized in %s at (%.3f, %.3f, %.3f), z and tilt within a flat '
            'floor; the map frame matches the saved session, so poses recorded '
            'against it are still valid' % (self.map_path, t.x, t.y, t.z))

    # ------------------------------------------------------------ reporting

    def succeed(self, why):
        self.get_logger().info(why)
        self.anchored_pub.publish(Bool(data=True))

    def fail(self, why):
        self.get_logger().error('map frame NOT anchored: %s' % why)
        self.anchored_pub.publish(Bool(data=False))

    def check_timeout(self):
        if self.awaiting_verdict:
            waited = (self.get_clock().now() - self.requested_at).nanoseconds / 1e9
            if waited > self.verdict_timeout_s:
                self.awaiting_verdict = False
                self.fail(
                    'cuVSLAM never reported an outcome within %.0f s. The '
                    'request was accepted, so localization may or may not have '
                    'happened -- that is exactly what must not be assumed. If '
                    'this repeats, check whether a newer isaac_ros_visual_slam '
                    'reworded the messages this node matches on.'
                    % self.verdict_timeout_s)
            return
        if self.done:
            return
        el = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        if el > self.timeout_s:
            self.done = True
            self.fail(
                'cuVSLAM never reported %.1f s of healthy tracking within '
                '%.0f s. Check the camera is streaming and the scene has '
                'texture.' % (self.settle_s, self.timeout_s))

    @staticmethod
    def _roll_pitch(q):
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                          1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        return roll, math.asin(sinp)


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
