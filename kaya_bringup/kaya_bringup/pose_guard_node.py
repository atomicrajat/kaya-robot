"""Watch map -> base_footprint while driving, and stop if it stops being real.

WHY THIS EXISTS
origin_anchor_node validates the pose ONCE, at localisation, and refuses a bad
one. Nothing watched it afterwards. So when cuVSLAM lost visual tracking mid
traverse -- "Visual tracking is lost", after the controller rotated fast enough
to smear the features -- IMU-only dead reckoning integrated the pose out past
13 km while the robot kept driving on it. Nav2 was still planning, still
issuing velocities, still convinced it knew where it was.

That is the failure worth guarding: not that a run fails, but that the robot
keeps moving in the real world on coordinates that are fiction.

It also catches the smaller version of the same thing. On one traverse the pose
jumped from (0.08, 2.31) to (-0.30, 1.13) between samples -- 1.2 m in about 4 s
on a robot capped at 0.15 m/s, so not motion. A jump like that moves the goal
relative to the robot mid-approach, and is enough on its own to explain goal
approaches that wander.

WHAT IT CHECKS, and why each one
  Z and TILT      This robot drives on a flat floor. A map pose 0.5 m
                  underground or rolled 40 deg is not a pose, it is a failure
                  wearing one. Same check origin_anchor_node applies at
                  localisation; there is no reason it should stop being true
                  a minute later.
  SPEED           Implied speed between consecutive transforms. vx_max and
                  vy_max are 0.15 m/s, so about 0.21 m/s on the diagonal.
                  Anything far above that was not travelled.

WHAT IT DOES ABOUT IT
Cancels whatever Nav2 is executing, through the navigate_to_pose action's own
cancel service, and latches the verdict on ~/pose_ok. It does NOT publish to
/cmd_vel: fighting the controller for the topic would be a race, and the
controller would win half of it. Cancelling removes the thing generating the
commands.

It does not try to recover. A diverged tracker needs relocalisation, and
guessing where the robot is in order to keep going is the exact mistake this
node exists to prevent.
"""

import math

from action_msgs.srv import CancelGoal
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, String
import tf2_ros


class PoseGuard(Node):

    def __init__(self):
        super().__init__('pose_guard')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rate_hz', 10.0)
        # Same limits origin_anchor_node uses when it accepts a localisation.
        self.declare_parameter('max_z_m', 0.10)
        self.declare_parameter('max_tilt_deg', 8.0)
        # vx_max and vy_max are 0.15, so ~0.21 m/s diagonally. 0.6 leaves room
        # for TF timing jitter without admitting anything that could be real.
        self.declare_parameter('max_speed_m_s', 0.6)
        # One bad sample is jitter; several in a row is a diverged tracker.
        self.declare_parameter('strikes', 3)
        self.declare_parameter('enabled', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.strikes_needed = int(self.get_parameter('strikes').value)

        self.prev = None
        self.strikes = 0
        self.tripped = False

        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ok_pub = self.create_publisher(Bool, '~/pose_ok', latched)
        self.why_pub = self.create_publisher(String, '~/pose_fault', latched)
        self.cancel_cli = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)
        self.ok_pub.publish(Bool(data=True))
        self.get_logger().info(
            'watching %s -> %s: |z| <= %.2f m, tilt <= %.1f deg, speed <= %.2f m/s'
            % (self.map_frame, self.base_frame,
               self.get_parameter('max_z_m').value,
               self.get_parameter('max_tilt_deg').value,
               self.get_parameter('max_speed_m_s').value))

    @staticmethod
    def _roll_pitch(q):
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                          1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        return roll, math.asin(sinp)

    def tick(self):
        if self.tripped or not self.get_parameter('enabled').value:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.0))
        except tf2_ros.TransformException:
            return

        stamp = Time.from_msg(tf.header.stamp).nanoseconds / 1e9
        t = tf.transform.translation
        roll, pitch = self._roll_pitch(tf.transform.rotation)
        faults = []

        max_z = self.get_parameter('max_z_m').value
        max_tilt = math.radians(self.get_parameter('max_tilt_deg').value)
        if abs(t.z) > max_z:
            faults.append('z = %+.3f m (limit %.2f)' % (t.z, max_z))
        if abs(roll) > max_tilt or abs(pitch) > max_tilt:
            faults.append('roll %+.1f deg, pitch %+.1f deg (limit %.1f)'
                          % (math.degrees(roll), math.degrees(pitch),
                             self.get_parameter('max_tilt_deg').value))

        if self.prev is not None:
            dt = stamp - self.prev[0]
            if dt > 1e-3:
                d = math.hypot(t.x - self.prev[1], t.y - self.prev[2])
                speed = d / dt
                if speed > self.get_parameter('max_speed_m_s').value:
                    faults.append(
                        'moved %.2f m in %.2f s = %.2f m/s, which this robot '
                        'cannot do' % (d, dt, speed))
        if stamp != (self.prev[0] if self.prev else None):
            self.prev = (stamp, t.x, t.y)

        if not faults:
            self.strikes = 0
            return

        self.strikes += 1
        self.get_logger().warn('pose looks wrong (%d/%d): %s'
                               % (self.strikes, self.strikes_needed,
                                  '; '.join(faults)))
        if self.strikes >= self.strikes_needed:
            self.trip('; '.join(faults))

    def trip(self, why):
        self.tripped = True
        self.get_logger().error(
            'POSE NOT TRUSTWORTHY -- cancelling navigation. %s. The tracker '
            'has diverged; the robot was about to keep driving on coordinates '
            'that are not real. Relocalise before moving again.' % why)
        self.ok_pub.publish(Bool(data=False))
        self.why_pub.publish(String(data=why))
        if self.cancel_cli.service_is_ready():
            # Empty goal_info cancels everything the action server is running.
            self.cancel_cli.call_async(CancelGoal.Request())
        else:
            self.get_logger().error(
                'navigate_to_pose cancel service unavailable; could not stop '
                'the robot from here')


def main(args=None):
    rclpy.init(args=args)
    node = PoseGuard()
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
