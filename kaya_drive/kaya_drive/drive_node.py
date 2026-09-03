"""ROS 2 node driving the Kaya robot's three omni wheels from /cmd_vel, and
publishing wheel odometry.

Plain node, not a ros2_control hardware interface -- deliberately. Per the
curriculum: write the plain node first to learn the mechanism, port to
ros2_control once the mechanism is understood and trusted.
"""

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from kaya_drive.feetech_bus import FeetechBus, COUNTS_PER_REV
from kaya_drive.kinematics import OmniKinematics


def wrap_delta(new, old, period=COUNTS_PER_REV):
    """Signed shortest delta from old to new on a wrapping counter, e.g.
    4090 -> 5 is +11, not -4085. Valid as long as the true motion between
    reads is less than half a revolution, which holds by a wide margin at
    any control rate/speed this robot runs."""
    return (new - old + period // 2) % period - period // 2


class DriveNode(Node):

    def __init__(self):
        super().__init__('drive_node')

        # ---------------------------------------------------------- parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 1_000_000)
        self.declare_parameter('servo_ids', [1, 2, 3])
        # theta_i in degrees, CCW+ from +X, same order as servo_ids. Confirmed
        # 2026-09-03 -- see kinematics.py docstring and CAD_CONTEXT.md.
        self.declare_parameter('wheel_theta_deg', [-60.0, 60.0, 180.0])
        self.declare_parameter('wheel_radius', 0.037)      # MEASURED, see kaya_dimensions.yaml
        self.declare_parameter('chassis_radius', 0.138)    # CALIBRATED 2026-09-03 via 10-turn spin test, was 0.143
        # Analytical starting point: raw units are encoder counts/sec on a
        # 4096-count/rev wheel, so counts/sec = v_i / (wheel_radius * 2*pi/4096).
        # Phase 2 calibration (measured 2 m roll, then 10 full spins) corrects
        # this for real-world slip and loaded rolling radius -- override via
        # this parameter once that's done, rather than editing the formula.
        self.declare_parameter('wheel_speed_scale', 0.0)  # 0.0 => use the analytical value
        self.declare_parameter('max_linear_speed', 0.15)   # m/s, conservative until calibrated
        self.declare_parameter('max_angular_speed', 1.0)   # rad/s
        self.declare_parameter('cmd_vel_timeout', 0.5)     # s -- stale cmd_vel is treated as zero
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('odom_frame_id', 'odom')
        # base_footprint, not base_link: kaya.urdf.xacro's base_footprint_joint
        # makes base_footprint the PARENT of base_link (fixed, +wheel_radius in
        # Z). base_link already has a parent, so TF's one-parent-per-frame rule
        # means odometry must attach at base_footprint, not base_link, whatever
        # the curriculum prose's shorthand says.
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        p = self.get_parameter
        self.servo_ids = [int(x) for x in p('servo_ids').value]
        thetas = {sid: math.radians(t) for sid, t in
                  zip(self.servo_ids, p('wheel_theta_deg').value)}
        self.wheel_radius = p('wheel_radius').value
        self.kin = OmniKinematics(thetas, p('chassis_radius').value, self.wheel_radius)

        scale = p('wheel_speed_scale').value
        self.counts_per_mps = scale if scale > 0.0 else (
            COUNTS_PER_REV / (2.0 * math.pi * self.wheel_radius))

        self.max_v = p('max_linear_speed').value
        self.max_w = p('max_angular_speed').value
        self.cmd_timeout = p('cmd_vel_timeout').value
        self.odom_frame = p('odom_frame_id').value
        self.base_frame = p('base_frame_id').value
        self.publish_tf = p('publish_tf').value

        # ------------------------------------------------------------- bus
        self.bus = FeetechBus(p('serial_port').value, p('baud').value, self.servo_ids)
        if not self.bus.enter_wheel_mode():
            self.get_logger().error(
                'one or more servos did not confirm wheel mode -- stopping before driving anything')
            self.bus.stop_and_release()
            raise RuntimeError('servo wheel-mode handshake failed')
        self.get_logger().info(
            f'servos {self.servo_ids} in wheel mode on {p("serial_port").value} '
            f'@ {p("baud").value} baud, counts_per_mps={self.counts_per_mps:.1f}')

        # -------------------------------------------------------------- state
        self.last_positions = self.bus.read_positions()
        if len(self.last_positions) != len(self.servo_ids):
            self.get_logger().warn(
                f'only {len(self.last_positions)}/{len(self.servo_ids)} servos answered at '
                'startup -- odometry will not integrate until all three respond')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Unwrapped running total, alongside the wrapped self.theta used for
        # the published pose. Calibrating the rotational term means spinning
        # the robot exactly N turns and comparing to what odometry says
        # happened -- self.theta wraps to [-pi, pi) every revolution and
        # would hide that, so this is what the calibration log line reads.
        self.theta_unwrapped = 0.0
        self.last_odom_time = self.get_clock().now()
        # Accumulated (unwrapped) counts per wheel, for continuous-joint
        # /joint_states -- an ever-increasing angle avoids the visual "snap"
        # a wrapped [-pi, pi) value would show in RViz every revolution.
        self.wheel_accum_counts = {sid: 0 for sid in self.servo_ids}

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        # ---------------------------------------------------------------- I/O
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', odom_qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None
        # Replaces kaya_bringup's placeholder joint_state_publisher, which
        # zero-fills the wheel joints until this exists (see imu.launch.py).
        # Launch that file with real_joint_states:=true alongside this node.
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        # For the measured-roll / N-turn calibration procedure: call this at
        # the exact moment the physical run ends and read the stats back,
        # rather than eyeballing a periodic log line.
        self.create_service(Trigger, 'get_calibration_stats', self.on_get_calibration_stats)
        self.create_service(Trigger, 'reset_odometry', self.on_reset_odometry)

        self.timer = self.create_timer(1.0 / p('control_rate_hz').value, self.on_timer)
        self.get_logger().info('drive_node up')

    # ------------------------------------------------------------ calibration
    def on_get_calibration_stats(self, request, response):
        dist = math.hypot(self.x, self.y)
        response.success = True
        response.message = (
            f'x={self.x:.4f} y={self.y:.4f} straight_line_dist={dist:.4f} m  '
            f'theta_unwrapped={math.degrees(self.theta_unwrapped):.2f} deg '
            f'({self.theta_unwrapped / (2 * math.pi):.4f} turns)  '
            f'wheel_radius_param={self.wheel_radius:.5f}  '
            f'chassis_radius_param={self.kin.chassis_radius:.5f}')
        self.get_logger().info(response.message)
        return response

    def on_reset_odometry(self, request, response):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.theta_unwrapped = 0.0
        response.success = True
        response.message = 'odometry reset to (0, 0, 0)'
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ cmd_vel
    def on_cmd_vel(self, msg):
        self.target_vx = max(-self.max_v, min(self.max_v, msg.linear.x))
        self.target_vy = max(-self.max_v, min(self.max_v, msg.linear.y))
        self.target_w = max(-self.max_w, min(self.max_w, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()

    # -------------------------------------------------------------- control loop
    def on_timer(self):
        stale = (self.get_clock().now() - self.last_cmd_time).nanoseconds > self.cmd_timeout * 1e9
        vx, vy, w = (0.0, 0.0, 0.0) if stale else (self.target_vx, self.target_vy, self.target_w)

        wheel_speeds = self.kin.twist_to_wheel_speeds(vx, vy, w)
        raw = {sid: int(round(v * self.counts_per_mps)) for sid, v in wheel_speeds.items()}
        self.bus.set_wheel_speeds(raw)

        self._update_odometry()

    # ---------------------------------------------------------------- odometry
    def _update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        positions = self.bus.read_positions()
        if len(positions) != len(self.servo_ids):
            # A dropped serial transaction, not a zero-speed wheel -- skip this
            # tick rather than feed kinematics a wrong reading. Neither
            # last_positions nor last_odom_time advances, so the next
            # successful cycle's delta and dt both span the full gap instead
            # of computing an inflated velocity from a stale dt.
            return
        self.last_odom_time = now

        wheel_mps = {}
        for sid in self.servo_ids:
            delta_counts = wrap_delta(positions[sid], self.last_positions[sid])
            wheel_mps[sid] = (delta_counts / COUNTS_PER_REV) * 2.0 * math.pi * self.wheel_radius / dt
            self.wheel_accum_counts[sid] += delta_counts
        self.last_positions = positions

        vx, vy, w = self.kin.wheel_speeds_to_twist(wheel_mps)

        # Integrate in the odom frame: rotate the body-frame velocity by the
        # current heading before accumulating.
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.theta_unwrapped += w * dt
        self.theta = math.atan2(math.sin(self.theta + w * dt), math.cos(self.theta + w * dt))

        stamp = now.to_msg()
        qz, qw = math.sin(self.theta / 2.0), math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Uncalibrated placeholders -- tightened once Phase 2's measured-roll
        # and 10-turn-spin calibration gives real error figures.
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = w
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.02
        self.odom_pub.publish(odom)

        js = JointState()
        js.header.stamp = stamp
        js.name = [f'wheel_{sid}_wheel_joint' for sid in self.servo_ids]
        js.position = [self.wheel_accum_counts[sid] * 2.0 * math.pi / COUNTS_PER_REV
                       for sid in self.servo_ids]
        js.velocity = [wheel_mps[sid] / self.wheel_radius for sid in self.servo_ids]
        self.joint_pub.publish(js)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # -------------------------------------------------------------------- safety
    def destroy_node(self):
        try:
            self.bus.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DriveNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
