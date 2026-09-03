"""Conditions the Gemini 335L's built-in IMU for use by the Kaya navigation stack.

The Orbbec driver publishes a well-formed sensor_msgs/Imu on
/camera/gyro_accel/sample at ~203 Hz with genuine device timestamps. Three things
still stand between that and something an EKF can consume:

  1. Covariances are launch-file placeholders (0.01), ~200x pessimistic.
     We measure the real noise floor at startup and publish that instead.
  2. A residual gyro bias of ~0.56 deg/s remains after the factory correction,
     which integrates to ~39 deg/min of heading drift.
  3. Nothing re-zeroes the bias after the sensor warms up.

Frames: this node does NOT rotate the data. The samples stay in the camera's
optical frame and the geometric relationship to base_link is expressed in the
URDF, which is where REP-105 says it belongs. Rotating data in a driver is how
you end up with two sources of truth for the same transform.
"""

import math
import os
import time
import statistics
import threading

import rclpy
import yaml
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


class CameraImuNode(Node):

    def __init__(self):
        super().__init__('camera_imu_node')

        self.declare_parameter('input_topic', '/camera/gyro_accel/sample')
        self.declare_parameter('output_topic', 'imu/data_raw')
        # Empty means "pass the incoming frame_id through untouched", which is
        # the correct default: we are not rotating the data, so we must not
        # relabel the frame it is expressed in.
        self.declare_parameter('frame_id', '')
        self.declare_parameter('bias_file', '')
        self.declare_parameter('auto_bias_on_start', True)
        self.declare_parameter('bias_samples', 600)          # ~3 s at 203 Hz
        self.declare_parameter('still_gyro_range_max', 0.02)  # rad/s peak-to-peak
        # Gyro bias is NOT constant: it walks as the sensor warms. Measured on
        # this camera, startup vs ~30 min later, the optical-frame Y axis --
        # the one that becomes robot YAW, since the optical frame has Y down --
        # moved 0.00203 rad/s, i.e. 7.0 deg/min of accumulating heading error,
        # and changed sign. A single startup calibration is therefore worth
        # very little an hour into a patrol. These re-estimate it automatically
        # whenever the robot has been genuinely still long enough, which is the
        # only time the estimate is valid.
        self.declare_parameter('auto_rezero_enabled', True)
        self.declare_parameter('auto_rezero_interval_s', 60.0)
        self.declare_parameter('auto_rezero_still_s', 3.0)
        # If >0 these override the measured noise floor.
        self.declare_parameter('gyro_stddev_override', 0.0)
        self.declare_parameter('accel_stddev_override', 0.0)

        p = self.get_parameter
        self.frame_override = p('frame_id').value
        self.n_bias = int(p('bias_samples').value)
        self.still_max = float(p('still_gyro_range_max').value)
        self.auto_rezero = bool(p('auto_rezero_enabled').value)
        self.rezero_interval = float(p('auto_rezero_interval_s').value)
        self.rezero_still_s = float(p('auto_rezero_still_s').value)

        self.gyro_bias = [0.0, 0.0, 0.0]
        self.gyro_cov = self._diag(0.01)   # placeholder until measured
        self.accel_cov = self._diag(0.01)
        self._lock = threading.Lock()
        self._collecting = False
        self._buf_g, self._buf_a = [], []
        self._on_done = None
        self._count = 0
        self._still_window = []
        # ~1 s of samples at 203 Hz: long enough that a brief pause during a
        # manoeuvre cannot masquerade as the robot being at rest.
        self._still_window_n = 200
        self._still_since = None      # monotonic time the robot last started being still
        self._last_bias_time = 0.0    # monotonic time of the last successful estimate

        if p('bias_file').value:
            self._load_bias(p('bias_file').value)

        # ------------------------------------------------------------- QoS
        # MUST match the Orbbec publisher, which is RELIABLE. A BEST_EFFORT
        # subscriber would also match here (asking for less than offered is
        # always allowed) but we take the data as-offered.
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=100,
                         durability=QoSDurabilityPolicy.VOLATILE)

        # CALLBACK GROUPS -- the reason the first version of this node deadlocked.
        # Every callback created without an explicit group lands in the node's
        # DEFAULT group, which is MutuallyExclusive: at most one of its callbacks
        # runs at a time, no matter how many executor threads exist. The rezero
        # service blocks waiting for the subscription to fill a buffer, so with
        # both in one group the subscription could never run and the service
        # always timed out. Separate groups let the MultiThreadedExecutor run
        # them concurrently.
        self._sub_group = MutuallyExclusiveCallbackGroup()
        self._srv_group = MutuallyExclusiveCallbackGroup()

        self.sub = self.create_subscription(Imu, p('input_topic').value, self.on_imu, qos,
                                            callback_group=self._sub_group)
        self.pub = self.create_publisher(Imu, p('output_topic').value, qos)

        # A Trigger service, not a topic: re-zeroing is a request with a
        # definite completion and a success/failure answer. That is exactly the
        # shape a service is for. (A long-running job with feedback would be an
        # action instead -- Nav2 goals are actions for that reason.)
        self.srv = self.create_service(Trigger, 'imu/rezero_gyro', self.on_rezero,
                                       callback_group=self._srv_group)

        self.create_timer(10.0, self._report)

        if p('auto_bias_on_start').value:
            self.get_logger().info(
                f'Estimating gyro bias from {self.n_bias} samples -- KEEP THE ROBOT STILL.')
            self._start_collection(self._finish_startup_bias)
        else:
            self.get_logger().info('auto_bias_on_start=false; using loaded/zero bias.')

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _diag(v):
        return [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]

    def _load_bias(self, path):
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            self.get_logger().warn(f'bias_file {path} not found; starting from zero.')
            return
        with open(path) as fh:
            d = yaml.safe_load(fh) or {}
        if 'gyro_bias' in d:
            self.gyro_bias = [float(v) for v in d['gyro_bias']]
            self.get_logger().info(f'Loaded gyro_bias {self.gyro_bias} from {path}')

    def _start_collection(self, on_done):
        with self._lock:
            self._buf_g, self._buf_a = [], []
            self._on_done = on_done
            self._collecting = True

    def _finish_startup_bias(self, ok, msg):
        # Seed the auto-rezero clock so the periodic re-estimate does not fire
        # immediately on top of the startup one.
        self._last_bias_time = time.monotonic()
        lvl = self.get_logger().info if ok else self.get_logger().warn
        lvl(msg)

    # ----------------------------------------------------------------- callback
    def on_imu(self, msg):
        g = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        a = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)

        if self._collecting:
            with self._lock:
                self._buf_g.append(g)
                self._buf_a.append(a)
                if len(self._buf_g) >= self.n_bias:
                    self._collecting = False
                    ok, text = self._solve_bias()
                    if self._on_done:
                        self._on_done(ok, text)
                    self._on_done = None
        else:
            self._maybe_auto_rezero(g)

        out = Imu()
        out.header.stamp = msg.header.stamp          # preserve the DEVICE stamp
        out.header.frame_id = self.frame_override or msg.header.frame_id
        out.angular_velocity.x = g[0] - self.gyro_bias[0]
        out.angular_velocity.y = g[1] - self.gyro_bias[1]
        out.angular_velocity.z = g[2] - self.gyro_bias[2]
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = a
        out.angular_velocity_covariance = self.gyro_cov
        out.linear_acceleration_covariance = self.accel_cov
        # REP-145: no orientation estimate in this message.
        out.orientation_covariance = [-1.0] + [0.0] * 8
        out.orientation.w = 1.0
        self.pub.publish(out)
        self._count += 1

    # -------------------------------------------------------- automatic re-zero
    def _maybe_auto_rezero(self, g):
        """Re-estimate gyro bias periodically, but only while the robot is
        genuinely still -- a bias estimated during motion would bake the
        motion into the correction. Stillness is judged from the gyro itself
        rather than from /cmd_vel: a commanded stop is not the same as a
        stopped robot, and this node deliberately has no dependency on the
        drive stack."""
        if not self.auto_rezero:
            return
        now = time.monotonic()

        # Track a short rolling window of recent samples and require the whole
        # window to be quiet, so a momentary lull mid-motion cannot qualify.
        self._still_window.append(g)
        if len(self._still_window) > self._still_window_n:
            self._still_window.pop(0)
        if len(self._still_window) < self._still_window_n:
            return

        rng = max(max(s[i] for s in self._still_window) - min(s[i] for s in self._still_window)
                  for i in range(3))
        if rng > self.still_max:
            self._still_since = None
            return

        if self._still_since is None:
            self._still_since = now
            return

        if (now - self._still_since) < self.rezero_still_s:
            return
        if (now - self._last_bias_time) < self.rezero_interval:
            return

        self.get_logger().info(
            f'still for {now - self._still_since:.1f}s -- re-estimating gyro bias')
        self._start_collection(self._finish_auto_rezero)

    def _finish_auto_rezero(self, ok, msg):
        self._last_bias_time = time.monotonic()
        self._still_since = None
        self._still_window.clear()
        if ok:
            self.get_logger().info(f'auto re-zero: {msg}')
        else:
            self.get_logger().warn(f'auto re-zero skipped: {msg}')

    # -------------------------------------------------------------------- solve
    def _solve_bias(self):
        g, a = list(self._buf_g), list(self._buf_a)
        n = len(g)
        rng = [max(s[i] for s in g) - min(s[i] for s in g) for i in range(3)]
        if max(rng) > self.still_max:
            return False, (f'Robot was MOVING during bias estimation '
                           f'(gyro range {max(rng):.4f} > {self.still_max} rad/s). '
                           f'Bias unchanged. Hold still and call /imu/rezero_gyro.')

        self.gyro_bias = [statistics.fmean([s[i] for s in g]) for i in range(3)]
        gsd = [statistics.stdev([s[i] for s in g]) for i in range(3)]
        asd = [statistics.stdev([s[i] for s in a]) for i in range(3)]

        ov_g = self.get_parameter('gyro_stddev_override').value
        ov_a = self.get_parameter('accel_stddev_override').value
        # Use the worst axis: covariance here is isotropic, so the honest
        # single number is the largest, not the average.
        sg = ov_g if ov_g > 0 else max(gsd)
        sa = ov_a if ov_a > 0 else max(asd)
        self.gyro_cov = self._diag(sg ** 2)
        self.accel_cov = self._diag(sa ** 2)

        drift = math.degrees(math.sqrt(sum(v * v for v in self.gyro_bias))) * 60
        return True, (
            f'Bias estimated from {n} samples:\n'
            f'  gyro_bias  ({self.gyro_bias[0]:+.6f}, {self.gyro_bias[1]:+.6f}, '
            f'{self.gyro_bias[2]:+.6f}) rad/s  -> removed {drift:.1f} deg/min of drift\n'
            f'  gyro  sd   ({gsd[0]:.6f}, {gsd[1]:.6f}, {gsd[2]:.6f}) rad/s  '
            f'-> cov {sg**2:.3e}\n'
            f'  accel sd   ({asd[0]:.6f}, {asd[1]:.6f}, {asd[2]:.6f}) m/s^2 '
            f'-> cov {sa**2:.3e}')

    # ------------------------------------------------------------------ service
    def on_rezero(self, request, response):
        done = threading.Event()
        result = {}

        def finished(ok, text):
            result['ok'], result['text'] = ok, text
            done.set()

        self.get_logger().info('rezero_gyro requested -- hold still...')
        self._start_collection(finished)
        # Blocking here is only safe because on_imu lives in a DIFFERENT
        # callback group and the executor is multi-threaded. See __init__.
        if not done.wait(timeout=15.0):
            self._collecting = False
            response.success = False
            response.message = 'timed out waiting for samples (is the camera publishing?)'
            return response
        response.success = result['ok']
        response.message = result['text']
        self.get_logger().info(result['text'])
        return response

    def _report(self):
        if self._count:
            self.get_logger().info(f'republished {self._count} samples ({self._count/10.0:.1f} Hz)')
        self._count = 0


def main(args=None):
    rclpy.init(args=args)
    node = CameraImuNode()
    # MultiThreadedExecutor: the rezero service blocks while the subscription
    # callback fills the buffer. On the default single-threaded executor that
    # would deadlock -- the service would hold the only thread and the
    # subscription could never run.
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
