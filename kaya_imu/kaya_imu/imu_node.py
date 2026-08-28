"""ROS 2 node publishing sensor_msgs/Imu from the MPU-6050 on the Kaya robot."""

import math
import os

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu, Temperature

from kaya_imu.mpu6050 import MPU6050

# Identity calibration: what we fall back to when no file is supplied.
# Written out explicitly so the "uncalibrated" path is obvious in the logs.
IDENTITY_CALIB = {
    'accel_offset': [0.0, 0.0, 0.0],   # m/s^2, subtracted from the raw reading
    'accel_scale':  [1.0, 1.0, 1.0],   # dimensionless sensor gain, divided out
    'gyro_offset':  [0.0, 0.0, 0.0],   # rad/s, subtracted
    'gyro_scale':   [1.0, 1.0, 1.0],
}


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        # ---------------------------------------------------------- parameters
        # Declared, not hardcoded: every one of these can be overridden from a
        # launch file or the command line without editing source.
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('accel_range_g', 2)
        self.declare_parameter('gyro_range_dps', 250)
        self.declare_parameter('dlpf', 3)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('publish_temperature', True)
        # Sensor noise, as standard deviations. Turned into covariance below.
        self.declare_parameter('accel_stddev', 0.02)   # m/s^2   (measured: ~0.03)
        self.declare_parameter('gyro_stddev', 0.003)   # rad/s

        p = self.get_parameter
        self.frame_id = p('frame_id').value
        rate = p('publish_rate_hz').value

        # -------------------------------------------------------- calibration
        self.calib = self._load_calibration(p('calibration_file').value)

        # ------------------------------------------------------------- driver
        self.imu = MPU6050(
            bus=p('i2c_bus').value,
            address=p('i2c_address').value,
            accel_range=p('accel_range_g').value,
            gyro_range=p('gyro_range_dps').value,
            dlpf=p('dlpf').value,
            sample_rate_hz=rate,
        )
        self.get_logger().info(
            f'MPU6050 up on i2c-{p("i2c_bus").value} @ 0x{p("i2c_address").value:02x}, '
            f'chip rate {self.imu.sample_rate_hz:.1f} Hz')

        # ----------------------------------------------------------------- QoS
        # SENSOR_DATA semantics: BEST_EFFORT + KEEP_LAST(depth 5). A dropped
        # sample at 100 Hz is worth less than the latency of retransmitting it,
        # and the next one is 10 ms away. VOLATILE because a late subscriber
        # wants live data, not a stale reading from before it connected.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Imu, 'imu/data_raw', sensor_qos)
        self.temp_pub = (self.create_publisher(Temperature, 'imu/temperature', sensor_qos)
                         if p('publish_temperature').value else None)

        # ------------------------------------------------- covariance matrices
        av = p('accel_stddev').value ** 2
        gv = p('gyro_stddev').value ** 2
        self.accel_cov = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
        self.gyro_cov = [gv, 0.0, 0.0, 0.0, gv, 0.0, 0.0, 0.0, gv]

        # --------------------------------------------------------------- timer
        # A deadline-aware scheduler, not sleep(). rclpy computes the next fire
        # time from a fixed epoch, so the work done inside the callback does not
        # accumulate into the period the way `read(); sleep(0.01)` does.
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self._count = 0
        self._errors = 0
        self.create_timer(10.0, self._report)

    # ------------------------------------------------------------------ helpers
    def _load_calibration(self, path):
        if not path:
            self.get_logger().warn(
                'No calibration_file given -- publishing RAW data. '
                'Heading will drift. Run: ros2 run kaya_imu calibrate')
            return dict(IDENTITY_CALIB)
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            self.get_logger().error(f'calibration_file {path} not found -- using identity.')
            return dict(IDENTITY_CALIB)
        with open(path) as fh:
            data = yaml.safe_load(fh) or {}
        calib = dict(IDENTITY_CALIB)
        for key in IDENTITY_CALIB:
            if key in data:
                calib[key] = [float(v) for v in data[key]]
        self.get_logger().info(
            f'Loaded calibration from {path}\n'
            f'  accel_offset {calib["accel_offset"]}\n'
            f'  accel_scale  {calib["accel_scale"]}\n'
            f'  gyro_offset  {calib["gyro_offset"]}')
        return calib

    def _apply(self, vec, offset, scale):
        """true = (measured - offset) / scale, per axis."""
        return tuple((v - o) / s for v, o, s in zip(vec, offset, scale))

    # ----------------------------------------------------------------- callback
    def on_timer(self):
        try:
            d = self.imu.read()
        except OSError as exc:
            self._errors += 1
            if self._errors % 50 == 1:
                self.get_logger().error(f'I2C read failed ({self._errors} total): {exc}')
            return

        accel = self._apply(d['accel'], self.calib['accel_offset'], self.calib['accel_scale'])
        gyro = self._apply(d['gyro'], self.calib['gyro_offset'], self.calib['gyro_scale'])

        msg = Imu()
        # Stamp as close to the read as possible; this timestamp is what every
        # downstream filter uses to order and interpolate measurements.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration_covariance = self.accel_cov
        msg.angular_velocity_covariance = self.gyro_cov

        # REP-145: covariance[0] = -1 declares "this message carries NO
        # orientation estimate". The MPU6050 has no magnetometer and we run no
        # fusion here, so claiming an orientation would be a lie that
        # robot_localization would happily believe.
        msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.pub.publish(msg)
        self._count += 1

        if self.temp_pub is not None:
            t = Temperature()
            t.header = msg.header
            t.temperature = d['temp']
            t.variance = 0.0
            self.temp_pub.publish(t)

    def _report(self):
        self.get_logger().info(f'published {self._count} samples ({self._count / 10.0:.1f} Hz)'
                               + (f', {self._errors} I2C errors' if self._errors else ''))
        self._count = 0

    def destroy_node(self):
        try:
            self.imu.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
