"""Apply the camera settings the launch route silently drops, and verify them.

WHY THIS EXISTS
Some Orbbec parameters passed through `launch_arguments` never reach the
driver. They are not rejected and nothing is logged -- the launch succeeds, the
camera runs, and `ros2 param get` still reports the old value. Measured on this
robot: `ir_ae_max_exposure: 20000` applied from launch, while
`enable_ir_long_exposure: false` did NOT, and kept reading true across three
relaunches.

That one matters more than it looks. A sensor cannot expose for longer than its
frame period -- 33.3 ms at 30 fps -- and with long exposure enabled and no
upper bound, auto-exposure settled near 50 ms in ordinary indoor light. The
camera then delivered 20 Hz no matter what fps it was asked for, and the 51 ms
median gap between IR frames WAS the exposure time:

    left_ir    18.0 Hz  ->  28.4 Hz   with long exposure off and AE capped
    right_ir   20.9 Hz  ->  30.8 Hz

Late IR makes cuVSLAM's pose late, which makes TF late, and a late TF is what
made Nav2 discard the goal and report "Reached the goal!" 23 cm into a 5 m
drive. So this is not a picture-quality setting; it is why the robot arrives.

IT VERIFIES, RATHER THAN ASSUMING
Every value is read back after being set, and a mismatch is logged as an ERROR
naming both values. A tuner that silently fails would recreate exactly the
problem it exists to fix -- this project has now been caught twice by settings
that reported success and did nothing.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

CAMERA = '/camera/camera'

# name -> (value, type). Only settings the launch route is known to drop, or
# that must be re-asserted after the stream starts.
WANTED = {
    'enable_ir_long_exposure': (False, ParameterType.PARAMETER_BOOL),
    'ir_ae_max_exposure': (20000, ParameterType.PARAMETER_INTEGER),
}


class CameraTuner(Node):

    def __init__(self):
        super().__init__('camera_tuner')
        self.declare_parameter('camera_node', CAMERA)
        self.declare_parameter('settle_s', 12.0)
        node = self.get_parameter('camera_node').value
        self.set_cli = self.create_client(SetParameters, node + '/set_parameters')
        self.get_cli = self.create_client(GetParameters, node + '/get_parameters')
        # The camera takes a while to come up and start streaming; applying
        # these before the stream exists is how they get dropped.
        self.timer = self.create_timer(
            float(self.get_parameter('settle_s').value), self.apply)
        self.get_logger().info(
            'waiting %.0f s for %s, then applying %d camera settings'
            % (self.get_parameter('settle_s').value, node, len(WANTED)))

    def apply(self):
        self.timer.cancel()
        if not self.set_cli.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(
                'camera set_parameters never appeared; IR exposure is NOT '
                'capped, so expect ~20 Hz IR and late transforms')
            return

        params = []
        for name, (val, typ) in WANTED.items():
            pv = ParameterValue(type=typ)
            if typ == ParameterType.PARAMETER_BOOL:
                pv.bool_value = val
            else:
                pv.integer_value = val
            params.append(Parameter(name=name, value=pv))

        fut = self.set_cli.call_async(SetParameters.Request(parameters=params))
        fut.add_done_callback(self.on_set)

    def on_set(self, fut):
        try:
            results = fut.result().results
        except Exception as e:                            # noqa: BLE001
            self.get_logger().error('set_parameters failed: %s' % e)
            return
        for name, r in zip(WANTED, results):
            if not r.successful:
                self.get_logger().error('%s rejected: %s' % (name, r.reason))
        # Reported success is not the same as applied -- that is the whole
        # reason this node exists.
        self.create_timer(2.0, self.verify_once)

    def verify_once(self):
        for t in list(self.timers):
            if t.timer_period_ns == 2_000_000_000:
                t.cancel()
        fut = self.get_cli.call_async(
            GetParameters.Request(names=list(WANTED)))
        fut.add_done_callback(self.on_verify)

    def on_verify(self, fut):
        try:
            values = fut.result().values
        except Exception as e:                            # noqa: BLE001
            self.get_logger().error('could not read parameters back: %s' % e)
            return
        for name, got in zip(WANTED, values):
            want, typ = WANTED[name]
            actual = (got.bool_value if typ == ParameterType.PARAMETER_BOOL
                      else got.integer_value)
            if actual != want:
                self.get_logger().error(
                    '%s is %r, wanted %r -- the setting did NOT take. IR will '
                    'run slow and transforms will be late.'
                    % (name, actual, want))
            else:
                self.get_logger().info('%s = %r' % (name, actual))


def main(args=None):
    rclpy.init(args=args)
    node = CameraTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                                 # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
