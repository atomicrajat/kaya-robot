"""Make sure map_server actually reaches ACTIVE, and say so if it does not.

WHY THIS EXISTS
map_server is a lifecycle node; nav2's lifecycle_manager is supposed to bring
it up with autostart. On this board that intermittently fails at startup, with
map_server logging

    failed to send response to /map_server/change_state (timeout):
    client will not receive response

The map itself loads fine -- 312 x 369 at 0.05 m -- but the transition response
is lost, the manager never learns the node activated, and map_server sits in
INACTIVE. An inactive lifecycle node does not publish, so /map never arrives,
the global costmap's static_layer has nothing, and the planner fails with

    GridBased plugin failed to plan: "Costmap timed out waiting for update"

which then triggers the behaviour tree's clear-costmap recovery in a loop. The
symptom is a robot that will not move at all, several layers away from the
cause. It cost a full debugging session to find, and the fix by hand is one
command, so it may as well be automatic.

WHY IT WATCHES RATHER THAN WAITS A FIXED TIME
The race is load-dependent: it happens when nvblox is allocating its voxel
layers and cuVSLAM is initialising, and those take between one and two minutes
with no predictable ordering. A TimerAction would be guessing, which is the
mistake origin_anchor_node.py documents having made. This polls the node's own
reported state and acts on the condition that matters.

It is idempotent: when the manager wins the race, this sees ACTIVE and does
nothing at all.
"""

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Bool

ACTIVE = 'active'
INACTIVE = 'inactive'
UNCONFIGURED = 'unconfigured'


class MapServerGuard(Node):

    def __init__(self):
        super().__init__('map_server_guard')
        self.declare_parameter('target_node', 'map_server')
        # Long enough to cover nvblox's ~2 minute allocation, since that is
        # what the transition is losing the race against.
        self.declare_parameter('timeout_s', 240.0)
        self.declare_parameter('period_s', 3.0)

        self.target = self.get_parameter('target_node').value
        self.timeout_s = self.get_parameter('timeout_s').value
        self.t0 = self.get_clock().now()
        self.done = False
        self.nudged = 0

        self.ok_pub = self.create_publisher(
            Bool, '~/map_active',
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_cli = self.create_client(GetState, '/%s/get_state' % self.target)
        self.set_cli = self.create_client(ChangeState, '/%s/change_state' % self.target)
        self.create_timer(self.get_parameter('period_s').value, self.tick)
        self.get_logger().info('watching %s until it reports active' % self.target)

    def tick(self):
        if self.done:
            return
        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        if not self.get_cli.service_is_ready():
            if elapsed > self.timeout_s:
                self.fail('%s never offered get_state' % self.target)
            return
        self.get_cli.call_async(GetState.Request()).add_done_callback(self.on_state)
        if elapsed > self.timeout_s:
            self.fail('%s never reached active within %.0f s'
                      % (self.target, self.timeout_s))

    def on_state(self, fut):
        if self.done:
            return
        try:
            label = fut.result().current_state.label
        except Exception as e:                        # noqa: BLE001
            self.get_logger().warn('get_state failed: %r' % e)
            return
        if label == ACTIVE:
            self.done = True
            if self.nudged:
                self.get_logger().info(
                    '%s is active (it needed %d nudge(s) -- the lifecycle '
                    'manager lost the startup race)' % (self.target, self.nudged))
            else:
                self.get_logger().info('%s is active' % self.target)
            self.ok_pub.publish(Bool(data=True))
            return
        # Not active: push it one step towards active and check again next tick.
        if label == UNCONFIGURED:
            tid = Transition.TRANSITION_CONFIGURE
        elif label == INACTIVE:
            tid = Transition.TRANSITION_ACTIVATE
        else:
            return                                    # transitioning; let it be
        self.nudged += 1
        self.get_logger().warn(
            '%s is %s, not active; requesting %s'
            % (self.target, label, 'configure' if label == UNCONFIGURED else 'activate'))
        req = ChangeState.Request()
        req.transition.id = tid
        self.set_cli.call_async(req)

    def fail(self, why):
        self.done = True
        self.get_logger().error(
            'map NOT served: %s. The global costmap has no static layer, so '
            'the planner will report "Costmap timed out waiting for update" '
            'and the robot will not move.' % why)
        self.ok_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = MapServerGuard()
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
