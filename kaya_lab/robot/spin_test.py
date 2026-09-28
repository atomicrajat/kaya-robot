"""Spin in place at 0.5 rad/s for 6 s and compare the yaw change measured by the gyro
(integrated, bias from 1.5 s standing still), the wheels (/odom) and cuVSLAM (TF).
Checks the gyro's sign and scale before trusting it for odometry. Ends with zero twist."""
import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu

from ball_policy_node import tf_matrix, yaw_of

rclpy.init()
n = Node("spin_test")
q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
tf = tf2_ros.Buffer()
tf2_ros.TransformListener(tf, n)
st = {"R": None, "cal": [], "bias": None, "yaw": 0.0, "t": None, "wy": None}


def imu(m):
    if st["R"] is None:
        try:
            st["R"] = tf_matrix(tf.lookup_transform("base_link", m.header.frame_id, Time()))[:3, :3]
        except tf2_ros.TransformException:
            return
    w = st["R"] @ np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
    if st["bias"] is None:
        st["cal"].append(w[2])
        st["t"] = t
        return
    if st["t"] is not None and 0 < t - st["t"] < 0.1:
        st["yaw"] += (w[2] - st["bias"]) * (t - st["t"])
    st["t"] = t


def odom(m):
    st["wy"] = yaw_of(m.pose.pose.orientation)


n.create_subscription(Imu, "/camera/gyro_accel/sample", imu, q)
n.create_subscription(Odometry, "/odom", odom, 10)
pub = n.create_publisher(Twist, "/cmd_vel", 10)


def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.01)


def vyaw():
    return yaw_of(tf.lookup_transform("odom", "base_link", Time()).transform.rotation)


t_wait = time.time()
while (st["wy"] is None or len(st["cal"]) < 150) and time.time() - t_wait < 15:
    spin(0.1)
if st["wy"] is None or len(st["cal"]) < 150:
    raise SystemExit(f"not ready: wheel odom {'ok' if st['wy'] is not None else 'MISSING'}, gyro samples {len(st['cal'])}")
st["bias"] = float(np.mean(st["cal"]))
print(f"gyro raw z samples: mean {math.degrees(st['bias']):+.4f} deg/s, std {math.degrees(float(np.std(st['cal']))):.4f} deg/s")
print(f"gyro bias {math.degrees(st['bias']):+.3f} deg/s from {len(st['cal'])} samples")
unw = {"wheel": [st["wy"], 0.0], "vslam": [vyaw(), 0.0]}


def track():
    for k, f in (("wheel", lambda: st["wy"]), ("vslam", vyaw)):
        y = f()
        unw[k][1] += math.remainder(y - unw[k][0], 2 * math.pi)
        unw[k][0] = y


g0 = st["yaw"]
cmd = Twist()
cmd.angular.z = 0.5
t0 = time.time()
while time.time() - t0 < 6.0:
    pub.publish(cmd)
    spin(0.1)
    track()
for _ in range(10):
    pub.publish(Twist())
    spin(0.1)
    track()
spin(1.0)
track()
print(f"yaw change  gyro {math.degrees(st['yaw'] - g0):+7.1f} deg   wheels {math.degrees(unw['wheel'][1]):+7.1f} deg"
      f"   cuVSLAM {math.degrees(unw['vslam'][1]):+7.1f} deg   (commanded ~ +{math.degrees(0.5 * 6):.0f})")
