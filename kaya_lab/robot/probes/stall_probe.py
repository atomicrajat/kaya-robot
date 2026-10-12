"""Stall-detection measurement: drive slowly forward into a wall and log what each
signal does before and after contact. Place the robot ~40 cm from a wall, facing it,
with no ball.

    python3 stall_probe.py [speed_mps=0.08] [seconds=9]

Logged at 10 Hz:
  wheel_vx   forward speed from the wheel encoders (drive_node /odom twist)
  load_1..3  each servo's present load, -1..1 of max torque (/joint_states effort)
  acc_x      forward acceleration, IMU (bias removed from 1 s standing still)
  vio_vx     forward speed from cuVSLAM (/visual_slam/tracking/odometry)
Ends with a zero twist.
"""
import os
import sys

# One-off probes live one level below the node. Make the node's modules (ball_policy_node,
# depth_scan, goal_marker, yolo_trt) and kaya_lab/ball_detector importable without copying.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(_HERE, os.pardir),
                os.path.join(_HERE, os.pardir, os.pardir, "kaya_lab")]
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState

from ball_policy_node import tf_matrix

SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.08
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 9.0

rclpy.init()
n = Node("stall_probe")
be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
tf = tf2_ros.Buffer()
tf2_ros.TransformListener(tf, n)
st = {"wheel_vx": None, "load": None, "acc": [], "R": None, "vio_vx": None, "cal": []}


def on_odom(m):
    st["wheel_vx"] = m.twist.twist.linear.x


def on_js(m):
    if len(m.effort) == 3:
        st["load"] = list(m.effort)


def on_vio(m):
    st["vio_vx"] = m.twist.twist.linear.x


def on_imu(m):
    if st["R"] is None:
        try:
            st["R"] = tf_matrix(tf.lookup_transform("base_link", m.header.frame_id, Time()))[:3, :3]
        except tf2_ros.TransformException:
            return
    a = st["R"] @ np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    st["acc"].append(a[0])


n.create_subscription(Odometry, "/odom", on_odom, 10)
n.create_subscription(JointState, "/joint_states", on_js, 10)
n.create_subscription(Odometry, "/visual_slam/tracking/odometry", on_vio, be)
n.create_subscription(Imu, "/camera/gyro_accel/sample", on_imu, be)
pub = n.create_publisher(Twist, "/cmd_vel", 10)


def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.01)


t_wait = time.time()
while (st["load"] is None or st["wheel_vx"] is None or len(st["acc"]) < 150) and time.time() - t_wait < 15:
    spin(0.1)
if st["load"] is None:
    raise SystemExit("no load readings in /joint_states effort - is the new drive_node running?")
acc_bias = float(np.mean(st["acc"][-150:]))
print(f"still: load {np.round(st['load'], 3)}, acc_x bias {acc_bias:+.3f} m/s^2")
print("   t   wheel_vx  vio_vx   acc_x   load_1  load_2  load_3")
cmd = Twist()
cmd.linear.x = SPEED
t0 = time.time()
while time.time() - t0 < SECONDS:
    st["acc"].clear()
    pub.publish(cmd)
    spin(0.1)
    acc = (np.mean(st["acc"]) - acc_bias) if st["acc"] else float("nan")
    L = st["load"] or [math.nan] * 3
    vio = st["vio_vx"] if st["vio_vx"] is not None else math.nan
    print(f"{time.time() - t0:5.1f}  {st['wheel_vx']:+7.3f}  {vio:+7.3f}  {acc:+6.2f}  "
          f"{L[0]:+6.3f}  {L[1]:+6.3f}  {L[2]:+6.3f}")
for _ in range(10):
    pub.publish(Twist())
    spin(0.1)
print("stopped")
