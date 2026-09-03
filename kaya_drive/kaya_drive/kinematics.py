"""Three-wheel omni kinematics for the Kaya robot.

theta_i is the wheel's body angle from +X, CCW-positive (REP-103: +X forward,
+Y left, +Z up). Confirmed layout (see kaya.urdf.xacro and CAD_CONTEXT.md
Sec. "Answered 2026-09-03"):

    servo 1 -> front-right -> theta = -pi/3  (300 deg)
    servo 2 -> front-left  -> theta =  pi/3  ( 60 deg)
    servo 3 -> rear        -> theta =  pi    (180 deg)

    v_i = -sin(theta_i)*vx + cos(theta_i)*vy + L*w

where v_i is the wheel's rolling (tangential) speed, positive in the
direction that a positive-yaw (CCW, viewed from above) rotation would push
that contact point. All three servos' raw Goal Speed sign already matches
this convention with no per-wheel flip needed.
"""

import numpy as np


class OmniKinematics:

    def __init__(self, thetas, chassis_radius, wheel_radius):
        """thetas: {servo_id: theta_rad}, in a fixed iteration order."""
        self.servo_ids = list(thetas.keys())
        self.chassis_radius = chassis_radius
        self.wheel_radius = wheel_radius
        # Row i: [-sin(theta_i), cos(theta_i), L] maps a body twist
        # (vx, vy, w) to wheel i's rolling speed v_i.
        self.M = np.array([
            [-np.sin(thetas[sid]), np.cos(thetas[sid]), chassis_radius]
            for sid in self.servo_ids
        ])
        self.M_inv = np.linalg.pinv(self.M)

    def twist_to_wheel_speeds(self, vx, vy, w):
        """Body twist (m/s, m/s, rad/s) -> {servo_id: wheel rolling speed m/s}."""
        v = self.M @ np.array([vx, vy, w])
        return dict(zip(self.servo_ids, v))

    def wheel_speeds_to_twist(self, wheel_speeds):
        """{servo_id: wheel rolling speed m/s} -> (vx, vy, w), forward kinematics.
        Requires all three wheels -- a missing reading is a dropped serial
        transaction, not a zero speed, and feeding it in as zero would bias
        the estimate. Callers should skip the odometry update for a cycle
        instead of calling this with an incomplete reading."""
        v = np.array([wheel_speeds[sid] for sid in self.servo_ids])
        vx, vy, w = self.M_inv @ v
        return float(vx), float(vy), float(w)
