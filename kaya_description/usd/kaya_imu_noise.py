"""The real Gemini 335L IMU's errors, to make simulated IMU data look real.

Isaac Sim's IMU is perfect. The real one, measured on the robot at rest
(30 s, 6622 samples at 203 Hz, 2026-09-27, optical frame):

    accel  white noise sd  0.0073 / 0.0098 / 0.0090 m/s^2 (x, y, z)
           gravity reads   9.793 m/s^2 - a 0.17% scale error
    gyro   white noise sd  0.00051 / 0.00054 / 0.00046 rad/s
           bias            +0.0099 / +0.0002 / +0.0053 rad/s, steady over the
                           30 s and within 0.0005 of a recording a month
                           earlier (2026-08-28) - a fixed offset, not drift

Apply to a reading already in camera_accel_gyro_optical_frame (which is how
the sim IMU is mounted). For training, randomise around these rather than
using them exactly - they are one camera on one day.
"""

import numpy as np

ACCEL_SD = np.array([0.0073, 0.0098, 0.0090])
ACCEL_SCALE = 9.793 / 9.81
GYRO_SD = np.array([0.00051, 0.00054, 0.00046])
GYRO_BIAS = np.array([0.0099, 0.0002, 0.0053])


def corrupt(accel, gyro, rng=np.random.default_rng()):
    """Perfect sim accel/gyro (3-vectors) -> what the real sensor would say."""
    accel = np.asarray(accel) * ACCEL_SCALE + rng.normal(0.0, ACCEL_SD)
    gyro = np.asarray(gyro) + GYRO_BIAS + rng.normal(0.0, GYRO_SD)
    return accel, gyro
