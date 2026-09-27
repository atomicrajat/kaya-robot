# kaya

ROS 2 Jazzy packages for the Kaya omni-wheel robot (Jetson Orin Nano 8GB, JetPack 7.2.1).

See `ROADMAP.md` for the plan and the next tests, and `ENVIRONMENT.md` for the
development machine — the project moved from the MacBook to the Ubuntu
workstation on 2026-09-08.

| Package | Purpose |
|---|---|
| `kaya_imu` | IMU conditioning. `camera_imu_node` conditions the Gemini 335L's built-in IMU. (The external MPU-6050 was physically removed; its driver was deleted 2026-09-27 and is in git history.) |
| `kaya_description` | URDF/xacro. Publishes `base_link -> camera_link`; the Orbbec driver supplies the rest of the camera chain. |
| `kaya_bringup` | Launch composition. |

**Rebuilding the robot:** `robot_setup/` - provisioning, health check,
pre-flash backup, and the full setup log.

**Simulation:** Kaya as an Isaac Sim robot, calibrated against this one -
`kaya_description/usd/README.md`. The measurements behind the URDF and the
sim are in `kaya_description/cad/kaya_dimensions.yaml`.

## Phase 1 (complete)

    ros2 launch kaya_bringup imu.launch.py
    ros2 service call /imu/rezero_gyro std_srvs/srv/Trigger

`/imu/data_raw` at 203 Hz, bias-corrected, measured covariances, full TF from
`base_footprint` to `camera_accel_gyro_optical_frame`.

## IMU selection

Measured side by side, stationary:

| | MPU-6050 | Gemini 335L |
|---|---|---|
| `\|a\|` error | +50.9 % | -0.2 % |
| accel noise (worst axis) | 0.042 m/s^2 | 0.0064 m/s^2 |
| gyro bias | 6.06 deg/s | 0.56 deg/s |
| drift, uncorrected | 367 deg/min | 38.9 deg/min |
| drift, after bias removal | - | **0.19 deg/min** |

Neither has a magnetometer. Absolute heading comes from VSLAM loop closure
(Phase 4), not a compass -- indoor magnetic heading is corrupted by building
steel and by the robot's own servo magnets.

Camera IMU timestamps are device-generated: inter-stamp jitter 0.004 ms vs
inter-arrival jitter 4.694 ms. Suitable for VIO.

## TODO before Phase 2

Every dimension in `kaya_description/urdf/kaya.urdf.xacro` is a placeholder.
Measure chassis radius, wheel radius, and the camera mount offset.
