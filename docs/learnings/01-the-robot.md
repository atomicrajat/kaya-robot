# Chapter 1: the robot, and the constraint that shaped everything

*Part of the Kaya learning narrative. Index: [`../../LEARNINGS.md`](../../LEARNINGS.md).
Sources: `robot_setup/SETUP_LOG.md`, `kaya_description/cad/CAD_CONTEXT.md`,
commits `735fc6d`, `ef20a4d`, `371f15b`, `b73f12f`, `41b051b`, `d249280`, `17ba587`, `5f5fa43`, `67400b9`.*

## 1.1 What Kaya is

Three omni wheels at 60, 180 and 300 degrees around a triangular chassis, each
on a Waveshare ST3020 serial-bus servo. An omni wheel has free rollers around
its rim, so it can be driven along its own direction and slide freely
sideways. Three of them at 120-degree spacing give a **holonomic** base: it can
translate in any direction and rotate, all independently, without turning
first. The brain is a Jetson Orin Nano. The only sensor is an Orbbec Gemini
335L camera: two infrared (IR) imagers for stereo, a depth output computed on
the camera, a colour imager, and a six-axis IMU (gyroscope plus accelerometer)
at about 200 Hz. There is no LiDAR, no wheel-less odometry, no magnetometer.

## 1.2 The constraint: 8 GB, one GPU, no accelerator

The carrier board's device tree calls itself an "Orin NX Engineering Reference
Developer Kit". That is the carrier, which is shared across module families.
The module was identified from the silicon: byte 0x10 of the fuse block reads
`0xD5`, which is the Orin Nano 8 GB SKU. Three facts that do not depend on
any EEPROM agree: 8 CUDA streaming multiprocessors, 7.44 GB of RAM, and zero
DLA cores (every Orin NX has two).

Why this matters for everything after: about 40 TOPS, 8 GB shared between
CPU and GPU, and no deep-learning accelerator to push inference onto. Visual
odometry, 3D mapping, planning and any perception all compete for one GPU
and one memory pool. The project's budget numbers, measured later with the
full navigation stack running, were 3.7 GB used of 7.5, with cuVSLAM at 7-9
ms per frame and nvblox holding 1.2 GB. Every later decision about what runs
where comes back to this.

## 1.3 Lesson: the power mode that reports itself and does nothing

The GPU came up capped at 624.75 MHz with only 15 W and 7 W modes offered.
The widely posted fix is to symlink the "super" nvpmodel configuration and run
`nvpmodel -m 2`. It reports `MAXN_SUPER`. The clocks do not change, and a
reboot regenerates the symlink from the device tree. Tested on JetPack 6.2.1
and 7.2.1.

The frequency table is the honest witness:

    cat /sys/devices/platform/bus@0/*.gpu/devfreq/*.gpu/available_frequencies
    before:  306 ... 624.75 MHz
    after:   306 408 510 612 714 816 918 1020 MHz

The higher points were **absent, not capped**. They live in the BPMP firmware
device tree, so no userspace change can create them. Only reflashing with the
`jetson-orin-nano-devkit-super` board config adds them (not the `-maxn`
variant, which exits on this SKU). The lesson is the first instance of the
pattern that runs through the whole project: a tool that reports a number is
not a tool that reports the truth. `nvpmodel -q` reports what was requested;
the clock table reports what exists.

## 1.4 Setup faults worth knowing before they cost a day

Each of these is written up with its fix in `robot_setup/SETUP_LOG.md`, and
`robot_setup/kaya_provision.sh` applies them as stages on a fresh flash.

- **A plain `colcon build` locks the board out of ssh.** Six workers times
  unbounded compiler jobs is about 36 C++ compilations on 8 GB; the OOM killer
  fired 17 times. `MAKEFLAGS="-j2" colcon build --parallel-workers 1` builds
  the camera driver in 3m37s; unconstrained it ran 37 minutes and failed.
- **JetPack 7 ships no Intel wifi driver**, and Canonical's extra-modules
  package has the wrong vermagic. `iwlwifi` is built from NVIDIA's kernel
  source, one directory only, in about a minute.
- **The built driver loads and the card stays dead** because JetPack 7's
  kernel cannot read `.zst` compressed firmware and says nothing about it.
  `unzstd` the two `.ucode.zst` files.
- **The camera refuses to initialise** on an unguarded property read in the
  Orbbec ROS driver (`OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT`, which this
  firmware rejects). One `try/catch`, and it is lost on every re-clone.
- **`isaac-ros-nvblox` cannot be installed** on Jetson because the
  metapackage pulls an x86-only Triton server. The core nvblox packages
  install directly; only the demo launch files are lost.
- **Servos and the IMU look absent** on a fresh user that is not in `dialout`
  and `i2c`.
- **The bootloader cannot flash NVMe directly** on this board; the initrd
  method, which boots a real kernel first, is what SDK Manager uses.

## 1.5 Frames: where everything is, relative to what

ROS keeps a tree of coordinate frames, TF. Every reading and every goal is
expressed in some frame and the tree converts between them. The conventions
(REP-103, REP-105): X forward, Y left, Z up; and the chain
`map -> odom -> base_footprint -> base_link -> sensors`. `base_footprint` is
the robot's rotation centre projected to the floor; `base_link` is the same
point at axle height, one wheel radius (37 mm) above it.

**One parent per frame.** The camera driver publishes its own internal chain
(`camera_link -> camera_depth_frame -> ... -> camera_accel_gyro_optical_frame`),
so the robot description publishes exactly one transform,
`base_link -> camera_link`, and nothing below it. Two publishers for one
frame give a transform that flickers between two values.

**The fault that split the tree.** cuVSLAM, the visual odometry, published
`odom -> base_link`. The robot description already parented `base_link`
under `base_footprint`. Two parents; TF permits one; the tree split into two
unconnected halves and Nav2 would not start. One line fixed it: cuVSLAM
anchors to `base_footprint`. A side effect worth noticing: an earlier commit
had shifted nvblox's obstacle heights by 37 mm to compensate for "the map
origin sitting one wheel radius off the floor". That offset was a symptom of
this bug, not a fact about the robot, and was reverted once the tree was
right.

**`camera_link` is the left IR lens, not the camera body.** The driver's
`camera_link -> camera_depth_frame` is the identity, and depth is computed in
the left IR imager, so the driver means the lens. The description had placed
`camera_link` at the body centre, 47.5 mm to the right. A water bottle on the
robot's centre line at 0.5 m and 1.0 m measured -44.5 and -38.9 mm with the
old transform and +3.0 / +8.6 mm with the new one. Every depth point had been
about 5 cm right of the truth throughout the navigation project; it was found
only while building the simulator, where the error showed up as a mismatch
against a known geometry.

**The roll sign.** The camera's pitch (4.89 degrees nose-up) and roll (0.94
degrees) came from averaging the IMU's gravity vector, which is the right
method. The roll's sign was lost converting into REP-103. It was caught when
the simulated IMU at rest did not match the real one until the sign flipped.

**One source of truth for every dimension.** `cad/kaya_dimensions.yaml` holds
each measurement and how it was taken; the URDF, the Fusion model and the
Isaac Sim asset are generated from it. Typed separately, they drift, and the
drift looks like a physics problem when it is a typo.

## 1.6 The omni drive

For a wheel at angle θ around the chassis at contact radius L, its rolling
speed for a commanded body velocity (vx, vy) and yaw rate ω is

    v_i = -sin(θ_i) vx + cos(θ_i) vy + L ω

Three wheels, three equations. Inverting them turns three encoder deltas into
body motion, which is wheel odometry. Two geometry facts had to be got right
before the equation meant anything:

- **The axle is radial.** The servo shaft points straight out from the
  chassis, so the wheel rolls tangentially. The first description had the
  axle tangential, which would have the wheels rolling radially; caught from
  the photographs.
- **Which servo is which wheel.** Two earlier guesses were wrong and each
  carried a warning saying it was unverified. Resolved by pulsing each servo
  alone in wheel mode at low speed and watching: servo 1 is front-right at
  300 degrees, 2 is front-left at 60, 3 is rear at 180, and a positive Goal
  Speed matches the equation's sign on all three. The script that did it is
  `kaya_drive/tools/wheel_probe.py`.

**L is calibrated, not measured.** The CAD contact radius is 143 mm. Spinning
the robot about ten turns and comparing the odometry's rotation with the
counted truth gave 138 mm. The difference is slip. Both numbers are kept, as
different quantities: 138 for the real drive, 143 for the simulator, which has
real geometry and no unmodelled slip.

**The servo bus.** The vendor SDK's `clearPort()` flushes only the output
buffer. EEPROM-area writes on this servo (switching Mode, for instance) can
acknowledge late; that acknowledgement then sits in the input buffer and is
read as the reply to the next request, with a checksum byte landing where a
data byte should be. Every request now flushes input immediately before
sending. Also confirmed by experiment: Goal Speed and Present Position share
one unit, raw encoder counts per second on a 4096-count encoder.

**Two safety habits built in.** The drive zeroes the wheels 0.5 s after the
last command arrives. And a stack killed with SIGKILL leaves the servos in
wheel mode with torque enabled, holding against the floor; `kaya_stop`
finishes by releasing them over the bus and reading Mode back, so its exit
status reflects the hardware rather than the assumption that a signal was
delivered.

## 1.7 The IMU, and why bias matters

The build had an external MPU-6050. Measured side by side at rest against the
camera's built-in IMU:

| | MPU-6050 | Gemini 335L |
|---|---|---|
| gravity magnitude error | +50.9% | -0.2% |
| accelerometer noise, worst axis | 0.042 m/s² | 0.0064 m/s² |
| gyro bias | 6.06 deg/s | 0.56 deg/s |
| heading drift, uncorrected | 367 deg/min | 38.9 deg/min |
| drift after bias removal | - | **0.19 deg/min** |

The camera's IMU also stamps its samples on the device: 0.004 ms of
inter-stamp jitter against 4.7 ms of inter-arrival jitter, which matters once
it is fused with images. The MPU-6050 was unbolted and its driver deleted.

**Bias** is the small non-zero rate a gyroscope reports while still. Integrated
into heading, it is a drift that grows with time, and it walks as the sensor
warms. On this camera the yaw-axis bias moved by 0.00203 rad/s and changed
sign over about 30 minutes, which alone is 7 degrees per minute of heading
error from a calibration taken at start-up. It was badly misleading while
debugging mapping: after a four-minute drive the robot was back at its start
and odometry reported a 21-degree heading change still growing while parked.

Three fixes, each measured on the stationary robot:

| change | stationary yaw drift |
|---|---|
| gyro only, bias from start-up | 2.25 deg/min |
| fuse wheel yaw rate alongside the gyro (an unbounded integral needs an anchor) | 0.83 deg/min |
| re-estimate bias whenever the gyro itself says the robot has been still for 3 s | 0.22 deg/min |

The last of these is a real technique (zero-velocity updates), applied in the
driver rather than inside the estimator. The proper form carries gyro bias as
an estimated state, which every serious visual-inertial system does and the
wheel-plus-IMU filter used here structurally cannot. That is why Chapter 2
replaces the filter with cuVSLAM: not because the filter was badly tuned, but
because the architecture could not do the one thing that mattered.
