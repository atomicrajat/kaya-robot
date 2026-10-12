# Chapter 2: seeing where you are

*Part of the Kaya learning narrative. Index: [`../../LEARNINGS.md`](../../LEARNINGS.md).
Sources: commits `d475b7d`, `8937ce0`, `3c562db`, `bcd05d2`, `1dec231`, `1ed1f18`, `64a9087`,
`ab1cb7b`, `cc4b040`, `eaaa63a`, `e8fb2bd`, `9b3a4ce`, `523b533`, `b2fd345`, `3aa071a`, `482c450`,
`d1571e1`, `a820bbd`; `kaya_lab/LEARNING_LOG.md` "The frame-sync bug".*

## 2.1 The words

**Odometry** is an estimate of how far the robot has moved since it started,
built up step by step. Wheel odometry integrates encoder counts; it drifts
slowly and never jumps, but it cannot know when the wheels slipped. **Visual
odometry** tracks features from frame to frame and estimates the camera's
motion from how they move. **Visual-inertial odometry (VIO)** adds the IMU,
which fills in the motion between frames at 200 Hz and makes rotation
estimates far more stable. **SLAM** (simultaneous localisation and mapping)
is odometry plus a map of landmarks, so that returning to a known place can
correct the accumulated drift: **loop closure**. The result is the TF chain
from Chapter 1: `odom -> base_footprint` is the smooth, drifting odometry
estimate; `map -> odom` is the correction SLAM applies on top of it.

## 2.2 Two routes, and why the second one won

**Route 1, RTAB-Map on RGB-D.** A forgiving, well-documented pipeline that
consumes colour plus depth and the robot's own wheel-and-IMU odometry. It
produced a map quickly and became the reference against which route 2 was
checked. It also taught two things the hard way, both found while chasing
loop closures that matched in 2D and failed geometric verification every
time ("0/20 inliers from 105 matches", twice, reproducibly):

- **Distorted images were being fed as if rectified.** The camera's factory
  calibration carries real lens distortion and the pipeline expects it
  removed. Enabling the driver's undistortion fixed it; no checkerboard was
  ever needed.
- **The software depth-to-colour aligner could not keep up** at 1280x720
  against the depth sensor's 848x480, and silently passed 15% of frames
  through unaligned while advertising the wrong intrinsics. Matching colour
  to the native 848x480 made registration one-to-one: 483 of 483 frames.

**Route 2, cuVSLAM on the stereo IR pair.** NVIDIA's VIO consumes the two
infrared imagers directly. Verified on this camera before writing a line:
both streams 848x480 mono, already rectified (distortion coefficients all
zero), a 94.9 mm baseline matching the specification, and hardware stereo
synchronisation (0.18 ms median stamp delta). There is no registration step,
no software aligner and no resolution mismatch to get wrong, because both
images come off one sensor pair on one clock. Tracking cost 6.1 ms per frame
against RTAB-Map's 160 ms per iteration, a 26x difference, and that headroom
is what later let SLAM, mapping and planning share the 8 GB board.

The IMU noise parameters fed to cuVSLAM were derived from this camera's own
measured noise floor, not copied from a RealSense example. Raw IMU samples go
in, not the bias-corrected stream from Chapter 1, so there are not two bias
corrections fighting.

**The projector stays off.** The camera has an IR dot projector for depth.
Its pattern is fixed to the camera, so a feature tracker locking onto the dots
concludes the robot is not moving. That reasoning was then tested rather than
trusted (2.6 below).

## 2.3 The regression that hid at rest

This is the project's central lesson, and it deserves the full story.

On the morning of 2026-09-07 cuVSLAM returned a 16.4 m loop to 2.2 cm, with
1.2% raw drift. Every run after that drifted 14.5-32%. The investigation
blamed, in order, the room being dark in infrared, the IR projector, the
scene contents, and a ground-plane constraint. Each was measured. Each was
wrong. Eventually the VIO configuration was diffed against the last commit
known to work, and one substantive line had changed: the camera driver's
`time_domain`, from `global` to `system`.

That knob looks like a timestamp-freshness setting. It also decides whether
the IMU and the images share a time base. Measured on the camera alone, so
both settings faced the same load:

| | image interval std | IMU interval std | IMU max deviation |
|---|---|---|---|
| `global` | 0.001 ms | 0.004 ms | 0.015 ms |
| `system` | 0.107 ms | 5.474 ms | **11.374 ms** |

The IMU samples every 4.93 ms. Under `system` its stamps deviated by up to
11.4 ms, more than two whole sampling periods. A visual-inertial estimator
integrates those samples between frames; mis-stamped samples produce rotation
and velocity error **proportional to the actual motion**. Nothing at rest,
ruinous while driving. That is exactly why it was so hard to see: a
stationary robot held 1.4 mm over 94 s, and every static check passed. IMU
rate, stereo pairing (419 of 419 matched), calibration, baseline, stamp
latency, sharpness, exposure, tracking rate, `vo_state`. All healthy. Every
driven loop failed.

How the change had been justified is the lesson itself. The commit that
introduced `system` said: "Measured cost to VIO of switching: none
detectable. `vo_state` stayed 1 and track time was 10.8-15.3 ms against
11.1 ms." Both numbers were real. Neither measures accuracy. A drifting
estimator reports `vo_state = 1` at full frame rate right up until it is
metres wrong. **The change was validated against instrumentation that could
not detect the harm it did.**

Reverting it and driving a 16.92 m loop gave 0.63% raw drift, a 23-50x
improvement and better than the 1.2% of the original good run.

## 2.4 The lighting theory, and its retraction

The first explanation for the bad runs was that the room was dark in
infrared: LED lighting emits almost none, and the IR imagers were running
without the projector. A texture statistic (how much the image has to track
on) read 4.6 with the projector off and 11.8 with it on. The morning's good
run had been in daylight. The story fit.

Then the projector was tested rather than reasoned about:

| condition | raw VIO drift | loop closure | return error |
|---|---|---|---|
| daylight, projector off | 1.2% | fires | 2.2 cm over 16.4 m |
| night, projector off | 15% | cannot fire | 3.03 m over 20 m |
| night, projector on | 14.5% | fires | 4 mm over 8.5 m |

The projector restored **place recognition** (its dots deform with the real
geometry, so a revisited spot looks the same again) but did nothing for raw
odometry. And once `time_domain` was fixed, the daylight run at 0.63% drift
had texture 4.45, the same as every failing run; daylight and night both
measured 4.4. Indoor IR texture is identical by day and night. So "VIO needs
daylight" and "the projector is needed at night" were both artefacts of the
timestamp bug, and the repo records their retraction as carefully as it
recorded them. Texture does predict trackability (a blank wall is
untrackable), but it was never what changed.

## 2.5 A constraint that stated a true fact and made things worse

A wheeled robot on a flat floor has three degrees of freedom, not six, and
the estimator was drifting in all six: one run put the robot 1.8 m below the
floor it stood on. cuVSLAM has a ground-plane constraint for exactly this.
It was turned on, and raw `odom -> base_footprint` Z read 0.000 as designed.

Measured on a driven loop, it doubled the drift (14.5% to 32%), and the Z
error reappeared in `map -> odom`, where the constraint does not reach. The
likely mechanism: this robot does pitch and roll a little, on omni rollers
over a real floor under acceleration. Forcing those to zero tells the
estimator something false, and the inconsistency has nowhere to go but
planar motion. **Over-constraining injects error rather than removing it.**
Left off, with the numbers recorded at the parameter.

## 2.6 The camera pipeline is a shared resource

Four separate faults had the same shape: something consumed camera bandwidth
or time, and the cost showed up not as a CPU number but as navigation error.

**The point cloud.** `/camera/depth/points` was published at full 848x480
(407k points) for exactly one consumer. Measured stamp lag across the whole
camera pipeline, full stack running:

| stream | before | after 4x decimation |
|---|---|---|
| depth image | 0.468 s | 0.030 s |
| depth points | 0.508 s | 0.043 s |
| **left IR** | 0.464 s | 0.030 s |

The third row is the one that matters: the cloud was starving the IR stream
cuVSLAM tracks on. The first instinct had been to widen the collision
monitor's timeout to stop it complaining about stale data; that would have
silenced the message while making the safety layer reason about 1.8 s old
data.

**Colour resolution.** Colour at the driver's default 1280x720 against depth
at 848x480 cost about 15% of depth frames in the software aligner, the second
time this exact mismatch had cost the project, because resolution is set per
launch file and the lesson did not transfer.

**Colour against the stereo pair.** With the full stack running, the left IR
ran at 24.5 Hz with a 66.6 ms 90th-percentile gap against the right's 28.8 Hz
and 35.0 ms. cuVSLAM needs matched pairs, so the slower eye set the true
tracking rate. The pairing tool reported 0% unmatched, and it was right: every
surviving left frame did have a partner. Only comparing the two eyes' rates
exposed it. The cost of colour is linear, measured on one 3.5 m goal:

| colour | final error |
|---|---|
| off | 6.3 cm |
| 5 fps | 10.5 cm |
| 15 fps | 14.7 cm |
| 30 fps | ~20 cm |

Navigation runs colour off; the dashboard shows the IR stream, which is
already running and costs nothing.

**Exposure.** A session went into a "camera stall under load", chased through
memory, nvblox and colour. None of them. The driver ships IR auto-exposure
with no upper bound, and in ordinary indoor light it settled near 50 ms. A
sensor cannot expose for longer than its frame period, so the pair could not
exceed about 20 Hz whatever rate was requested; the 51 ms gap between frames
*was* the exposure time. Capping it at 20 ms took the left IR from 18.0 to
28.4 Hz. The launch system then silently dropped the parameter (accepted, not
applied, still reading `true` after three relaunches), so a small node sets
it after the stream is up and reads it back, logging an error on mismatch. A
tuner that silently failed would recreate the problem it exists to fix.

**A dictionary key set twice.** The launch file passed `enable_frame_sync`
twice in one Python dict: `false`, with a comment saying the IR pair must not
wait for depth, and an older `true` further down. The later key wins. For 17
days the pair had been buffered behind the slower depth stream. flake8 found
it (F601). An A/B test proved what it cost: the average rate was 30 Hz
either way, but the gap between IR frames was 5.7 ms median / 190 ms worst
with sync on against 33.6 / 73 ms off, and the pose estimate's age fell from
about 180 ms to 101 ms. **Averages hide bursts; measure the distribution.**

## 2.7 What could not be fixed

With depth enabled, the IR pair arrives about 0.47 s late. Eight software
knobs were tried and ruled out one by one (time domain, host-time sync,
registration, frame sync, the point cloud, and more); the wheel TF on the same
board stamps at 0.006 s. The depth engine runs on the camera and the IR
frames come out behind it. The stack is configured for the latency that
exists (Chapter 3 shows what that means for Nav2), and the limit is recorded
rather than hidden.

## 2.8 Two things that bite once

**After a hard power loss the GPU may not come back.** The secure-firmware
bootstrap can fail to set up its carveout; CUDA reports zero devices while
the camera streams at 29.8 Hz and looks fine. `dmesg | grep -i nvgpu` shows
ACR errors; a clean reboot fixes it. Check this before suspecting software.

**VIO does not survive being carried.** A test after a manual reposition
started 49 m below the floor with 176 degrees of roll already accumulated.
Restart the stack after moving the robot by hand.
