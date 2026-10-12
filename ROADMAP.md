# Kaya: where it stands, the design, and the settings that must not drift

**Goal, as set at the start.** Ask the robot, in words, to go somewhere
described by what you *do* there ("show me where I can cook", "take me to the
washbasin") and have it work out where that is, drive there autonomously, and
avoid obstacles on the way. Plus a web dashboard with a live map, live robot
position, and a place to type the question. **That goal was reached on
2026-09-10.** The project then moved on to training the robot in simulation
(`SIM_RL_PLAN.md`), which produced a second working demo on 2026-09-29.

This file is the status, the reasoning behind the design, the record of how
navigation was made to work, what is still open, and the table of settings
that each cost a day. The step-by-step guide is `GETTING_STARTED.md`; the
teaching version of all of it is `LEARNINGS.md`.

---

## Where the project stands (2026-10-12)

| phase | capability | measured |
|---|---|---|
| 1 | IMU conditioning (the camera's IMU, not the MPU-6050) | 0.19 deg/min drift after bias removal |
| 2 | holonomic drive + wheel odometry | forward, strafe and rotate with no cross-talk |
| 3 | wheel + gyro EKF | superseded by cuVSLAM, which estimates gyro bias itself |
| 4 | visual-inertial odometry (cuVSLAM, stereo IR) | **0.63% drift** over 16.9 m |
| 5 | nvblox + Nav2 (omni) | 3.5 m traverses to 6-14 cm; 0.5 m pure strafe, -0.1 deg heading |
| 6 | save a map, relocalise into it | **5.9 cm** from a 5 m wrong hint (afternoon the map was made) |
| 7 | dynamic obstacles | a person is seen while present and leaves no trace in the map |
| 8 | dashboard: live map, robot, planner, camera, click-to-navigate, teleop | runs on the workstation; costs the Jetson nothing |
| 9 | keyframe memory (pose-tagged images every ~1 m / 45 deg) | 62 places for a 202 m² flat |
| 10 | **ask in words** | six acceptance questions, six different correct places; drove 5.1 m to the kitchen to 14 cm |
| 11 | open-vocabulary detection and following (OWLv2 on the workstation) | objects localised in the map to ~1 cm repeatability; follower publishes standoff goals. Prototyped, not polished |
| RL | drive to a point; ball to goal | 100% in 3 min; **real robot 3 goals in 4** (v2.4) |
| 12 | memory and events ("when did you last see...") | not started |

---

## The design: what "show me where I can cook" required

**Remember scenes, let a reasoning model choose.** The map is pure geometry:
it knows a wall is at (3.2, -1.4), not that it is a kitchen. The gap is filled
by remembering *what places looked like* and asking a model which one matches:

    while driving:   keyframe image + its map pose   ->  a few dozen JPEGs
    after a drive:   caption each keyframe ONCE       ->  captions.json
    on a question:   question + captions -> model picks one place
                     -> navigate to that keyframe's pose

This was chosen over object detection and rebuilding meaning from class
labels, which was the first plan:

| | object detection | keyframes + reasoning model |
|---|---|---|
| Jetson cost | YOLO at 1-2 Hz, competes for GPU | saving JPEGs, nearly free |
| vocabulary | COCO's 80 fixed classes | anything describable |
| "somewhere to read" | impossible | works |
| new code on robot | detect -> depth -> TF -> cluster | an image writer |

**Caption once, query many.** Sending every keyframe image to the model per
question would have cost over a minute per question, growing with every
drive. Captioning all 62 keyframes once took 1m58s; a question is then text
against text, about a second, flat in the size of the store. It also makes
the memory inspectable: when the robot says the kitchen is at (-2.06, 4.74),
you can read what it saw there.

**Captions are structured: `here` versus `beyond`.** The first captions sent
the robot to a corridor mouth from which a fridge was visible 2.7 m away. A
fridge in `beyond` is a signpost; a sink in `here` is the kitchen. That split,
and nothing domain-specific, fixed it: a rule like "cooking means a sink" was
tried and made things worse (it answered "washbasin" with a bedroom).

**Where the model runs: the workstation, by latency, not preference.** The
Orin has ~3.9 GB free with the navigation stack up, and a 7B vision model
would compete with cuVSLAM and nvblox for the same 8 GB. Captioning and
answering are one-shot with a human waiting, where a second is not felt;
tracking and following are a control loop, where latency *is* error, and
those stay on the robot. `KAYA_VLM` and `OLLAMA_HOST` move the model without
a code change. The model used is qwen2.5vl:7b under Ollama.

**Answering and driving stay separate.** The dashboard shows the remembered
photo, the pose and the model's reason, and a separate button to go. A model
that is confidently wrong should cost a glance at a picture, not a drive
across the flat. It is also allowed to say no: asked for a swimming pool it
refused rather than picking the nearest room.

## Running this on an Orin Nano 8 GB

**The semantic layer does not need to be fast.** Navigation is real-time and
must stay so; understanding the room can lag by seconds. Measured headroom
with the full navigation stack running: 3.7 GB of 7.5 GB used, nvblox 1.2 GB,
cuVSLAM 7-9 ms per frame.

Rules that followed, each one backed by a failure:

1. **Detect at 1-2 Hz, not 30.** A room does not change fast.
2. **Throttle at the source for off-board consumers.** A workstation
   subscriber on the full-rate depth topic pulled ~20 MB/s over wifi, starved
   `collision_monitor` of its only source, and stopped the robot while the
   follower tracked its target perfectly. `perception_throttle` republishes
   colour, depth and IR at 2 Hz on `/kaya/perception/*`.
3. **Never let anything starve cuVSLAM.** The full-resolution point cloud,
   the colour stream, the depth-to-colour aligner and the IR auto-exposure
   each did, and each showed up as navigation error rather than as a CPU
   number.
4. **Verify against raw VIO drift on a driven loop, not `vo_state`.** Every
   health counter reported perfect health while the estimator drifted 15-32%.
5. **Language and reasoning off-board.**

---

## How navigation got here

### The 3.5 m traverse, fault by fault

Every step below is a measurement on the same goal from the same start.

    .nvblx reloaded, the original config      stalled 1.00 m short
    nvblox lethal band 0.35 -> 0.22 m         0.62 m
    dynamic mapper finally gets the slice     0.43 m
    BT server timeout 20 -> 2000 ms           0.31 m
    nvblox out of the global costmap          full distance, overshoots
    VoxelLayer instead of the ESDF layer      first "Reached the goal!"
    wz_max 1.0 -> 0.5, tolerances from the    three goals reached, stable,
    measured rotational excursion             heading within 1 deg

| fault | what it did | fix |
|---|---|---|
| nvblox layer in the GLOBAL costmap | an ESDF cannot say "I have not looked there", so everything outside the camera cone read as lethal and the planner could only plan inside the FOV | static map globally; nvblox is not a costmap source |
| nvblox ESDF as the local obstacle layer | same blindness, locally | `VoxelLayer` on `/camera/depth/points`: mark, clear, leave unknown alone |
| slice settings under `static_mapper` only | navigation uses the DYNAMIC mapper, whose slice started at 0.0, the floor plane, so polished tile reflections fused as obstacles | give `dynamic_mapper` the same slice |
| `wz_max: 1.0` | the controller satisfied heading by driving arcs; once it rotated fast enough to lose visual tracking and dead-reckoned the pose out past 13 km | 0.5 rad/s |
| `xy_goal_tolerance: 0.10` | tighter than the robot's own 10.9 cm rotational excursion, so heading corrections always broke the position check | 0.15 m, set from that measurement |

**The base was never at fault.** Commanded a pure in-place turn, this robot
closes 360 deg with 1.7 cm of net translation (`tools/spin_translation_test.py`).

### Five Nav2 defaults that stopped the robot from ever moving (Phase 5)

All inherited from `nav2_bringup`'s TurtleBot template, all silent: a global
costmap with its corner at the origin (the robot started outside it); a
velocity smoother with `y = 0.0` that deleted every strafe command after the
`Omni` motion model had produced it; a collision monitor watching a LiDAR
topic nothing published; no `transform_tolerance` against a camera-stamped
TF; and a progress checker demanding 0.5 m of motion for a 0.5 m goal.

### The false "Reached the goal!"

Nav2's `ControllerServer::isGoalReached()` ignores the return value of
`transformPose`. When the transform is too old the output pose stays
default-constructed at (0, 0, 0), so the goal checker compares the robot
against the **origin**, and a robot that has just left its mark is near the
origin. It stopped 23 cm into a 5 m drive and reported success,
reproducibly, to the centimetre. The fix is `transform_tolerance` above the
measured TF lag (1.5 s against 0.47 s), so Nav2 uses the latest transform
instead of discarding the goal.

### The camera was never stalling

A session was spent chasing a "camera stall under load" through memory,
nvblox and colour. The IR frames were 51 ms apart because the Orbbec driver
ships `enable_ir_long_exposure` true with no exposure cap, and a sensor
cannot expose for longer than its frame period. Capping exposure at 20 ms took
the left IR from 18.0 to 28.4 Hz. The launch file silently drops that
parameter, so `camera_tuner_node` sets it after start-up and reads it back.
MAXN_SUPER (`nvpmodel -m 2`) took cuVSLAM from 15.3 to 28.4 Hz and stopped its
input queue turning a 0.47 s lag into 4 s spikes.

### The colour stream and the stereo pair

Every colour frame is bandwidth taken from the IR pair cuVSLAM tracks on, and
the cost is linear. Same 3.5 m goal, same start, only `color_fps` changed:

| colour | traverse accuracy |
|---|---|
| off | **6.3 cm** |
| 5 fps | 10.5 cm |
| 15 fps | 14.7 cm |
| 30 fps | ~20 cm |

Navigation runs with colour **off**; the dashboard shows the IR stream, which
is already running for cuVSLAM and costs nothing. The ball demo runs colour at
15 fps because the detector needs it and that stack has no nvblox or Nav2.

---

## What is still open

### The relocalisation recovery radius is not fixed, and why is unresolved

Measured 2026-09-08, robot on its mark, which IS the saved map's origin:

| hint error | afternoon (map just built) | that night |
|---|---|---|
| 0 m | - | localises, 1-2 cm |
| 2.0 m | - | localises, 2.3 cm |
| 3.5 m | - | localises, 1.5 cm |
| 5.0 m | **5.9 cm** | **fails, reproducibly** |

Nothing in the stack changed between the runs. **Lighting is not the cause**:
indoor IR texture measured 4.39 by day and 4.4 at night, and both "VIO needs
daylight" and "the projector is needed at night" were tested to destruction
and found to be artefacts of the `time_domain` bug. What differed was the
scene's contents (a person at the desk; furniture moved later). By 2026-09-10
relocalisation failed three times from the mark itself, after the round tables
had moved, and the project switched to anchoring a fresh frame at the mark.
To resolve: re-run the 5 m hint on a fresh map, then vary one thing at a
time (room empty vs occupied, lamp on vs off). Do not write "needs daylight"
until that says so; this project wrote it once and retracted it.

### The IR pair arrives 0.47 s late whenever depth is on

| configuration | IR stamp lag |
|---|---|
| depth pipeline OFF | 0.019 s |
| depth pipeline ON | **0.459 s** |
| `time_domain` system | 0.420 s |
| `enable_sync_host_time` | 0.463 s |
| `depth_registration` false | 0.468 s |
| `enable_frame_sync` false | 0.470 s |
| `enable_point_cloud` false | 0.425 s |
| wheel TF, same board | 0.006 s |

The depth engine is on-device and the IR frames come out behind it. No
software knob moves it, so the stack is configured for the latency that
exists (`transform_tolerance` 1.5 s). The proper fix would be in the camera.

### Also open

- **The moved-furniture test has never been run** as a controlled test. It is
  the requirement that motivated the costmap rebuild.
- **`bt_navigator` times out acknowledging the first `follow_path` of each
  session**, and only the first: first-contact discovery, not load.
  `default_server_timeout` 2000 ms covers it; the cause is upstream.
- **`map_server` loses its lifecycle race** on this board often enough to
  matter; `map_server_guard` nudges it active. The race is unfixed.
- **Long traverses end less accurately than short hops** (14-20 cm against
  6 cm) and most of the loss is on the final rotate-to-heading, where the
  0.47 s pose lag lands on the last action.
- **Phase 11 following** was proven on the live stream (OWLv2 localises a
  bed and a chair in the map to ~1 cm across frames; the follower publishes
  goals 1 m short of the target) but was not made into a demo. Two lessons
  are recorded in `perception/`: prompts match appearance, not identity ("a
  blue board" found the book that "a book" did not), and a 0.12 confidence
  threshold accepts curtains (0.25 does not).
- **Phase 12** (events, "when did you last see...") is not started.
- **Autonomous exploration** and **mesh cleanup** remain optional.

## Practical notes

- **Charge fully.** Runtime tracks charge in: 2h30m, 1h50m, under an hour
  after a quick top-up. Budget under two hours with the full stack.
- **Commit as you verify.** A battery death has already stranded verified
  work once.
- **Keep people out of the way during a mapping run.** Mapping mode uses
  `static_tsdf` with decay off, so anything moving is fused in permanently.
- **Restart the stack after carrying the robot.** VIO does not survive being
  picked up: a spin test after a manual reposition started 49 m below the
  floor with 176 deg of roll.
- **After a hard power loss, check `dmesg` for ACR before suspecting
  software.** The GPU can fail its secure-firmware bootstrap and CUDA reports
  0 devices while the camera streams happily. A clean reboot fixes it.

---

## Hard-won settings that must not drift

Each of these cost real debugging time and is documented at the parameter.

| setting | value | why |
|---|---|---|
| `time_domain` | `global` | `system` corrupts IMU timestamps (11 ms deviation on a 4.9 ms sample): 0.63% -> 15-32% drift, invisible at rest |
| `enable_frame_sync` | `false` | makes the IR pair wait for depth: bursty frames (190 ms gaps) and 180 ms pose age. It was set twice in one dict and the later `true` won for 17 days |
| `enable_ir_long_exposure` / `ir_ae_max_exposure` | `false` / 20 ms | unbounded auto-exposure settled at 50 ms and capped the pair at 20 Hz; set by `camera_tuner` because launch drops the parameter silently |
| power mode | MAXN_SUPER (`nvpmodel -m 2`) | cuVSLAM 15.3 -> 28.4 Hz; needs the `-super` flash, and `nvpmodel -q` lies after the symlink workaround |
| `enable_laser` | `false` | the projector's dots move with the camera; it restores place recognition but not odometry |
| `mapping:=true` for map runs | `static_tsdf`, decay off | dynamic-mode decay deletes the map 27 s (~4 m) behind the robot |
| `map_clearing_radius_m` | 100 | default 5 m trims the map to a bubble |
| colour resolution | 848x480 | mismatch with depth costs ~15% of depth frames in the software aligner |
| `integrate_depth_rate_hz` | 10 | default 40 is faster than the camera produces frames |
| point cloud decimation | 4x | full res starved the whole camera pipeline (0.46 s -> 0.03 s lag) |
| colour while navigating | off (5 fps if needed) | every colour frame costs IR bandwidth: 6.3 cm off, ~20 cm at 30 fps |
| global costmap source | the saved 2D grid, in BOTH costmaps | an ESDF cannot say "unseen"; nvblox is not a costmap source |
| local obstacle layer | `VoxelLayer` on the decimated depth cloud | mark, clear, leave unknown alone |
| `dynamic_mapper` slice | 0.05-0.20 m, same as `static_mapper` | the default 0.0 fused tile reflections as walls |
| `transform_tolerance` (both costmaps) | 1.5 s | above the 0.47 s TF lag; below it Nav2 reports "Reached the goal!" without moving |
| `default_server_timeout` | 2000 ms | the first `follow_path` of a session is never acknowledged in 20 ms |
| Nav2 velocity smoother | `[0.15, 0.15, 0.5]` | stock `y = 0.0` silently forbids strafing |
| `wz_max` | 0.5 rad/s | at 1.0 the controller drove arcs and lost visual tracking |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.15 m / 0.25 rad, `stateful` true | from the measured 10.9 cm rotational excursion |
| `camera_link` | the LEFT IR lens, +0.94 deg roll | the body centre put every depth point 5 cm right of the truth |
| cuVSLAM anchor frame | `base_footprint` | a second parent on `base_link` split the TF tree in two |

**The method lesson behind most of these:** verify a change against what it
was supposed to improve, not against health counters. `vo_state` and frame
times report perfect health while an estimator drifts metres off course.
