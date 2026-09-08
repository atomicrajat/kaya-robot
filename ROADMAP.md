# Kaya: semantic navigation roadmap

**Goal.** Ask the robot, in words, to go somewhere described by what you *do*
there — "show me where I can cook", "take me to the washbasin" — and have it
work out where that is, drive there autonomously, and avoid obstacles on the
way. Plus a web dashboard with a live map, live robot position, and a place to
type the question.

This document is the plan for getting there, the current milestone, and what to
test next session.

---

## Where we are now

Phases 1-7 are complete and verified on hardware.

| capability | measured |
|---|---|
| holonomic drive + wheel odometry | working |
| visual-inertial odometry (cuVSLAM) | **0.63% drift** over 16.9 m |
| autonomous navigation (Nav2 + nvblox) | goals reached to 2-12 cm |
| omni strafing | 0.5 m lateral, -0.1 deg heading change |
| map of the real space | **202 m^2**, 468k vertices |
| relocalisation after restart | **5.9 cm** from a 5 m wrong hint |
| navigate on a reloaded map | 11.6 cm, zero recoveries |
| dynamic obstacles | people are not fused into the map |

The hard part — reliable localisation and navigation — is done. Everything
below builds on top of it and does not require touching it.

---

## What "show me where I can cook" actually requires

### The approach: remember scenes, let a reasoning model choose

The map today is pure geometry — it knows a wall is at (3.2, -1.4), not that it
is a kitchen. The gap is filled by remembering *what places looked like*, and
asking a reasoning model which one matches the request.

    while mapping:   keyframe image + its map pose   ->  a few hundred JPEGs
    on a question:   "where can I cook?" + candidates ->  model picks one
                     -> navigate to that keyframe's pose

This is deliberately chosen over the obvious alternative of running object
detection and rebuilding meaning from class labels. Both were considered:

| | object detection | keyframes + reasoning model |
|---|---|---|
| Jetson cost | YOLO at 1-2 Hz, competes for GPU | saving JPEGs, nearly free |
| vocabulary | COCO's 80 fixed classes | anything describable |
| "somewhere to read" | impossible | works |
| "where can I cook" | infer from oven + fridge | sees a kitchen |
| new code on robot | detect -> depth -> TF -> cluster | an image writer |

COCO cannot represent "a cosy corner to read in" or "somewhere to put this
down". A model that looks at the scene can. The heavy lifting also moves off
the robot, which is what makes it affordable at all.

### Where the reasoning model runs

**Not on the Jetson.** NVIDIA Cosmos 3 — the natural fit conceptually, being a
reasoning VLM built for physical AI — ships as Nano at 16B parameters
(positioned for workstation inference) and Super at 64B (datacenter). At 16B
even INT4 quantisation is roughly 8 GB, which is the entire board, and the
navigation stack already holds 3.7 GB. This is not a close call.

Options, in order of practicality:

1. **Cloud vision model (recommended to start).** Send the question and a
   shortlist of keyframes, get back a choice. No Jetson cost, no deployment,
   easy to iterate. Needs wifi at query time, which the robot has.
2. **Cosmos 3 Nano on a workstation.** If a machine with enough VRAM is
   available, this is the closest thing to what was envisioned, and keeps
   everything on the local network.
3. **A small VLM on the Jetson (2-3B, quantised).** Only worth it if the robot
   must answer with no network at all. Expect it to be slow and to compete with
   navigation for memory.

The robot side is identical in all three — it stores keyframes and executes a
goal. Swapping the model is a configuration change, not a rebuild.

### Keeping the candidate set small

A few hundred keyframes is too many to send to any model per question. Narrow
first, cheaply:

- **Spatial thinning.** Keep one keyframe per ~1 m of travel and per ~45 deg of
  heading change. A 200 m^2 space needs a few hundred, not thousands.
- **Embedding pre-filter.** A small local image-text model (CLIP-class, well
  within the Jetson's budget) scores every keyframe against the question and
  passes only the top handful to the reasoning model. This is the piece that
  makes the whole thing scale, and it can run in the background as the
  user suggested — it does not need to be fast.

### Memory and events

Keyframes are timestamped and pose-tagged, so the same store answers "when did
you last see..." and "has anything changed in the kitchen". Worth a separate
events table for things worth remembering rather than every frame.

### Object detection: still useful later

Not abandoned, just not first. `isaac_ros_yolov8` is already installed, and
object positions in map coordinates are the right tool for precise targets —
"go to the fridge" rather than "go to the kitchen". It layers on top of the
keyframe store rather than replacing it.

### Web dashboard

Two stages:

- **Now, nearly free:** `foxglove_bridge` is already installed. Foxglove Studio
  runs on the Mac, shows the live map and robot position, and its 3D panel can
  publish a goal where you click. Measured cost when idle: none.
- **Later:** a small custom web app for the chat interface — a text box that
  hits the reasoning layer, plus the map view. Foxglove cannot host the
  question-asking part.

## Running this on an Orin Nano 8 GB

The user's instinct is right and worth stating as a design principle: **the
semantic layer does not need to be fast.** Navigation is real-time and must
stay so; understanding the room can lag by seconds or run in the background.

Measured headroom with the full nav stack running:

    memory used     3.7 GB of 7.5 GB      (~3.8 GB free)
    nvblox RSS      1.2 GB
    cuVSLAM track   7-9 ms per frame

Design rules that follow:

1. **Run detection at 1-2 Hz, not 30.** A room does not change fast. This is
   the single biggest saving and costs nothing in usefulness.
2. **Detect only while moving slowly, or when the view has changed.** No point
   re-detecting the same wall a hundred times.
3. **YOLOv8n (nano), TensorRT, INT8 or FP16.** Roughly 0.5-1 GB. Fits.
4. **Never let it starve cuVSLAM.** This project has already been bitten twice
   by exactly that class of problem — the point cloud starving the pipeline,
   and dynamic-mode decay deleting the map. Measure `track_execution_time_mean`
   and raw VIO drift on a driven loop before and after, not just `vo_state`.
5. **Language and reasoning happen off-board.** No LLM on the Jetson to start.

---

## Phased plan

Each phase is independently useful and independently testable.

### Phase 8 — Dashboard and click-to-navigate (small)
Bring up `foxglove_bridge`, connect Foxglove on the Mac, confirm live map and
robot position, publish a goal by clicking. Measure the cost to VIO of a
connected client streaming mesh and pointcloud topics — untested so far.

**Done when:** you can click a point on the map and the robot drives there.

### Phase 9 — Keyframe memory (the core new work, and it is small)
While driving, save a colour keyframe plus its map pose whenever the robot has
moved ~1 m or turned ~45 deg. Store as JPEG plus a JSON index. Nothing else on
the robot changes.

**Done when:** a mapping run leaves a few hundred pose-tagged images on disk,
and they survive a restart.

**Watch for:** almost nothing. Writing JPEGs is cheap. Still re-check raw VIO
drift on a driven loop, because this project has twice been surprised by a
change that looked free.

### Phase 10 — Ask in words
Query path: question -> shortlist keyframes -> reasoning model picks one ->
send its pose as a Nav2 goal. Cloud vision model to start; a CLIP-class local
pre-filter to keep the shortlist small once there are many keyframes.

**Done when:** "show me where I can cook" drives the robot to the kitchen, and
"take me to the washbasin" drives it to the sink.

**Note:** this is the phase that proves the whole idea, and it needs no new
robot capability — only the keyframes from Phase 9 and the navigation that
already works.

### Phase 11 — Objects, for precise targets
Now add `isaac_ros_yolov8`: detections projected through depth and TF into map
coordinates, stored alongside the keyframes. This is what turns "go to the
kitchen" into "go to the fridge".

**Watch for:** GPU contention — this is the first phase that genuinely competes
with cuVSLAM. Verify raw VIO drift on a driven loop stays under ~1%.

### Phase 12 — Memory and events
Event table, decay of stale objects, time-aware queries. "When did you last
see..." and noticing that something has moved.

### Later / optional
- Human detection and following (COCO already has `person`)
- Semantic segmentation via `isaac_ros_unet` (not installed, more GPU) if
  object boxes prove too coarse
- Autonomous exploration so it can map a new space unattended (nothing
  installed; needs a frontier explorer on the ESDF)
- Mesh cleanup for a presentable 3D model — cosmetic, not needed for navigation

---

## Next session: test after charging

In order. Stop at the first failure rather than pressing on.

1. **Health check after the unclean shutdown**
   - `sudo dmesg | grep -i nvgpu` — expect 0 ACR errors. If the GPU failed to
     initialise, reboot; CUDA will report 0 devices while the camera looks fine.
   - Confirm both repos still at the same commit.

2. **Re-verify the map still loads** (it was saved and tested once)

       ros2 launch kaya_bringup nav.launch.py \
         map_path:=/home/orin-nano/kaya_maps/home_final/cuvslam \
         hint_x:=3.0 hint_y:=-4.0

   Expect relocalisation to within ~10 cm despite the deliberately wrong hint.

3. **Navigate to two or three different points** on the saved map, including
   one in another room. Everything so far has been short hops in open space;
   a goal that requires actually crossing the map is untested.

4. **Drive a longer loop and re-check drift** — confirm 0.63% was not a lucky
   run. Read `odom -> base_footprint`, not the corrected pose.

5. **Then start Phase 8** (Foxglove), which needs no driving and little battery.

### Practical notes

- **Charge fully.** Runtime tracks charge in: 2h30m, 1h50m, and under an hour
  after a quick top-up. Budget under two hours with the full stack.
- **Commit as you verify.** A battery death has already stranded verified work
  once.
- **Keep people out of the way during a mapping run** — mapping mode uses
  `static_tsdf` with decay off, so anything moving is fused in permanently.
  That is the deliberate trade; navigation mode still handles people correctly.

---

## Hard-won settings that must not drift

Each of these cost real debugging time and is documented at the parameter.

| setting | value | why |
|---|---|---|
| `time_domain` | `global` | `system` corrupts IMU timestamps: 0.63% -> 15-32% drift |
| `mapping:=true` for map runs | `static_tsdf`, decay off | dynamic-mode decay deletes the map 27 s behind the robot |
| `map_clearing_radius_m` | 100 | default 5 m trims the map to a bubble |
| colour resolution | 848x480 | mismatch with depth costs ~15% of depth frames |
| `integrate_depth_rate_hz` | 10 | default 40 is faster than the camera produces frames |
| point cloud decimation | 4x | full res starved the whole camera pipeline |
| Nav2 velocity smoother | `[0.15, 0.15, 1.0]` | stock `y = 0.0` silently forbids strafing |

**The method lesson behind most of these:** verify a change against what it was
supposed to improve, not against health counters. `vo_state` and frame times
report perfect health while an estimator drifts metres off course.
