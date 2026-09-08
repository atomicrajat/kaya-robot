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

Four pieces, only one of which is genuinely new work.

### 1. Semantic layer: what is where

The map today is pure geometry — it knows a wall is at (3.2, -1.4) but not that
it is a kitchen. The gap is filled by detecting objects and recording where
they are.

    colour frame -> YOLOv8 -> bounding boxes
                            -> depth at box centre -> 3D point (camera frame)
                            -> TF (map <- camera)  -> 3D point (MAP frame)
                            -> semantic database

`isaac_ros_yolov8` is **already installed**, along with `isaac_ros_tensor_rt`
and the DNN encoder. Nothing new to build; it needs wiring to depth and TF.

**Why COCO is enough.** YOLOv8's default 80 classes include `oven`, `sink`,
`refrigerator`, `microwave`, `toaster`, `dining table`, `toilet`, `bed`,
`couch`, `tv`, `chair`, `potted plant`. Those map onto rooms almost directly:

| observed objects | inferred place |
|---|---|
| oven, refrigerator, microwave, sink | kitchen |
| toilet, sink | bathroom |
| bed | bedroom |
| couch, tv | living room |
| dining table, chairs | dining area |

So "where can I cook" resolves without any custom training. A custom model is
only needed for objects COCO does not know, and that is a later refinement.

**Storage.** A small SQLite database, one row per detected object instance:

    class, x, y, z, confidence, n_observations, first_seen, last_seen

Repeated detections of the same physical object get clustered by proximity, so
the fridge seen forty times becomes one row with high confidence rather than
forty rows. Objects seen once and never again decay in confidence — which is
also how the robot notices something has moved.

### 2. Language: question -> target

"Where can I cook" -> the concept of cooking -> `oven`/`stove`/`sink` -> look
up those rows -> take the centroid -> send a Nav2 goal.

Two options, and the first is strongly preferred to start:

- **Cloud LLM (recommended).** The robot is on wifi. One API call turns the
  question into a set of target classes. Costs nothing in Jetson resources,
  handles phrasing the robot has never seen, and is trivial to iterate on.
- **Local small model.** A 1-3B quantised model runs on Orin Nano but slowly
  and in competition with the navigation stack for the 8 GB. Worth doing only
  if offline operation becomes a requirement.

A hardcoded keyword table would also work for a demo, but it breaks the moment
you phrase something differently, which is the whole point of asking in words.

### 3. Memory and events

The same database, with time. Every detection is already timestamped, so
"when did you last see my keys" and "has anyone been in the kitchen today" fall
out of the same table. Worth keeping a separate `events` table for things
worth remembering rather than every frame's detections.

### 4. Web dashboard

Two stages:

- **Now, nearly free:** `foxglove_bridge` is already installed. Foxglove Studio
  runs on the Mac, shows the live map and robot position, and its 3D panel can
  publish a goal where you click. Measured cost when idle: none. This delivers
  the "live map view and live bot location" half immediately.
- **Later:** a small custom web app for the chat interface — a text box that
  hits the language layer, plus the map view. Foxglove cannot host the
  question-asking part.

---

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

### Phase 9 — Object detection into the map (the core new work)
Wire `isaac_ros_yolov8` to the colour stream, project detections through depth
and TF into map coordinates, write to SQLite, cluster repeats. Publish
detections as markers so they are visible in Foxglove.

**Done when:** driving through the kitchen puts a `refrigerator` row in the
database at roughly the right coordinates, and it survives a restart.

**Watch for:** GPU contention. Verify raw VIO drift on a driven loop is still
under ~1%.

### Phase 10 — Places from objects
Cluster object positions into regions and label them from their contents
(oven + fridge + sink -> kitchen). Expose a lookup: name -> coordinates.

**Done when:** the robot can answer "where is the kitchen" with a coordinate,
and drive there on command.

### Phase 11 — Ask in words
Language layer: question -> target classes -> database lookup -> Nav2 goal.
Cloud LLM to start. Add the chat box to the dashboard.

**Done when:** "show me where I can cook" drives the robot to the kitchen, and
"take me to the washbasin" drives it to the sink.

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
