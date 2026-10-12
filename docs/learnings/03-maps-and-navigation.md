# Chapter 3: maps and navigation

*Part of the Kaya learning narrative. Index: [`../../LEARNINGS.md`](../../LEARNINGS.md).
Sources: commits `5ae406c`, `3bfcf97`, `02ed67f`, `1febc98`, `a165a76`, `524a007`, `8019c62`,
`9f1543c`, `95127be`, `f515514`, `5ccdabe`, `8ae8307`, `90d133e`, `5e6b850`, `f87b9ef`, `82ba82b`,
`f31b8a7`, `e9bc7c3`, `9953450`, `1c0fbf5`, `934f6a4`, `2917ec0`, `a820bbd`, `d3ed387`, `83c823d`,
`f223e5b`; `ROADMAP.md`.*

## 3.1 How nvblox turns depth into a map

**TSDF.** nvblox fuses each depth image into a grid of 5 cm voxels. Each
voxel stores a *truncated signed distance*: how far it is from the nearest
surface, positive outside, negative inside, clipped to a few voxels. Fusing
many views refines it; the zero-crossing is the surface, and that is the
mesh you can look at (468k vertices for this flat).

**ESDF.** From the TSDF nvblox computes a *Euclidean signed distance field*:
for every voxel, the distance to the nearest obstacle. A planner wants that
directly, because "how close is the nearest wall" is the cost of being here.

**The slice.** A ground robot plans in 2D, so nvblox takes a horizontal band
of the ESDF between two heights and collapses it into a 2D costmap. The band
is where the robot's body is. The stock configuration slices from 0 to 1 m at
a 1 m reference, which suits a tall robot. Kaya is 19 cm to the top of its
camera: the stock slice looks for obstacles a metre over its head, drives
into chair legs and flags tabletops it fits under. It was set to 5-20 cm.

Two early faults in the same file, both found by running rather than
reading: nvblox namespaces its inputs per camera (`camera_0/depth/image`),
so an un-prefixed remap connected to nothing and the node reported healthy
with an empty costmap forever; and the heights had been written as if the
map origin were the floor when `base_link` sits 37 mm above it (the
Chapter 1 TF fault, since reverted).

## 3.2 The map that deleted itself

Phase 7 switched nvblox to **dynamic mapping**, which separates geometry that
moves from geometry that does not, so a person walking past is seen while
present and leaves no wall behind. That worked, measured: static obstacle
count flat at 3363-3382 cells while a person walked in, stood, and left, and
253 near-obstacle cells visible through the combined slice while they were
there. The commit noted that Nav2's costmap must read the *combined* slice;
reading the static one would have made the robot blind to a person in front
of it, which is less safe than before with nothing to warn you.

What the commit did not check was what the mechanism costs. Dynamic mode
works by **decay**: voxel weights are multiplied by 0.95 five times a second
and blocks under 0.001 are freed. A voxel not re-observed is gone after
ln(0.001)/ln(0.95) = 134 passes, 27 seconds, which at this robot's speed is
about 4 m. Two long mapping runs with excellent odometry (59.55 m at 1.05%
drift; 77 m) returned meshes of 17 and 18 m² around wherever the robot
finished. The map had not been distorted; it had been deleted behind the
robot as it drove. Decay cannot tell a person who walked away from a wall
you drove past.

The two jobs want opposite things, so they are two configurations:
`mapping:=true` runs static TSDF with decay off (one rigid world, nothing
forgotten, keep people out of the way), and navigation runs dynamic mode.
Verified with a cheap test, driving 5.5 m away and checking the start was
still there: 18 m² became 132 m², and on the full pass 202 m². Two related
defaults also truncated maps independently of decay: a clearing radius of
5 m around the robot (raised to 100) and a visualisation exclusion radius.

A process note that saved a driving run: the new config file "had no effect"
at first because `setup.py` globs config files at build time, so a new file
needs a `colcon build` to exist in the install space. Reading the values the
node reported at start-up caught it.

## 3.3 Saving a map, and the three things it is

A saved map is three independent artefacts, and for a while only one was
reloaded:

| | what it holds | who loads it |
|---|---|---|
| `cuvslam/data.mdb` | where the robot **is**: cuVSLAM's landmark database for relocalisation | `map_path:=` |
| `<name>_2d.pgm/.yaml` | the 2D occupancy grid Nav2 plans on | `map_2d:=` |
| `<name>.nvblx` | what the **space** looks like: the nvblox volume | `nvblox_map:=` |

Launching with only `map_path` relocalised perfectly into the saved
coordinate system and then rebuilt the room from scratch off the live camera.
Stationary, the costmap was the cone the camera could see: 371 of 391 cells
in the corridor ahead, nothing behind. The robot knew exactly where it was on
a map that had forgotten the building, and would plan through walls it had
not seen yet. Reloading the volume restored all 391 cells. The reloaded
volume then turned out to cost 3.3 GB of RAM for 202 m²; the same area as a
2D grid is 115 KB, and navigation uses the grid. `kaya_save_map` now writes
all three.

## 3.4 Relocalisation: the measurement, and the open question

With a saved map, cuVSLAM is given a **pose hint** at launch and asked to
find itself. The wrong hint is the point of the test: recovering from a
deliberately wrong hint proves it recognises the place from landmarks rather
than trusting what it was told.

| hint error | result |
|---|---|
| 1.8 m, robot stationary between save and reload | 7 mm |
| 5.0 m, the afternoon the 202 m² map was built | **5.9 cm** |
| 0, 2.0, 3.5 m, that night, same spot, same map | 1-2 cm |
| 5.0 m, that night | fails, reproducibly |

Nothing in the stack changed between afternoon and night. The first
explanation written down was lighting; Chapter 2 explains why that cannot be
right (IR texture was identical by day and night), and it was retracted. What
did differ was the scene's contents (a person at the desk). Two days later,
after the round tables had moved, relocalisation failed three times from the
mark itself, and the project switched to anchoring a fresh frame at a marked
start. The recovery radius shrinking with scene change is reproducible; its
cause is not established, and `ROADMAP.md` gives the order in which to
resolve it rather than a guess.

**Believing the verdict.** cuVSLAM's localisation call returns only "request
accepted"; the outcome arrives seconds later and only in its log. The first
anchor node announced success on every run, including runs whose next log
line was "Failed to localize in map". It now reads the verdict from
`/rosout`, treats an absent or unrecognised one as unverified, and then
checks the claimed pose against what a flat floor allows, because on a
memory-exhausted board cuVSLAM twice said "Successfully localized" at a pose
2.4 m from truth and 69 cm below the floor. A first version of the check
rejected a good 2 cm localisation on a transform buffered from before it
happened: a checker that invents failures is no better than one that invents
successes, so it polls until consecutive fresh samples agree.

## 3.5 Five Nav2 defaults that stopped the robot from ever moving

Nav2 is the ROS 2 navigation stack: a behaviour tree supervising a global
planner and a local controller over two costmaps (a global one for planning
and a local one around the robot for steering). Its template configuration
assumes a differential-drive TurtleBot at 0.5 m/s. Kaya's first goal, a
0.5 m strafe, produced no motion at all while everything reported healthy.
Five separate defaults, every one silent:

1. **The global costmap had no extent.** With no map server it fell back to
   5x5 m with its corner at the origin, and cuVSLAM starts the robot at
   (0, 0); noise put it at y = -0.017, outside its own costmap. Every plan
   failed "out of bounds".
2. **The velocity smoother's `max_velocity` is `[x, y, θ]` and ships as
   `[0.5, 0.0, 2.0]`.** A differential base cannot strafe, so y is zero. That
   silently negated the `Omni` motion model set 300 lines above it: MPPI
   emitted a correct lateral command peaking at 0.133 m/s and all 1043
   smoothed samples downstream had y = 0.0 exactly.
3. **The collision monitor watched a LiDAR topic nothing published** and kept
   stopping the robot for a source that never arrived.
4. **Neither costmap declared `transform_tolerance`**, so both sat at 0.3 s
   while cuVSLAM's camera-stamped transforms tailed to 0.78 s. The failure
   mode was the dangerous part: "Transform data too old" followed by "Reached
   the goal!" and "Goal succeeded" from a robot that never moved.
5. **The progress checker demanded 0.5 m of motion in 10 s** to prove the
   robot was not stuck; a 0.5 m goal can never pass, so every goal was
   declared stuck and the recovery chain's spin is why a pure strafe ended
   130 degrees rotated.

The result, once fixed, was the goal that proves the most: a 0.5 m pure
strafe, reached to 6.1 cm with 0.1 degrees of heading change. Lateral motion
with x and yaw held at zero is something a differential base cannot do at
all.

## 3.6 Costmap layers: an ESDF cannot say "I have not looked there"

With a saved map loaded, a 3.5 m traverse stalled a metre out regardless of
which map was loaded; the 3.3 GB volume and the 115 KB grid failed
identically. Rendering the global costmap as text next to the saved map
showed why:

    saved map:       ##########++++...................++++####  <-- goal
    global costmap:  #############++++++++++::::++++++++++####  <-- goal

Free space only in a cone narrowing to nothing a metre ahead. nvblox's
costmap layer reads an ESDF, a distance-to-nearest-obstacle field, and a
distance field has no value for "unobserved". Unobserved space is
indistinguishable from occupied, so it blocks by default, and the global
planner could only ever plan inside the camera's field of view.

The fix is the ordinary Nav2 split, which this robot had skipped because it
has no LiDAR: the saved 2D grid is the global costmap and lives in **both**
costmaps (a first version put it only in the global one, and the controller,
steering on a local costmap that was nvblox-only and decayed behind the
robot, drove sideways into a 0.15 m gap narrower than its own radius). The
local obstacle source is Nav2's `VoxelLayer` on the decimated depth cloud,
which raycasts and so has the three states that actually exist: mark where a
ray ends, clear what it passed through, and leave unreached cells alone.
nvblox keeps doing what it is good at, the 3D reconstruction and the saved
map; it is not an observation source for a costmap.

Two smaller findings on the way: nvblox's layer marks everything within its
inflation distance as **lethal**, not graded, and 0.35 m was more than twice
the robot's radius (cut to 0.22 m); and all the slice settings had been under
`static_mapper` while navigation uses the `dynamic_mapper`, whose slice
started at the floor plane and fused polished-tile reflections as walls. The
tuning was real and correct, and navigation had never used it.

## 3.7 The controller

MPPI (model predictive path integral) samples thousands of candidate
velocity sequences over a short horizon, scores each with a set of critics,
and commands the weighted best. Three things learned about it on this robot,
each on the same 3.5 m goal from the same start:

- **A longer horizon made it worse.** 56 steps of 0.05 s is 2.8 s and about
  0.42 m of lookahead at 0.15 m/s, which looked like a plausible reason for
  failing to thread clutter. Raised to 90 steps (batch cut to keep compute
  flat), the robot stalled 2.71 m short instead of 0.62: it dithered until
  the progress checker gave up. Lookahead was not the problem and batch size
  is not free to trade. Reverted, numbers at the parameter.
- **`PreferForwardCritic` is for bases that cannot strafe.** Disabled; the
  omni base's measured 0.5 m strafe with -0.1 degrees of heading is the
  reason.
- **MPPI does not rotate in place**; nothing in its critics rewards spinning
  on the spot, so it drives arcs while turning. At `wz_max` 1.0 rad/s it
  satisfied heading by driving an arc, finished 0.60 m past the goal while
  reporting success, and once rotated fast enough to lose visual tracking
  (frames were 66 ms apart under load, so 4 degrees between frames) and
  dead-reckoned the pose out past 13 km. Halved to 0.5.

## 3.8 Goal tolerances come from a measurement

The goal checker had two faults. `stateful: true` stops re-checking position
once it has been met and then only watches heading, so the robot latched
inside 10 cm, kept manoeuvring for the heading, and settled 0.60 m past the
target while logging "Reached the goal!" from there. Set to false, the robot
hunted instead: it passed through the goal at 8.0 and 2.4 cm without being
allowed to stop, because the heading gate (0.15 rad, set when every goal was
a short hop) was tighter than Nav2's own default.

The number that settled it: commanded a pure in-place turn, the base closes
360 degrees with 1.7 cm of net translation but excursions up to 10.9 cm part
way through. `xy_goal_tolerance` was 0.10 m, so the checker was asking the
robot to stay inside a circle smaller than its own rotational wobble. 0.15 m
clears the wobble and is well inside the robot's 0.32 m width; heading 0.25
rad; and with `wz_max` at 0.5 the stateful latch became the right behaviour
(reach the position, latch, then turn). Three consecutive 3.5 m goals, zero
tracking losses, heading within one degree.

## 3.9 The false "Reached the goal!"

The robot stopped 23 cm into a 5 m drive and reported success, reproducibly,
to the centimetre. That precision was the clue: not a controller wandering,
but a check returning the same wrong answer every time.

Nav2's `ControllerServer::isGoalReached()` transforms the goal into the
global frame and ignores the return value of the transform. When the
transform fails because the data is too old, the output pose is never
written and stays default-constructed at (0, 0, 0). The goal checker then
compares the robot against the **origin**. This robot anchors its origin at
the mark it starts from, so a transform failure just after setting off asks
"am I near (0, 0)?" while the robot is 10 cm from it. Yes. Success. It only
fires at the start; a run that got 4.25 m out was far enough from the origin
for the comparison to fail.

Other robots do not hit it because the failing branch is only reached when
the transform is older than `transform_tolerance`, and a robot whose TF is
fresh to tens of milliseconds never enters it. This one enters it constantly:
the camera emits its IR pair about 0.47 s late whenever depth is on (Chapter
2), cuVSLAM inherits that into every transform, and at 0.8 s the spikes under
motion crossed the line. 1.5 s is not a workaround; it is the correct setting
for the latency that exists, and Nav2 then uses the latest transform instead
of discarding the goal. The same run that found this also found that the
board had been at 25 W: MAXN_SUPER took cuVSLAM from 15.3 to 28.4 Hz and
stopped its input queue turning the 0.47 s lag into 4 s spikes. Result, the
marked start to the kitchen, 5.1 m: 14 cm, heading exact, zero transform
errors.

## 3.10 Two timing faults in the behaviour tree

`bt_navigator` gives the controller 20 ms to acknowledge a goal, and on this
board the controller is mid-solve when the request arrives. Every long goal
died about a second in: the tree aborted, the controller kept executing the
path it already held, and nothing supervised it. No replanning, no recovery;
the behaviour server never ran a single spin, backup or wait all session.
"It stops in between even though it can reach the goal" was this. Raised to
2000 ms; it still times out on the first `follow_path` of each session and
never on the second, which is first-contact discovery between two nodes
rather than load. A 10 Hz controller was tried for it and made things worse
(0.31 m short became 1.21 m); reverted.

`map_server` is a lifecycle node that intermittently loses its activation
race against nvblox's one-to-two-minute start-up. Inactive, it does not
publish, the static layer has nothing, the planner times out, and the tree
loops its clear-costmap recovery: a robot that will not move, several layers
from the cause. A guard node polls the state and nudges it active; it is
idempotent, and it watches a condition rather than waiting a fixed time,
because fixed delays had already failed once for the origin reset.

## 3.11 Stopping when the pose stops being real

The anchor validates the pose once, at localisation. When cuVSLAM lost
tracking mid-traverse, dead reckoning ran the pose out past 13 km while Nav2
kept planning and issuing velocities. The failure worth guarding is not a
failed run; it is a robot moving in the real world on coordinates that are
fiction.

`pose_guard` applies the flat-floor limits continuously (|z| and tilt) plus
an implied-speed check: 0.15 m/s on each axis puts a hard ceiling on how far
two consecutive transforms can be apart, and the diverging run showed 1.26
m/s. Three consecutive bad samples trip it, so jitter does not. Two things it
got wrong first: it tripped before the robot had moved, because
`map -> base_footprint` legitimately reads z = -0.49 m with 10 degrees of
pitch while cuVSLAM is initialising, so it now arms only after the anchor's
verdict; and it borrowed the anchor's limits (0.10 m, 8 degrees), which are
right for a stationary robot and wrong for a moving one. Measured on the same
360-degree turn with Nav2 loaded: 8.7 degrees of roll, 7.7 of pitch, 0.199 m
of z, all recoverable transients. Limits are 0.30 m and 15 degrees, far below
what is worth catching. On tripping it cancels through the navigation
action's own cancel, not by publishing zero velocity, because fighting the
controller for `/cmd_vel` is a race the controller wins half of. It does not
try to recover: a diverged tracker needs relocalisation, and guessing where
the robot is in order to keep going is the mistake it exists to prevent.

## 3.12 Process hygiene that masqueraded as algorithm bugs

`ros2 launch` starts the composable-node containers as separate processes.
Killing the launch process orphans them. Over one evening of relaunches that
accumulated ten orphaned nvblox processes at about 800 MB each; available
memory went to 54 MB and load average to 49. The two failures that followed
both looked like cuVSLAM bugs: it stopped reporting localisation outcomes at
all, and relocalisation returned a confident pose 2.4 m from truth. Neither
was. `kaya_run.sh` launches into its own session and signals the process
group, with a sweep for anything an earlier session left behind.

A separate afternoon had 65 orphaned Nav2 servers and 14 robot state
publishers, load average 94 on six cores, and the symptoms pointed at USB:
the camera took 27 s to set a log path, cuVSLAM took 67 s to initialise and
its track time tripled. Hours went into a USB problem that was a scheduling
problem. Enumerating node names to kill them failed twice (missed the Nav2
servers, then the state publishers), which is why the process group is the
primary mechanism and names are only a fallback.

And `pkill -f "<pattern>"` matches the shell that is running it, because
the pattern appears in that shell's own command line. It killed three ssh
sessions before it was understood, once from `kaya_run.sh start nvblox_map:=...`
matching its own "nvblox". Patterns now match executable paths and exclude
the calling shell and its parent.
