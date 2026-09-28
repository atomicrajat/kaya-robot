# Kaya: training it with RL in Isaac Lab (plan)

**Goal:** train Kaya with reinforcement learning in simulation to do tasks
where its **holonomic (omnidirectional) drive** is the point, and show them
working. Then publish the whole path, from building the robot to a trained
policy, as an end-to-end tutorial in this repo.

**Two demos, in this order:**

1. **Ball to goal** (first). Kaya finds a yellow ball, collects it and pushes
   it into a goal zone on the floor. It is fun to watch, needs only an open
   floor, a ball and a tape square to show on the real robot, and Kaya's slow
   speed suits careful ball control.
2. **Warehouse inspection** (second, sim only). Kaya drives through a
   warehouse and points its camera at a list of shelf locations without
   stopping - the industrial use case.

Written 2026-09-27; decisions last updated the same day. **A new session
should read this file first**, then `kaya_description/usd/README.md`.

---

## 0. Where things stand (read before touching anything)

- **Branch:** `isaac-sim-usd` (8 commits on `master`; not pushed, not merged).
  This file is not committed yet. Continue on this branch or branch from it.
- **The sim robot is done and verified.** `kaya_description/usd/kaya.usda` is
  an articulated Kaya: chassis, 3 wheels, 36 free rollers, the Gemini 335L
  with the real robot's calibration, and servo drives. Measured against the
  real robot: mass, table friction (15 deg tilt), camera pitch and roll (the
  real IMU), stereo calibration, and the servo step response. There are 13
  checks, and they pass on **CPU and GPU**:
  `~/isaac-sim/python.sh kaya_description/usd/kaya_sim.py --test [--device cuda]`
- **Software:**
  - Isaac Sim **6.0.1** at `~/isaac-sim`.
  - Isaac Lab **3.0.0** (`release/3.0.0-beta2`) at `~/IsaacLab`, linked to
    that Isaac Sim. RSL-RL, skrl, rl_games and SB3 are installed. Read
    Isaac Lab's own `CLAUDE.md` / `AGENTS.md`.
- **Machine:** RTX 5090 Laptop, 24 GB.
- **The real robot is not needed until the real-robot steps.** Pending
  hardware items are in the project memory (`kaya-next-robot-session`).

### Lessons from building the sim; do not relearn them

1. **240 Hz physics.** At 60 Hz the ~3 g rollers chatter.
2. **Rollers use SPHERE colliders, with armature and bearing damping.**
   Convex hulls chattered: 13% speed loss, and rollers spinning at 110 rad/s.
3. **Wheels are POSITION servos fed a moving target.**
   - Velocity-only drives creep on slopes, whatever the damping (solver
     residual).
   - PhysX reports continuous joints wrapped into **(-2pi, 2pi]**, so unwrap
     them yourself.
   - Integrate targets on the **physics step**, not the render clock.
   - `kaya_servos.py` does all of this; it needs a vectorised torch version
     for Isaac Lab.
4. **The real servo is not instant:** a 30 Hz drive loop, then 27.5 ms of
   delay, then a 2nd-order response (wn 31 rad/s, zeta 0.60). Keep this lag
   in training.
5. **On GPU, read poses from the physics tensors, not USD.** USD is not
   written back.
6. **Roller friction mu 0.40**, combine mode `min`. GPU physics holds static
   friction ~0.9 deg longer than CPU. **Re-run `tilt_test.py` after any mass
   or collider change**; the calibration drifted once.
7. **The camera:**
   - `camera_link` is the LEFT IR lens, 17.5 cm above the floor, pitched
     4.89 deg up.
   - Depth and colour are both 848x480, fx ~408 / 405 px.
   - FOV is about 92 x 61 deg, **forward only**.
   - The bottom of the image looks 25.6 deg below horizontal, so the floor is
     visible only from ~0.39 m ahead of the robot centre.
   - Depth works from 0.17 m; colour has no minimum range.

---

## 1. Shared foundation (both demos)

**D1. Action = body twist (vx, vy, w) at 10 Hz**, exactly what `/cmd_vel`
means on the real robot.
- A vectorised port of `kaya_servos.py` turns it into wheel targets: the
  kinematics `v_i = -sin(th) vx + cos(th) vy + L w`, the servo chain, and
  position-servo targets on the physics step.
- This is the Isaac Lab navigation pattern: a high-level policy commanding a
  low-level controller (cf. `isaaclab_tasks/manager_based/navigation/config/anymal_c`).
  Our low level is analytic and already calibrated.
- A trained policy therefore deploys to the robot as a `/cmd_vel` publisher.
- Speed limits match the drive node: 0.15 m/s and 1.0 rad/s.

**D2. Train, then prove the benefit of holonomy.** Every demo is also trained
with **vy = 0** (differential drive), on the same tasks. The side-by-side
numbers and videos are the result: what holonomy buys, measured rather than
asserted. Kaya's footprint is nearly round, so the benefit has to show up in
behaviour (sliding sideways, circling a point while facing it), not in
passing through gaps.

**D3. Algorithm.** RSL-RL PPO, Isaac Lab's default.
- The critic is **asymmetric**: it gets privileged state (true ball and goal
  poses, contacts).
- The policy gets only what the real robot can sense, plus a small memory
  (GRU or frame stack), because the camera faces forward only.

**D4. Domain randomisation, from measured values only:**
- floor mu in [0.25, 0.40] (the tile is <= 0.40);
- servo delay +-30%, wn / zeta +-20%;
- robot mass +-5%;
- camera pitch and roll +-0.5 deg;
- IMU noise (`kaya_imu_noise.py`);
- detection noise and dropouts (section 2).

### Milestones

**M0: Many Kayas at once, and the performance decision.**
- Load `kaya.usda` in an `InteractiveScene` cloned to N environments on GPU
  physics.
- Port the key `--test` checks to batched form.
- Measure physics steps/s at N = 64, 256, 1024 and 4096.
- *Decision:* keep the full 36-roller asset, or build a lighter training
  variant validated against it. The full asset stays the reference for
  validation.
- *Done when:* the checks pass batched, and the throughput and the decision
  are recorded here.

**M0 result (2026-09-27, RTX 5090 Laptop, GPU otherwise idle):**
`kaya_lab/scripts/m0_batched.py`. The batched checks pass at N = 64 and 1024:
mass 2.088 kg and rest height 37.0 mm in every env, and forward / strafe /
rotate at 98-100% of command through the vectorised servo chain
(`kaya_lab/kaya_lab/servos.py`, matched to the numpy original to 2e-6 rad/s by
`scripts/test_servos_match.py`). Nothing went unstable in any run.

    solver pos iters   N=64      N=1024     N=4096    (policy decisions/s at 10 Hz)
    32 (asset)         270       4.1k       7.4k
    16                 407       5.9k       11.5k
     8                 535       7.9k       16.2k     drive 98.2-100.3%
     4                 635       8.9k       20.8k     drive 97.8-100.9%

- Up to ~1024 envs the cost per physics step is roughly FIXED (kernel-launch
  latency x solver iterations), so more envs are nearly free until ~4096.
- **A second Isaac Sim instance on the GPU halved throughput** (a leftover
  `kaya_sim.py` GUI). Close everything else before measuring or training.
- **Decision:** keep the full 36-roller asset (no light variant needed yet);
  **train with 8 solver position iterations** on the flat floor. The 32 in the
  USD exists only to hold on a 10 deg slope; keep 32 for validation and B4.
  About 16k samples/s means ~100M samples in ~1.7 h. Revisit a lighter asset
  only if B1 needs far more samples.

**M1: Twist action plus drive-to-a-point on an empty floor.**
- A vectorised `KayaTwistAction`, an environment skeleton (DirectRLEnv is the
  easiest to read for a tutorial), and a random 2D goal.
- *Done when:* over 95% of random goals within 3 m are reached.

**M1 result (2026-09-27): DONE.** `Kaya-PointGoal-v0`
(`kaya_lab/kaya_lab/tasks/point_goal/`), 4096 envs, RSL-RL PPO, 8 solver
iterations, servo-chain randomisation on. Success **99.8% at iteration 25**
(~3 min) and 100% from iteration 50. By iteration 200: final distance
1.5 cm, mean episode 8.9 s, action std 0.50 -> 0.09, no falls.
Checkpoint: `~/IsaacLab/logs/rsl_rl/kaya_point_goal/2026-09-27_22-47-16/model_200.pt`
(ONNX + TorchScript exports in `exported/`).
- **First holonomy finding:** the policy drives DIAGONALLY. The drive node
  clips vx and vy separately (`drive_node.py:169-171`), so full vx plus
  full vy is 0.21 m/s, 41% faster than straight. The sim is faithful to the
  robot, and a diff-drive robot cannot do this. Mean goal 1.75 m in 8.9 s is
  above the 0.15 m/s straight-line limit. Seen in the GUI replay by the user.
- **vy = 0 comparison** (`Kaya-PointGoal-DiffDrive-v0`, same settings, 200
  iterations, `~/IsaacLab/logs/rsl_rl/kaya_point_goal_diffdrive/2026-09-27_23-21-10`):

      at iteration          omni (vx, vy, w)   diff drive (vy = 0)
      25: success           99.8%              62%
      200: success          100%               100%
      200: time per goal    8.9 s              12.6 s   (omni ~30% faster)
      200: final distance   1.5 cm             1.5 cm

  Both reach every goal. Omni learns faster and arrives sooner: it does not
  turn first, and it gets the diagonal speed. A modest gap on this easy
  task; the ball task is where the gap is expected.

New code goes in a package in this repo, e.g. `kaya_lab/` (Isaac Lab
extension layout, `kaya_lab/tasks/...`), so it ships with the tutorial.

---

## 2. Demo 1: ball to goal (FIRST)

**Task:** a yellow ball and a goal zone start at random positions on an open
floor. Kaya must find the ball, get behind it, push it, and get it into the
goal zone. When Kaya stops, the ball keeps rolling; the policy must learn to
ease off early.

**The ball (decided):** **15.3 cm diameter**, matte yellow, light (target
< 250 g), low bounce. Why this size, worked out from the camera geometry for
a ball held against the front edge:
- About **29% of the ball stays visible while it is pushed.** That is about a
  450 x 130 px yellow blob in the 848x480 colour image, easy for a colour
  detector.
- The ball's top stays **2.2 cm below the camera**, so the view straight
  ahead stays clear. It blocks only about the bottom 27% of the image.
- It clears the 19.5 cm front edge by about 4 cm, and is far too big
  (> ~10 cm) to roll under the deck.
- An 18 cm ball was considered: more visible (43%), but it blocks 45% of the
  view and sits level with the camera. Rejected, because the extra
  visibility buys nothing at these blob sizes.
- A tennis ball (6.7 cm) is invisible once it is close. Rejected.

**No cradle (decided).** Kaya pushes the ball with its flat front, like
RoboCup "push-dribbling". The ball is unstable on a flat face, so the policy
must keep correcting under it. That is the point: two behaviours only an omni
robot can do well should appear:
1. **sliding sideways** to keep the ball centred, without turning;
2. **turning by circling around the ball** while facing it, instead of
   spinning in place and flinging it away.

This makes the holonomic vs differential-drive comparison more dramatic.
- *Fallback, decided by data:* if success stalls, add a cradle **in sim
  first** and compare. Build one for real only if the numbers say so.

**Possible front bumper (check against the real robot).**
- In the CAD, the lowest front surface between the front wheels is the
  upper-deck edge, ~10 cm off the floor, with open space beneath.
- That would contact a 15.3 cm ball **above** its centre (7.65 cm). It would
  press the ball into the floor so that it scrubs, and could wedge it under
  the edge.
- If so, add a flat vertical strip across the front (roughly 2-10 cm high).
  Confirm from a photo of the real front before deciding. Whatever is
  decided, model the same shape in `kaya.usda`.

**Observations:**
- **Ball:** bearing and distance from a yellow-colour detector.
  - In sim, simulate the *detector's output*: visible only inside the
    camera's FOV, with the near-range floor cut-off, noise and random
    dropouts. Do not render images; it is fast and it matches how the real
    detector behaves.
  - On the robot: HSV threshold on the colour image. The blob size gives
    distance up close; depth confirms it beyond 0.17 m.
- **Goal:** its position relative to the robot, from odometry. It is not
  seen: near the goal, the ball hides the tape on the floor anyway.
- **Also:** base velocity, the previous action, and memory (D3).

**Rewards:**
- approach the ball;
- ball in contact with the front;
- ball progress towards the goal;
- a large success bonus when the ball rests in the zone;
- penalties: ball lost out of range or out of bounds, time, action jerk.

**Sim ball:** a rigid sphere with the real diameter, mass, friction and
rolling resistance, set from the measurements below.

**Milestones:**
- **B1:** the ball task with privileged ball pose (the teacher).
  *Done when:* over 90% success from random starts.
  **DONE 2026-09-28** (`Kaya-BallGoal-v0`, `kaya_lab/kaya_lab/tasks/ball_goal/`,
  box goal 46 x 30 x 30 cm, width randomised 40-70 cm, 4096 envs, 300
  iterations, ~75 min). Curriculum reached level 1.0 at iteration 100.
  At full difficulty: 93.5% (it. 125), 99.5% (it. 200), 99.9% (it. 250-300),
  ~0.1% ball/robot out of bounds, ~14 s per goal, ball in the front pocket
  ~60% of the time. Checkpoint:
  `~/IsaacLab/logs/rsl_rl/kaya_ball_goal/2026-09-28_00-28-45/model_300.pt`
  (ONNX in `exported/`). Replayed at level 1 with the real 46 cm box: the user
  confirmed the holonomic behaviours (sideways correction while pushing) and
  reliable scoring. Replay:
  `rl.sh play --task Kaya-BallGoal-v0 --viz kit --checkpoint <model_300.pt> env.initial_level=1.0 "env.goal_width_range=[0.46,0.46]"`
- **B2:** the detector-based observation (the student: trained directly or
  distilled from B1).
  *Done when:* within 10 percentage points of the teacher.
  **DONE 2026-09-28** (`Kaya-BallGoal-Det-v0`: `kaya_lab/kaya_lab/detector.py`
  and `tasks/ball_goal/ball_goal_det_env.py`). Trained directly with PPO
  (no distillation), with an asymmetric critic (actor 15 obs, critic +true
  ball = 19). Memory is explicit: the last detection is kept in the odom
  frame and re-expressed each step, plus the time since it was seen (no
  RNN). Detector: pinhole projection with the real colour intrinsics and
  lens pose, >= 12 px on the image, range noise 1 cm + 3% (+2 cm when cut
  off), bearing 0.7 deg, 5% dropout, 100 ms latency. It matches the real
  touching frames' top-edge rows exactly (383 / 387). Lens distortion is NOT
  modelled (~2 deg at the image edge): undistort on the robot.
  Results at full difficulty: 85% (it. 100), 96% (it. 125), 99.7% (it. 150),
  99.9-100% (it. 200-300). The ball is in view ~83% of the time, and 14.3 s
  per goal (teacher: 99.9%, 13.6 s). Curriculum level 1.0 at iteration 75.
  Checkpoint: `~/IsaacLab/logs/rsl_rl/kaya_ball_goal_det/2026-09-28_01-58-14/model_300.pt`
  (ONNX in `exported/`). Replay shows a belief marker (green = seen, red =
  memory).
- **B3:** (DEFERRED 2026-09-28 by the user: not on the path to the real
  robot; run unattended later with `rl.sh train --task Kaya-BallGoal-DiffDrive-v0
  --num_envs 4096 --max_iterations 300`) the vy = 0 comparison: success, time and ball-loss rate, plus
  side-by-side videos.
- **Real-frame check of the image detector** (`kaya_lab/kaya_lab/ball_detector.py`,
  2026-09-28; `scripts/test_ball_detector_real_frames.py`): it detects the
  ball in all 5 real frames (17 ms each), but with the DOCUMENTED colour
  camera pose and no distortion, the cut-off (touching) frames fit badly (4-6
  deg rms) and land in the wrong place; the whole ball at 0.5 m fits to 1.3
  deg. Freeing the camera height and pitch fits all five to 0.4-1.6 deg
  (h 19.2 cm, pitch 5.76). **So the real colour camera's pose and/or lens
  distortion differ from what is modelled. B5 prerequisite:** record
  /camera/color/camera_info (D) and the TF to camera_color_optical_frame,
  put them in the detector, and re-run this test until the touching frames
  fit (< ~1.5 deg) at x ~ 15.9 cm.
- **B4 result (2026-09-28): PASS.** `kaya_lab/scripts/b4_eval.py`
  (env `tasks/ball_goal/ball_goal_b4_env.py`): the B2 policy (exported
  `policy.pt`, unchanged), the robot's own Camera_rgb rendered at 848x480
  each policy step, the image detector `ball_detector.py`, 32 solver
  iterations, nominal servos, the real 46 cm box, level-1 starts, 16 envs.
  - Baseline, same script with the stand-in detector: **97%** (100 episodes).
  - Rendered images + image detector: **90.6%** (64 episodes; 81% on another
    32). Losses are mostly timeouts.
  - Detector vs truth: every visible ball detected; median error 0.6 cm, p90
    1.5 cm (better than the stand-in's modelled noise). "Detections while not
    visible" are mostly real glimpses at the image edge (median error 1.1
    cm); true phantoms (> 50 cm off) are 0.14% of frames.
  - Three bugs found on the way, all sim-side: (1) the camera sensor's pose is
    not refreshed after the robot is moved at reset: take the camera-in-base
    transform from the USD; (2) at 5 m spacing every camera saw the
    NEIGHBOURS' balls: 40 m spacing; (3) **the brown cardboard box passed the
    first HSV threshold** (rendered H 17-19, S ~100): now H 20-34, S >= 120.
    The real ball measures H 20-28 (median 23-26), S ~200, so it still
    detects in all five real frames.
  - Debug markers render INTO the robot's camera: debug_vis off for B4.
  - **For B5:** check real cardboard's HSV under real light. If colour ever
    fails, replace only the blob-finding step with a learned detector
    (rendered + real images); the sphere fit and the policy stay unchanged.
- **B4:** sim-to-sim: run the policy in the full-fidelity scene (RTX-rendered
  colour camera plus a real HSV detector on it, full roller physics,
  `KayaDriver`).
- **B5 FIRST REAL GOALS (2026-09-28).** `kaya_lab/robot/ball_policy_node.py`
  (numpy policy, image detector, odometry memory, goal from a taped start
  mark), on branch isaac-sim-usd deployed to the robot (expo services
  disabled, `expo` branch kept as backup). Setup: box mouth 1.50 m from the
  battery face (goal_x 1.57), ball ~40 cm ahead. Logs and snapshots:
  `kaya_lab/data/b5_runs_20260928/`.
  - The ball is **14.0 cm (circumference 44 cm), 112 g**, not 15.3 cm. The
    wrong size had made every real-frame fit fail (4-6 deg); with 14 cm they fit
    at 0.24-0.86 deg with the robot's own camera calibration. B2 fine-tuned 50
    iterations on 14 cm (100% in sim):
    `~/IsaacLab/logs/rsl_rl/kaya_ball_goal_det/2026-09-28_11-25-06_ball14cm_finetune/model_350.pt`.
  - **Run 1: SCORED in 29 s**, confirmed by the user. For ~12 s it chased
    phantoms 4-5 m to the side, then found the ball, drove behind it,
    pushed it 80 cm and strafed to line up at the mouth.
  - **Run 2: SCORED in 34.8 s.** Phantom rejection added: sphere-fit residual
    and play-area gates. A 1.5 deg fit gate rejected the REAL ball close up
    (moving: 1.8-3.1 deg) and caused hesitation; phantoms (a wooden desk edge)
    fit at 12-17 deg. The gate is now 5 deg.
  - Real brown cardboard under room light partly passes the colour threshold
    (speckles), as the B4 sim run predicted.
  - Jetson: detector 56-75 ms per frame, policy 0.13 ms; no PyTorch or ONNX
    on the robot, the policy runs in numpy (matches TorchScript to 1e-5).
  - The dry run is not meaningful for actions: "last action" feedback with a
    robot that doesn't move saturates the outputs.
  - **Runs 3-7, same setup** (logs in the same folder):

        run 3  timeout   a big lit cardboard box: the single-biggest-blob
                         detector rejected it and never looked at the real ball
        run 4  no goal   ball drifted right into a corner, robot could not
                         recover; cuVSLAM jumped 2 m (camera facing a plain wall)
        run 5  stopped   ball merged with brown cardboard up close (colour);
                         cuVSLAM jumped 0.33 m -> new jump guard stopped it
        run 6  stopped   perfect detection (0 rejections) and smooth wheel
                         odometry; ball pushed 60 cm in 4 s then ran off left
                         past the mouth, robot crossed the 1 m side limit
        run 7  SCORED    20.9 s with the 0.10 m/s policy: steady push, no
                         runaway, corrections at the mouth

    What changed along the way:
    - **Detector:** best of the 3 biggest blobs; then **YOLO11s**
      (pretrained COCO "sports ball", TensorRT FP16 on the Jetson, 6.4 ms)
      finds the ball, and the sphere fit runs inside its box. YOLO misses a
      ball cut off by the image edge, so colour is accepted only in a
      touching zone (bottom edge, <= 35 cm). ~40-55 ms per frame in total.
    - **Wheel odometry** (drive_node /odom) instead of cuVSLAM for the game:
      no jumps.
    - **0.10 m/s fine-tune** (`...slow010/model_400.pt`, 100% in sim, ~19 s per
      goal): the user saw 0.15 m/s kick the ball away.
  - **Still open:**
    - fine-tune YOLO on cut-off balls (auto-label with the sphere fit), so
      colour leaves the loop entirely;
    - walls, corners and recovery (section 2b);
    - brown cardboard near the ball can still merge with it in the colour
      fallback.
- **B5 (real robot):** a ROS 2 node with the HSV detector and odometry goal
  in, `/cmd_vel` out. On the floor at home: the ball and a tape goal square.
  Needs the pending robot session first (frame-sync check, URDF camera fixes
  deployed).

**The real ball (arrived 2026-09-27):** 15.3 cm, **113 g**, air-filled
rubber; grips the tile and rolls easily. Yellow with black "claw" stripes,
so the detector needs a morphological close (15x15) to keep one blob.
First camera check (848x480 colour, HSV H 18-38, S >= 90, V >= 80): at
~0.50 m it is a clean 124 px disc; its reflection in the tile is NOT picked
up.

Close-range check, ball **touching the front, centred** (2026-09-27):
- Still visible: a yellow cap **354 x 97 px**, top edge at row 385, so it
  blocks only the **bottom 20%** of the image (plan predicted 450 x 130 px /
  27%: the ball sits a little lower in view than the CAD estimate).
- About **17% of the ball's projected disc** is in frame (predicted 29%).
- Width-from-blob range is WRONG once the ball is cut by the image edge
  (it gave 0.18 m). **Fit a circle to the top arc instead**: radius 200 px,
  centre (481, 586), RMS 2.8 px. (A circle is only an approximation this
  far off-axis; see the exact sphere fit below for distances.)
- The circle centre's x gives the ball's lateral offset while pushing, so
  the "ball slipping sideways" signal exists on the real robot.
- The sim detector model must reproduce this: a touching ball is visible
  as a cut-off arc, not lost. Bearing from the colour image needs the
  colour camera's extrinsics (it is not the left IR lens).

Ball **touching the front, pushed sideways until it also touches a front
wheel** (2026-09-27):
- Still visible: 350 x 94 px cap, bottom 20% of the image, same as centred.
- Circle-fit centre moved from x=481 to x=334 (147 px, fit RMS 6.9 px;
  stripes and glare make it noisier). At ~0.105 m optical depth that is
  only **~4 cm of lateral travel** from "centred" to "against the wheel".
- So the front wheels bound the ball: the gap between them acts as a shallow
  natural pocket. The sim must keep ball-vs-wheel/roller collisions, and a
  ball touching a turning omni wheel will be kicked, which the policy has to
  learn to avoid.
- The pocket's edge is visible (x ~ 334 of 848): the detector sees the ball
  across the whole push range, so no blind lateral zone while dribbling.

Ball **touching the front and the OTHER front wheel** (2026-09-27):
- Visible: 372 x 101 px cap, bottom 21%, now also cut by the right image
  edge; circle fit noisier (RMS 12.8 px: stripes on the top arc).
- Circle centre x = 664. Colour intrinsics from `/camera/color/camera_info`:
  fx 404.9, fy 405.1, cx 423.5, cy 237.0.
- In pixels the ball centre spans x = 334 .. 664 between the two front
  wheels; the midpoint (x ~ 499) is right of the principal point, so the
  pocket is off the colour lens's centre line. Metric numbers below.
- All three touching positions: ball visible, 15-17% of its disc in frame,
  top edge at rows 381-387. Pushing never hides the ball.

Ball **~10 cm in front, not touching:** 261 x 144 px, bottom 30% of the
image, ~57% of its disc in frame, clean detection.

**Exact sphere-silhouette fit** (scratch script, to become the real
detector's range estimate): the silhouette of a 15.3 cm sphere resting on
the floor, least squares over the mask's edge pixels (image-border pixels
excluded), grid-initialised (plain least squares found wrong local minima).
Camera height and pitch fitted jointly over all five frames:
h = 19.2 cm, pitch 5.76 deg up (documented left-IR values: 17.5 cm, 4.89
deg; with five frames this is NOT a calibration, the two trade off with
range). Ball centre, relative to the point below the colour lens:

    frame                  X ahead   Y (left+)   rms
    ~0.5 m, whole ball     54.1 cm   -4.8 cm     0.40 deg
    10 cm gap              25.9      -3.8        0.58
    touching, centred      13.9      -1.7        1.61
    touching, one wheel    12.4      +3.6        2.31
    touching, other wheel  13.7      -8.4        1.47

- 10 cm gap vs touching: 12 cm apart (user placed ~10 cm). Consistent.
- **Pocket width: about 12 cm of lateral travel (Y +3.6 .. -8.4), centred
  ~2.4 cm right of the colour lens.** The earlier 8.6 cm pixel-scale guess
  was too small. Cut-off, striped arcs make the touching fits good to only
  a few cm; model that as detector noise in sim.
- **Depth cannot help when touching:** the ball's near surface is ~5-6 cm
  from the lens, well inside the 0.17 m depth minimum. Up close, range must
  come from the colour silhouette.

**Sim vs real contact check** (`kaya_lab/scripts/b0_ball_contact_probe.py`,
2026-09-27): Kaya creeps into a ball at 17 lateral offsets.
- Flat front over +-6 cm: ball centre 13.5 cm ahead of the colour lens
  (real: 12.4-13.9). In lens coordinates the flat front spans Y -8.4 ..
  +3.6 cm, **exactly** the real pocket (-8.4 .. +3.6). The colour lens sits
  2.4 cm left of the robot centre line (x 2.4, y 2.4, z 13.8 cm in
  base_link), which is the real frames' 2.4 cm pocket offset.
- 8-12 cm off-centre the front wheels funnel the ball back towards the
  middle; beyond ~15 cm it slides off the side. A natural cradle.
- Straight 10 s push at 5 cm/s: the ball stays in front for offsets within
  +-12 cm.
- The bumper worry (deck edge above the ball centre) did not show up: no
  wedging in sim. Still worth the front photo.

**The goal, revised 2026-09-28: an open-fronted cardboard box** lying on its
side, mouth **46 cm wide x 30 cm tall, 30 cm deep**, cardboard inside on
every face. It replaced the frame below: the whole ball fits inside, so
success is strict (the WHOLE ball behind the mouth, between the sides), and
the box stands free. Kaya is 19.1 cm tall and 30.6 cm wide (USD bounding
box), so it can nose in to finish a push. Training randomises the mouth
width over 40-70 cm (kinematic side panels moved per episode) so the policy
learns to aim; the policy is told the width. A scripted "drive forward"
check at curriculum level 0 scores in 95% of envs
(`kaya_lab/scripts/b1_scripted_check.py`).

*Superseded:* **The goal (user's build, 2026-09-27):** cardboard standing against a wall,
mouth 120 cm wide x 60 cm tall, 12 cm deep. The ball (15.3 cm) cannot fit
in 12 cm, so success = football rule: ball centre across the goal line
between the posts. The wall behind stops the ball, so the policy does not
have to stop it. Where things come from: the goal from localisation (a fixed
point in the map), never the camera, so brown cardboard is fine. The ball
from the colour detector (sim: a simulated detector output; no images are
rendered in training).

**Waiting on the user:**
1. ~~the ball's weight and exact diameter~~ (done, above);
2. **how far it rolls on the tiles after a gentle push** (sets rolling
   resistance);
3. **a photo of Kaya's front** (the bumper question).

---

## 2b. Ball v2: "play like a footballer" (the user's idea, 2026-09-28)

**Seen on the real robot:** a pushed ball tends to drift sideways; once it is
against a wall or in a corner, the policy cannot get it back. That is not
bad luck. The v1 arenas had **no side walls** (a ball leaving the area just
ended the episode as a failure), and the policy only ever *pushes*. Kaya
cannot pull a ball, so recovering one from a wall needs different skills:
- squeezing in between the wall and the ball;
- rolling it out along the wall;
- knocking it off the wall at an angle;
- then re-approaching to push again.

**v2 training:**
- arenas with walls on all sides, corners, and random obstacles (boxes,
  chair-leg posts), randomised per episode;
- the ball bounces off walls instead of the episode ending; out of bounds is
  gone;
- the curriculum includes balls placed against walls and in corners;
- the reward still counts progress towards the goal, so a ball stuck in a
  corner pays only once it is recovered;
- also add a small penalty on ball speed, for control: dribble, don't kick.
- The same detector interface (YOLO + sphere fit), so the robot node is
  unchanged.

**Built 2026-09-28: `Kaya-BallGoal-V2-v0`** (`tasks/ball_goal/ball_goal_v2_env.py`,
config in `ball_goal_env_cfg.py`):
- **Room:** 4 walls; the box stays on the +x wall; the back wall is 1.5-3.0 m
  back and each side wall 1.2-1.9 m out, redrawn every episode (kinematic
  walls).
- **Obstacles:** 4 slots, 15 / 22 / 28 / 35 cm cubes, 30 cm tall. Each is
  present with probability 0.2 (level 0) to 0.7 (level 1), placed clear of
  the ball, the robot and the goal mouth.
- **Wall starts:** up to 40% of starts (by level) put the ball against a wall
  or in a corner, including the corners by the goal wall.
- **Depth scan:** 16 rays over the 92 deg view, level, at camera height
  (passes over the 14 cm ball), max 3 m, 2% noise. Analytic (walls and
  slab-tested boxes), verified against brute-force ray marching to 0.2 cm.
  On the robot: the depth image row that looks level.
- **Observations:** 31 (B2's 15 + scan); critic 35. Network 256-128-64.
- **Extra rewards:** ball speed above 0.3 m/s (-0.3 per m/s); robot within
  3 cm of a wall or obstacle (-0.1 per step).
- A config class must live in the config-only module: Isaac Lab imports it
  before the simulator starts, and an env module there (it imports USD)
  crashed SimulationApp at startup.

**v2 / v2.1 on the real robot (2026-09-28, logs in data/b5_runs_20260928):**

    run  policy  setup                    result
    8    B2 slow straight                 SCORED 15.4 s (re-inflated ball)
    9    v2      straight                 SCORED  9.2 s
    10   v2      straight                 SCORED  9.8 s
    11   v2      ball at right wall       SCORED 13.7 s (freed it from the wall)
    12a  v2      corner beside the box    no - ball missed 33 s (YOLO at exactly 0.30)
    12b  v2      corner beside the box    FALSE goal (ball against the side panel)
    12c  v2      corner beside the box    FALSE goal (odometry drift)
    13   v2.1    corner beside the box    SCORED 30.6 s, but the node did not stop
    14   v2.1    corner beside the box    no - ball knocked away; FALSE goal again

- Detector fixes: YOLO 0.15 (still gated by the sphere fit); any colour blob
  that fits as a sphere to 1.5 deg.
- Scored check: inside-the-box lane (|y| < half_w - r + 3 cm) AND seen in
  front of the mouth first.
- **Localisation is now the bottleneck.** cuVSLAM loses tracking with the
  camera up against plain cardboard or wall (runs 13 and 14), and wheel odometry
  drifts 20-30 cm under hard strafing. Fused odometry (cuVSLAM, falling back
  to the wheels on a jump or a lost height) helps, but not up close.
- **Next: see the goal.** A printed ArUco marker (DICT_4X4_50, ID 0, 150 mm;
  `data/aruco/`) on the box's inside back panel gives the box pose directly;
  between sightings, carry it with odometry (like the ball's memory). No
  retraining: the policy still gets "goal point, goal direction".
- v2.1 = v2 + jam penalty (-0.2 per step pushing a jammed ball) +
  side-panel starts (30% of wall starts); 99% in sim.
  `kaya_ball_v2/2026-09-28_16-14-37_v21/model_400.pt`.

**Speed:** the user found 0.15 m/s a bit fast (the ball runs away). Retrain
or fine-tune at a lower top speed rather than scaling actions down on the
robot. First try: a 0.10 m/s fine-tune (`run_name slow010`).

**Night of 2026-09-28/29: fully on-board, v2.2 -> v2.4 (runs 19-34, filmed from run 20).**
No overhead camera and no new markers: the IMU, the three box markers, and
the depth camera. Every run from 20 on was filmed by the user; the videos
(1 frame/s contact sheets next to the logs) overruled the logs several times.

    run  policy  start                    result
    20   v2.2    right of the box         no - pinned against the panel's outside 25 s
    21   v2.2    right of the box         FALSE goal (stall froze drift; one bad frame)
    22   v2.2    right of the box         no - pushed into the panel 16 s; false goal
    23   v2.2    right of the box         no - same; false goal
    24   v2.3    right of the box         SCORED 55 s (pinned ~18 s, then 12 s in the empty box)
    25   v2.3    left                     no - box re-anchored 58 cm early
    26   v2.3    left                     FALSE goal - the ROBOT drove into the box
    27   v2.3    left                     SCORED 18.8 s
    28   v2.3    behind the robot         no - chased cardboard (colour-only detections)
    29   v2.3    behind the robot         no - spun 90 s: the real ball rejected as "outside"
    30   v2.3    behind the robot         no - stopped; colour gate rejected the far ball
    31   v2.3    behind, right            no - found it; 1.5 m side limit stopped it
    32   v2.3    behind                   SCORED (real) - uncalled; heading 65 deg off (IMU queue)
    33   v2.3    behind, left             no - reached the box from the side; ball "outside" 124x
    34   v2.3    behind, left             no - again at the box's side, against the left wall

v2.2: 0 of 4 from beside the box. v2.3: 3 real goals in 11 runs, including a
ball starting behind the robot (search, find, dribble 2.5 m, score).

**The one policy failure left:** from beside the box it goes for the mouth
from the side; the ball clips the panel's front corner and ends up against
the panel's outside (or the wall beside the box), and the robot presses it
there. v2.4 (`Kaya-BallGoal-V24-v0`, fine-tuned from v2.3) trains exactly
this: the user's measured ball (2,200 N/m squash, grip 1.0), close-range depth
blindness (< 0.3 m) for walls and obstacles, starts beside the box up to 35
cm in front of the mouth, and a penalty for the ball against a panel's outside.

**Robot-node fixes, each found by a run (ball_policy_node.py):**
- stall hold freezes all motion (run 20's video: wheels "drove" a pinned robot 1.5 m);
- three agreeing rejected marker sightings re-anchor the box (run 21);
- the box's part of the depth scan is computed from its marker pose (run 22:
  the real depth is blind to the panel the ball is pinned against);
- a goal needs 3 fresh detections inside over 0.5 s, a box seen within 10 s,
  and the robot's centre < 12 cm past the mouth line (runs 21, 22, 26, 32);
- colour-only detections need a YOLO-backed track, or 3 steady sightings
  (runs 28, 30); balls behind the start are real (run 29);
- no impact stalls while turning > 0.5 rad/s (run 29);
- the IMU queue keeps every sample (depth 1 lost ~65 deg in a spin, run 32);
- "ball beyond the goal wall" is judged in the box frame (run 33).

**v2.4 on the real robot (2026-09-29 ~02:15, fine-tuned from v2.3, `kaya_ball_v2/2026-09-29_00-25-07_v24/model_1097.pt`, `~/b5/node/policy_v24.npz`): 3 goals in 4.**

    run  start                              result
    35   beside the box, right              SCORED 49.7 s - pinned at the panel's corner, backed off, came round the front
    36   behind the robot, left             SCORED 42.0 s - brought the ball across the mouth before going in
    37   corner: right panel + back wall    SCORED 73.7 s - got it out of the corner
    38   beside the box, left               no - wheels snagged on the side panels while turning (the user)

Sim (v2.4 conditions, 512 episodes): beside the box v2.3 80.7% / 19.3 s ->
v2.4 89.1% / 11.5 s; corner 80.1% / 22.3 s -> 83.4% / 14.6 s.

**Next (v2.5, written and smoke-tested, NOT trained - the user chose to stop
here):** the bump penalty never counted the goal's side panels, and in sim the
rollers slide off rigid cardboard, so rubbing a panel was free; the real wheels
snag on it. `Kaya-BallGoal-V25-v0`: panels count as bumps, and -0.05/step for
the robot's centre > 5 cm into the goal (runs 24, 26). Fine-tune from v2.4
model_1097, ~300 iterations (~95 min).

**Earlier open question (answered: yes, mostly):** does v2.4 stop going for the goal from the side? Sim comparison
(`scripts/v2_corner_eval.py --cfg v24`, side and corner starts) then the same
filmed starts on the robot. Also: a small penalty for the robot inside the
empty goal (run 24).

## 3. Demo 2: warehouse inspection (SECOND, sim only)

**Task:** in a warehouse with shelves and aisles, Kaya visits a list of
inspection points and **points its camera at each shelf location while moving
through the aisles**. Scoring is time and coverage.

The comparison: the omni robot sweeps its camera onto each target while
still moving; the differential-drive robot must stop, turn, look, turn back
and restart. It is the industrial pitch, and needs no physical build.

- **Environment:** Isaac Sim's warehouse,
  `/Isaac/Environments/Simple_Warehouse/full_warehouse.usd`, plus procedural
  aisle layouts for training variety.
- **Observations:**
  - depth from a **ray-cast pinhole camera** with the Gemini's intrinsics
    and pose, which is cheap (cf. `MultiMeshRayCasterCameraCfg` +
    `PinholeCameraPatternCfg` in `manager_based/drone_arl/navigation`);
  - the next target's position and look-at direction, from the map;
  - odometry and memory.
- **Rewards:** progress towards the target, the camera pointed at it (a
  "capture" when aligned within a tolerance), a time penalty, collisions.
- **Milestones:**
  - W1 privileged;
  - W2 ray-cast depth;
  - W3 the vy = 0 comparison with videos;
  - W4 full-fidelity sim-to-sim.
- **Real world:** later, possibly, using the saved map's places ("go look at
  the kitchen, then the printer") from the existing scene memory.

---

## 4. Risks

- **Throughput.** 40 links per robot at 240 Hz may cap the environment count.
  M0 measures it first.
- **Ball physics fidelity:** rolling resistance, spin, and the contact with a
  flat front at 0.1-0.15 m/s. Match the real ball's roll-out distance; if
  the real ball behaves differently, randomise over it.
- **Push-dribbling may be hard to learn.** Curriculum: ball already touching
  the front at start, then further away; goal close, then far. Only then the
  cradle fallback.
- **The forward-only camera during sideways motion.** Memory has to work.
- **Isaac Lab 3.0 is a beta** on Isaac Sim 6.0's experimental APIs. Pin the
  versions in the tutorial.
- **Real-world detection:** glare on shiny tile and lighting changes. Tune
  the HSV range on real frames, and use matte ball paint.

---

## 5. Tutorial outline (the public end product)

1. Build a Kaya: hardware, flashing, provisioning (`GETTING_STARTED.md`)
2. Measure your robot: what matters and how (`kaya_description/cad/`)
3. From CAD to a simulated robot: USD, articulation, the omni-wheel problem
4. Is the sim right? Tilt, step and camera checks against the real robot
5. Isaac Lab: many Kayas, a twist action, drive to a point (M0-M1)
6. **Ball to goal**: detector-in-the-loop observations, push-dribbling
   rewards, curricula (B1-B2)
7. What holonomy buys, measured (B3, W3)
8. From sim to sim to real: the ball demo on the real robot (B4-B5)
9. **Warehouse inspection**: ray-cast depth and look-at rewards (W1-W4)

Each chapter ends in a runnable command and a number to compare against.

---

## 6. Where to start in the new session

1. Read section 0, then `kaya_description/usd/README.md` and
   `kaya_description/usd/kaya_isaac.py`.
2. Smoke tests:
   `~/isaac-sim/python.sh kaya_description/usd/kaya_sim.py --test --device cuda`
   and `cd ~/IsaacLab && ./isaaclab.sh -p scripts/environments/list_envs.py`.
3. Start **M0**. The ball measurements are not needed until B1; M0 and M1
   come first either way.
