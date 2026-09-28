# Kaya RL: learning log

A running record of what we did, **why**, and what we measured, written so
that you can learn from it and it can later grow into the tutorial chapters
in `SIM_RL_PLAN.md` section 5. Newest entries at the bottom of each section.
Jargon is defined the first time it appears.

---

## 2026-09-27 - Can the camera see the ball when Kaya is pushing it?

**Question.** Demo 1 has Kaya push a 15.3 cm ball with its flat front. If the
ball disappears from the camera when touching, the policy is blind at the
exact moment it needs to steer.

**What we did.** Grabbed colour frames (848x480) from the robot with the ball
at 0.5 m, 10 cm, and touching (centre, against each front wheel), then ran a
**colour detector** on them:

- **HSV threshold.** HSV = Hue, Saturation, Value. Hue is "which colour"
  independent of brightness, so "yellow" is one hue range (18-38 in OpenCV's
  0-180 scale) whatever the lighting. Far more robust than thresholding RGB.
- **Morphological close** (dilate then erode, 15x15 px) fills the gaps the
  ball's black stripes cut into the mask, so the ball stays one blob.

**Results.** The ball is visible in every position; touching, it is a
cut-off yellow cap in the bottom ~20% of the image.

**Lesson 1 - check your measuring method, not just the measurement.**
- Distance from blob width (`range = fx * diameter / width_px`, the pinhole
  camera model) is right for a whole ball, but WRONG once the image edge cuts
  it (it said 0.18 m).
- A circle fit to the top arc fixed that, but a sphere far off the optical
  axis projects as a stretched shape, not a circle, so its distances were off
  too.
- The final method fits the **exact silhouette of a sphere resting on the
  floor**. That fit itself first had a sign error (camera tilted down instead
  of up), caught because it fitted the *easy* whole-ball frame badly. Always
  sanity-check a new tool on the case where you know the answer.
- It then fell into wrong local minima; a coarse grid search for the starting
  point fixed that. Least squares only finds the nearest valley.

**Lesson 2 - depth cameras have a minimum range.** The Gemini's depth works
from 0.17 m; a touching ball is ~5 cm away. Up close, range must come from
the colour silhouette. This shapes what the sim detector must imitate.

**Lesson 3 - the robot's shape matters.** The two front wheels form a pocket
~12 cm wide that holds the ball. The simulation must keep ball-wheel contact,
or it trains on a different robot.

Data and scripts: `kaya_lab/data/ball_frames_20260927/`.

---

## 2026-09-27 - The frame-sync bug, verified with an A/B test

**Background.** cuVSLAM (visual-inertial odometry, VIO: estimating the
robot's motion from the stereo IR cameras plus the IMU) needs fresh,
evenly spaced frames. A launch file set `enable_frame_sync` twice in one
Python dict; the later `'true'` silently overrode the intended `'false'`.
Frame sync makes the camera hold fast streams until the slow one (depth) is
ready.

**A/B test.** Same launch, same arguments, only that line different; A run
twice to rule out "it was just warming up". What we measured:

| | sync ON (bug) | sync OFF (fix) |
|---|---|---|
| gap between IR frames, median / worst | 5.7 / 190 ms | 33.6 / 73 ms |
| age of the pose estimate, median / worst | ~180 ms / 1.03 s | 101 / 116 ms |

**Lesson - averages hide the problem.** The *average* rate was ~30 Hz in both.
Only the *gaps* showed the frames arriving in bursts. Measure the
distribution, not just the mean.

---

## 2026-09-27 - M0: many Kayas at once in Isaac Lab

### The pieces (vocabulary)

- **Isaac Sim** simulates physics (PhysX) and renders cameras. **Isaac Lab**
  is a framework on top for *learning*: it runs thousands of copies of a
  scene in parallel on the GPU and talks to RL libraries.
- **Environment (env).** One copy of the task: one Kaya, its floor, later a
  ball. RL needs millions of samples, so we run N envs side by side (N = 64
  ... 4096) in one simulation, all stepped together on the GPU. That is why
  every quantity in the code is a tensor of shape `(N, ...)`.
- **Articulation.** A robot made of rigid bodies connected by joints. Kaya
  is 40 bodies (chassis, 3 wheels, 36 rollers) and 39 joints.
- **Actuator.** What drives a joint. An *implicit* actuator is a PD
  controller solved inside the physics engine: torque = stiffness *
  (target_pos - pos) + damping * (target_vel - vel). We set
  stiffness/damping to `None` so Isaac Lab keeps the values already in
  `kaya.usda`, which were calibrated against the real servo. Re-typing them
  would be a second source of truth, and they would drift.
- **Physics step vs control step.** Physics runs at 240 Hz (the tiny rollers
  chatter at lower rates). The policy decides at 10 Hz, the rate we will use
  on the robot. **Decimation** = physics steps per policy step = 24.
- **Why a servo model in Python.** The real servo doesn't obey instantly: the
  drive node samples `/cmd_vel` at 30 Hz, there is ~27.5 ms of bus and
  firmware delay, then the motor responds like a spring-damper (natural
  frequency 31 rad/s, damping ratio 0.6, which gives a 16% overshoot). A policy trained on an
  instant robot learns to make sharp moves the real one can't follow. So
  every physics step we run that chain, vectorised over all N envs.

(continued as M0 progresses)

### Porting the servo chain: test against the original, carefully

We rewrote the single-robot servo model (numpy) as a batched torch version.
Before trusting it, `scripts/test_servos_match.py` feeds both the same
commands with no simulator at all.
- First run: off by exactly one physics step. Then a "fix" made it worse.
- The real cause: the **original** accumulates time with `t += dt`, and
  floating-point rounding makes its 30 Hz tick land a step early or late now
  and then. The new version ticks on an exact grid.
- So the right test compares each response with the best whole-tick
  alignment. It matches to 2e-6 rad/s.
- **Lesson:** when two implementations disagree, don't assume the new one is
  the one that's wrong. And don't stack guesses: after a fix makes things
  worse, go and look at the data.

### M0 results: speed is what makes RL possible

- **Throughput** = how many simulated robot-steps per second. PPO needs tens
  to hundreds of millions of samples, so this decides hours vs days.
- The 5 batched checks pass for every copy (mass, rest height, forward,
  strafe, rotate at 98-100%).
- Two things mattered, both found by measuring:
  1. **Another Isaac Sim was open** and using the GPU: it halved the speed.
     (You spotted this.)
  2. **Solver iterations.** PhysX solves joints and contacts iteratively. The
     asset uses 32 iterations so the robot holds on a 10 deg slope. On a flat
     floor, 8 gives the same driving accuracy and 2.2x the speed. Different
     settings for different jobs: 8 for training, 32 for validation.
- On a GPU, going from 64 to 1024 robots costs almost nothing, because each
  step is dominated by a fixed launch overhead, not by the number of robots.
  That is why RL uses thousands of parallel environments.

Numbers: `SIM_RL_PLAN.md`, M0 result.

---

## 2026-09-27 - M1: the first RL task, drive to a point

Code: `kaya_lab/kaya_lab/tasks/point_goal/`. Run with `kaya_lab/scripts/rl.sh`.

### The RL loop in one paragraph
The **policy** is a small neural network. Ten times a second it reads an
**observation** (numbers describing the situation) and outputs an **action**.
The environment applies the action for 0.1 s of physics and returns a
**reward** (a number: was that good?). An **episode** is one attempt, from
reset to success, failure or timeout. **PPO** repeats two phases: collect
experience from all envs (the *rollout*), then nudge the network towards
actions that did better than expected. **Better than expected** is judged by a
second network, the **critic**, which learns to predict future reward. The
difference between what happened and what the critic predicted is the
**advantage**. PPO's "proximal" part stops each update from changing the
policy too much at once (the clip parameter, 0.2).

### How M1 is defined (and why)
| piece | what | why |
|---|---|---|
| observation (8) | goal in the robot's frame (x ahead, y left) / 3 m; body velocity / limits; last action | only things the real robot knows from odometry; scaled to about [-1, 1] because networks learn badly from big raw numbers |
| action (3) | vx, vy, w in [-1, 1], x (0.15 m/s, 0.15 m/s, 1 rad/s) | exactly `/cmd_vel`, so the policy deploys unchanged |
| action pipeline | twist -> wheel speeds -> 30 Hz latch -> 27.5 ms delay -> servo response | the policy must learn with the real robot's lag |
| progress reward | distance closed per step / (full speed x 0.1 s) | dense signal: reward every step, not only at the end |
| success | within 5 cm AND below 2 cm/s: +50, episode ends | the speed test makes it stop, not drive through the goal |
| fail | more than 5 m from the goal: -10 | stops it wandering off forever |
| time penalty | -0.05 per step | otherwise dawdling costs nothing |
| action-rate penalty | -0.05 x change in action squared | smooth commands; jerky ones are hard on the real servos |
| episode | 30 s max | 3 m at 0.15 m/s takes 20 s |
| randomisation | servo delay +-30%, wn / zeta +-20%, per episode | the policy must work on a robot slightly different from the model |

**Holonomic vs differential drive:** `Kaya-PointGoal-DiffDrive-v0` is the
same task with vy forced to 0. Comparing the two is how we *measure* what
omni wheels buy.

### The PPO settings (`agents/rsl_rl_ppo_cfg.py`)
- **Networks:** actor and critic are each 2 hidden layers of 64 (ELU
  activation). Observation normalisation keeps running means and variances,
  so inputs stay well scaled.
- **Rollout:** 24 steps per env per iteration. With 256 envs that is 6144
  samples per update.
- **Exploration:** the actor outputs a mean and adds Gaussian noise, which
  starts at std 0.5. PPO shrinks the noise as the policy becomes confident;
  watch "Mean action std" in the log.
- **gamma 0.99:** rewards 10 s ahead still count (at 10 Hz).
- **Adaptive learning rate** targets a KL divergence (how far the policy
  moved) of 0.01 per update.

### Engineering details worth knowing
- **Collision filtering.** In a direct env, Isaac Lab only filters
  collisions between envs automatically when the scene is declared in
  config. Its cartpole example filters on CPU only. We always filter, so
  neighbouring robots can never bump each other.
- **Staggered resets.** At start each env gets a random elapsed time, so
  thousands of envs don't all reset in the same step (the reward curves would
  saw-tooth).
- **Rendering.** Physics 240 Hz, policy 10 Hz, rendering 30 Hz in the GUI.

### How to read the training log
- `Mean reward`: goes up as it learns.
- `Metrics/success_rate`: fraction of finished episodes that succeeded.
  **Target: > 0.95.**
- `Metrics/final_distance_m`: how far from the goal episodes end.
- `Episode_Reward/*`: each reward term separately. This is how you debug a
  reward: which term dominates?
- `Mean action std`: exploration noise. It falls as the policy commits.

### Running it: GUI vs headless
- **Isaac Lab 3.0 is headless by default.** The window only opens with
  `--viz kit`. (The first "GUI" run trained with no window because of this.
  Check a new version's defaults, don't carry old ones over.)
- **Rendering is expensive, physics is cheap.** With the window open, 256
  robots took 31 s per iteration; headless, 4096 robots take 7.3 s. Drawing
  256 detailed CAD meshes 30 times a second costs far more than simulating
  them.
- So the workflow is: watch a few iterations in the GUI to check the scene
  and rewards make sense, train headless with thousands of envs, then
  **replay** the trained policy in the GUI with `rl.sh play ... --viz kit`.
- **Size the floor to the env grid.** 4096 envs at 4 m spacing cover
  ~256 m; the default floor is 100 m. We size it from the grid, and log
  `Metrics/fell_rate` so a robot falling off the world can't go unnoticed.

### M1 result: it learned in 3 minutes, and found a loophole
- **99.8% success by iteration 25**, 100% from 50. After that it only got
  faster and more precise: final distance 3 -> 1.5 cm, episode 15 -> 8.9 s.
  The exploration noise (std) fell from 0.50 to 0.09 as it committed.
- **Reward hacking, the benign kind.** The episodes were shorter than the
  speed limit allows. The policy had discovered that the limits apply to
  vx and vy separately, so going diagonally at full speed on both gives
  0.21 m/s. We checked the real drive node: it clips the same way. So this
  is a real advantage of the omni base, not a sim bug. **RL finds any
  loophole in how you defined the task. When a number looks too good, find
  out why before celebrating.**
- Replay: `rl.sh play ... --checkpoint <full path to model_N.pt>`
  (`--checkpoint` needs the full path). Playing also exports
  `exported/policy.onnx`: the file that will run on the robot.

---

## 2026-09-27 - B1 prep: does the sim ball touch Kaya where the real one does?

`scripts/b0_ball_contact_probe.py`: 17 Kayas creep into balls at different
sideways offsets and record where contact happens.
- The sim's flat front spans exactly the sideways range we measured on the
  real robot with the camera (-8.4 .. +3.6 cm from the colour lens), and the
  depth agrees within ~1 cm. **Two independent measurements (real camera
  frames, CAD-built sim) agreeing is how you earn trust in a simulator.**
- The front wheels funnel an off-centre ball back towards the middle: a
  free "cradle".

**How the robot knows where things are** (your question):
- **Training renders no camera images**: far too slow. It uses a *simulated
  detector output* (bearing and distance, only inside the camera's view,
  with noise and dropouts), modelled on the real detector we measured. On
  the robot, the real HSV detector produces the same numbers from real
  frames.
- The **goal** comes from localisation: the goal is a fixed point in the
  map, and cuVSLAM knows the robot's pose in the map.
- The **ball** is only "seen" when it is in view, so the policy must learn
  to *search*: turn until the ball appears. Memory (B2) lets it remember
  where it last saw the ball.
- **Teacher/student:** B1 trains a "teacher" that is told the true ball
  position (easier to learn). B2 trains the "student" that only gets the
  detector output, and compares.

### M1 vs the vy = 0 ablation
An **ablation** removes one thing and measures the difference; here, the
sideways wheel speed. Same task, same settings:
- omni: 99.8% success by iteration 25, 8.9 s per goal at iteration 200;
- diff drive: 62% by iteration 25, 12.6 s per goal at iteration 200.
Both end at 100% and 1.5 cm. So on an easy task omni mostly buys *speed*
(no turning first, diagonal driving) and *faster learning*. The prediction
to test in B3: pushing a ball widens the gap, because steering a pushed ball
needs sideways correction.

### Replay commands (saved runs)
Close other Isaac instances first (and don't replay while a training run is
going: it would halve the training speed).

    cd ~/work_projects/kaya_robot/kaya
    # M1 omni
    kaya_lab/scripts/rl.sh play --task Kaya-PointGoal-v0 --num_envs 32 --viz kit \
        --checkpoint ~/IsaacLab/logs/rsl_rl/kaya_point_goal/2026-09-27_22-47-16/model_200.pt
    # M1 differential drive (vy = 0): note the different --task
    kaya_lab/scripts/rl.sh play --task Kaya-PointGoal-DiffDrive-v0 --num_envs 32 --viz kit \
        --checkpoint ~/IsaacLab/logs/rsl_rl/kaya_point_goal_diffdrive/2026-09-27_23-21-10/model_199.pt

Swap `model_200.pt` for `model_50.pt` to see a half-trained policy.
The `--task` must match the run: the diff-drive policy only ever learned
with vy = 0.

---

## 2026-09-28 - B1: the ball task, with the box goal

### The goal changed, and why the box is better
Your 46 x 30 x 30 cm box beats the 120 x 60 x 12 cm frame: the ball fits
*completely* inside, so a goal is unambiguous and the ball can't bounce back
out. It is harder to hit (+-15 cm of sideways error allowed instead of
+-52 cm). Training varies the mouth width over 40-70 cm, so the policy learns
to **aim** and is robust to a slightly wrong goal position on the real robot.

### Curriculum learning
Starting with the full task (ball anywhere, robot facing anywhere), a random
policy would almost never score, and so it would never learn what scoring is
worth. So the start positions have a **level** from 0 to 1:
- level 0: ball near the mouth, robot right behind it, facing it;
- level 1: ball up to 2.3 m out and 1.6 m to the side, robot anywhere.
When recent success passes 75% the level rises by 0.05, and it falls if
success drops below 30%. Success therefore hovers near 75% *by design*
while the level climbs. **Read the level, not just the success rate.**

### Two mistakes worth remembering
1. **`pkill -f "<pattern>"` killed my own shell.** `-f` matches whole command
   lines, and the shell running the command contains the pattern too. Edits
   chained after it silently never ran. Fix: `kaya_lab/scripts/stop.sh`
   keeps the pattern inside a file. (The robot's `kaya_run.sh` documents the
   same trap.)
2. **A checker that couldn't see success.** The first scripted check said 0%
   scored, with every ball stopping at +7.3 cm against a +7.65 cm threshold.
   Too close to be a coincidence. The env resets a scored episode *inside*
   `step()` and clears the success flag before the script looks, so a
   success could never be observed. Reading the +100 reward instead showed
   **95%**. The robot's height (19.1 cm) was measured to rule out the box's
   top panel first. **When a result sits suspiciously close to a
   threshold, suspect the measurement.**

### B1 result
- The curriculum reached level 1.0 (the hardest starts) at iteration 100,
  about 25 min in.
- Success at full difficulty: 93.5% at iteration 125, 99.5% at 200, 99.9%
  at 250-300.
- The ball sits in the front pocket about 60% of the time: it really
  dribbles.
- You confirmed in the GUI: it gets behind the ball, corrects sideways while
  pushing (holonomy), and scores.
- **Replay gotcha:** the curriculum level lives in the environment, not in
  the policy, so a plain replay restarts at level 0 (easy starts only). Add
  `env.initial_level=1.0`, and `"env.goal_width_range=[0.46,0.46]"` for your
  real box. `env.<field>=<value>` overrides any config field from the command
  line (Hydra).
- **Teacher caveat:** this policy is told the ball's TRUE position at all
  times, even behind it. The real robot can't know that. B2 fixes it.

---

## 2026-09-28 - B2: seeing the ball only through the camera

Task `Kaya-BallGoal-Det-v0`; code `kaya_lab/kaya_lab/detector.py` and
`tasks/ball_goal/ball_goal_det_env.py`.

### The simulated detector (`detector.py`)
The real robot will run: HSV threshold -> biggest yellow blob -> sphere fit ->
ball position on the floor. In training we skip the images and compute the
same *output* from the true ball position:
- **Projection:** the ball is projected through the colour camera's real
  intrinsics (from the robot's `camera_info`) at the lens position we
  measured (2.4 cm left of centre, 17.5 cm up, pitched 4.89 deg up).
- **Visibility:** it only counts if at least 12 px of it lands on the image.
  So a ball behind or beside the robot is invisible, and a touching ball is
  still visible as a cap at the bottom.
- **Noise:** range noise 1 cm + 3% of distance, plus 2 cm when the ball is cut
  off by the image edge; bearing noise 0.7 deg.
- **Dropouts:** 5% of frames are missed at random.
- **Latency:** the policy sees the detection one step (100 ms) late.

**Checked against the real frames:** for a touching ball, the top-edge rows
come out at 383 and 387 in sim, against 383 and 387 on the real robot. At
0.5 m the ball is 122 px across in sim and 124 px real.
- Known gap: the ideal pinhole model ignores lens distortion (about 2 deg at
  the image edge). The real detector should undistort using `camera_info`.

### Memory without a recurrent network
When the ball is detected, its position is stored in the odometry (world)
frame. Every step it is re-expressed in the robot's current frame, together
with "seconds since last seen". Turn away and the remembered ball stays
where it was; it rolls off and the memory goes stale, so the policy learns
to go and look. The real robot has odometry, so this carries over directly.
A recurrent network (GRU/LSTM) would *learn* its own memory instead: more
general, but slower to train and harder to debug.

### Asymmetric actor-critic
- The **actor** (the policy, deployed) sees 15 numbers: the remembered ball,
  its age, seen now?, the goal, the box width, its own velocity, and its last
  action.
- The **critic** (training only) sees those plus the TRUE ball position and
  velocity: 19 numbers.
- Why this helps: better value estimates make learning faster, and the
  critic is discarded at deployment, so the "cheat" never reaches the robot.
  The log shows it: Actor `in_features=15`, Critic `in_features=19`.

### New metric
`Metrics/ball_seen_frac`: the fraction of time the ball is in view. Expect
it to rise as the policy learns to keep the ball in front of the camera.
(Caution: early in training it reads low because of the staggered start
times. A direct test showed 95% visibility at level 0, as expected.)

### B2 result: the camera-only policy matches the teacher
- Full difficulty at iteration 75. Success: 85% (it. 100), 96% (125),
  99.7% (150), 99.9-100% (200-300). About 14.3 s per goal against the
  teacher's 13.6 s.
- The fraction of time the ball is in view rose from 57% to ~83%: the policy
  learned to keep the ball in front of the camera. Nobody told it to; it
  scores more that way.
- **Why so little was lost:** the explicit memory plus the asymmetric critic
  made the hard part (partial observability) cheap to learn. Worth trying
  first, before reaching for recurrent networks or teacher-student
  distillation.
- In the replay, a disc shows the policy's *belief* about the ball (green =
  seen, red = memory). Visualising what the policy believes is one of the
  best debugging tools there is.

---

## 2026-09-28 - B4: the full-fidelity check

The B2 policy, unchanged, now driven by a **real image detector** on
**rendered** camera frames (`ball_detector.py`: HSV -> blob -> sphere fit).
This is the code the robot will run.

| | success |
|---|---|
| B2 in training (stand-in detector, 8 solver iterations) | 99.9% |
| B4 baseline: stand-in detector, full physics, real box | 97% |
| **B4: rendered camera + image detector** | **90.6%** |

**Four bugs, all caught before the real robot:**
1. The camera's pose went stale after reset. The camera sensor doesn't
   update its pose unless asked, so the detector thought the camera was 1.9 m
   away. Fix: the camera is bolted on, so use the fixed transform in the USD.
2. **Every camera saw the neighbours' balls.** 16 arenas on one open floor.
   A real room has one ball. Fix: space arenas 40 m apart for this test.
3. **The cardboard box looked like the ball.** Rendered brown cardboard has
   hue 17-19 and the first threshold started at 18. The measured real ball
   is 20-28, so the threshold now starts at 20 and needs saturation >= 120.
   *This could happen on the real robot too, so re-check with the real box.*
4. The debug markers appear in the robot's camera image (markers are real
   rendered objects). They are switched off for evaluation.

**How we found them:** by logging the detector against the truth (position
error, detections while the ball "can't" be visible) and by *looking at the
rendered frames and the masks*. Numbers told us something was wrong; the
images told us what.

### Colour threshold vs a trained detector (your question)
The detector has two jobs:
- (1) *find* the ball in the image;
- (2) *measure* where it is.
A learned model (YOLO etc.) is better at (1): it learns shape and texture,
not just colour, so cardboard and lighting fool it less. It still needs (2):
a box around the ball is not a distance. And the policy only ever sees
"ball at (x, y), seen or not", so the detector can be swapped **without
retraining**. Plan: colour first (cheap, 17 ms, explainable); switch (1) to
a learned model if the real room fools it.

---

## 2026-09-28 - B5: the real robot

**It worked:** a policy trained only in simulation scored with the real ball
in the real box: runs 1, 2 and 7 (29, 35 and 21 s). The policy's own
behaviour carried over: get behind the ball, push, strafe to line up. Every
failure was in **perception or localisation**, not in the learned behaviour.
That is the typical sim-to-real story.

**The big lesson: measure the physical thing first.** The ball was 14 cm,
not 15.3. With the wrong size, no real frame would fit, and three wrong
theories followed (distortion, stripes and glare, camera height: fits can
trade ball size against camera height). A tape measure settled it in one
step. Verify inputs before you debug outputs.

**Perception, in the order it failed:**
1. Colour threshold only: fooled by warm light on wood and cardboard.
2. Roundness gate: too strict at first; the real ball moving up close fits
   at 2-3 deg. Set thresholds from measured data (real 1.8-3.1 vs phantoms
   12-17), never from guesses.
3. Biggest blob only: a big cardboard patch hid the real ball. Look at
   several candidates.
4. YOLO11s (pretrained): finds the ball and ignores cardboard, but misses a
   ball cut off by the image edge, so colour is kept only for the touching
   zone. Next: fine-tune YOLO on cut-off balls.

**Localisation:** visual odometry can jump when the camera faces a plain
wall. For a short game on a small floor, wheel odometry is the better tool:
it drifts slowly and never jumps. Pick the sensor for the job.

**Speed:** retrain for a new top speed rather than scaling actions on the
robot. The 0.10 m/s fine-tune stopped the ball running away (run 7).

**Tools that paid off:**
- a dry-run mode;
- saving a snapshot of every rejected detection (it showed the cardboard
  box, the desk edge and the merged blob);
- logging the detection source and the fit residual on every step;
- odometry-jump and play-area stops.

---

## 2026-09-28 - v2: the footballer (walls, corners, obstacles)

**Why v2 was needed (seen on the real robot):** a ball that drifted into a
corner was unrecoverable. In v1 training there were no side walls, so a
drifting ball just ended the episode, and the policy never met a ball in a
corner. **A policy can only be good at situations it trained on.**

**What changed:** the room, obstacles and wall starts are in
SIM_RL_PLAN 2b. The two ideas worth learning:
- **Give the policy the sensor the task needs.** Avoiding walls needs to
  *see* walls, so v2 adds a 16-ray depth scan. It is taken at camera height,
  so it passes over the ball, and it is one row of the real depth image, so
  it deploys.
- **Scale difficulty with the curriculum, including the environment.** More
  obstacles and more wall/corner balls appear as the level rises (your
  suggestion). At level 0 the rooms are nearly empty so the basics are
  learned first.

**Checking the checker again:** the scan is computed with formulas (fast),
and was verified against a slow, obviously-correct method: stepping along
each ray 2 mm at a time. They agree to 0.2 cm, the step size.

**Engineering gotcha:** Isaac Lab imports a task's *config* module before
starting the simulator. Put the config class in a file that imports
physics/USD code and the simulator crashes at startup. Configs go in
config-only files.

---

## 2026-09-28 - v2 on the real robot, and v2.1

**v2 on the real robot:** runs 9 and 10 scored in 9.2 and 9.8 s (the old
policy took 15.4 s in the same setup). Run 11 got a ball off the right wall
and scored in 13.7 s: the v1 policy failed exactly this in run 4.

**The corner beside the box (runs 12a-c): 0/3.** Three different lessons:
1. **Detection:** the ball was in view but missed for 33 s. YOLO scored it at
   exactly 0.30 (threshold > 0.30), and the colour fit was perfect but outside
   the touching zone. A probe that *saves the frame and prints every
   candidate* showed it in one shot. Fixed: YOLO 0.15 (its box must still
   pass the sphere fit), and any colour blob that fits as a sphere to 1.5 deg.
2. **False goals, twice.** The scored check was too loose: a ball outside a
   side panel counts as "inside" by the mouth's width. It is now tightened
   to inside-lane and must have entered through the mouth. Then wheel-odometry
   drift (20-30 cm after 40 s of hard strafing) faked "through the mouth".
   **When the robot's estimate and your eyes disagree, your eyes win.** The
   lasting fix is to see the box.
3. **The robot jams the ball against the side panel** (your observation: it
   pushes hard, the ball squeezes and can't come out; it should tap and
   back off).

**v2.1, and a wrong hypothesis caught in time.** I assumed the sim ball was
too rigid and slippery to jam, so I made it squashier and grippier. A
scripted test pushed balls into a panel with both: the OLD ball jammed 90%
of the time, the new one 72%. The physics was never the gap. The missing
pieces were:
- the *situation* (balls rarely started beside a side panel);
- a *reason to stop pushing* (a jam cost nothing but time).
v2.1 adds side-panel starts and a jam penalty, and keeps v2's ball.
**Test your fix does what you think before you train on it.**

## Fully on-board: stall detection (2026-09-28, night)

Up against a wall, the wheel encoders say the robot is still moving (0.08 m/s
while blocked) and the camera sees plain cardboard: both of its senses lie.
A third one doesn't: **the IMU feels the hit.** A wall hit is a -1.27 m/s^2
deceleration spike. A free ball push has none. The detector:
- averages the forward deceleration over 0.1 s (run 19 used 3 samples, so
  floor vibration read as impacts: 552 of 589 steps "stalled");
- when it goes over 0.6 m/s^2, freezes the wheel distance;
- **holds that for 3 s while the command still pushes the same way.** A 0.7 s
  hold let the wheels creep "through" the wall.

Checks: open floor 4/60 steps flagged; wall froze at 0.34 m and held.

**Feed the policy what the sim would.** When stalled, the policy's speed
input is now 0, because in the sim a blocked robot's speed IS 0. Before this,
the policy saw "moving forward at 0.08" while pressed on a pinned ball, and
simply kept doing what worked while moving.

## v2.2: strike off-centre (your idea)

Pushing a ball square-on into a surface drives it straight into the surface.
Struck off-centre with a front wheel, it has a side to escape to. v2.2 adds a
small penalty for pushing forward with the ball *centred* in the pocket next
to a wall, box panel or obstacle, and a top speed of 0.10 m/s.

## v2.3: back off, then kick (your idea, run it like a footballer)

Your words: "step back, then strike it from the right wheel or left wheel as
a kick, and then come back and see the next move". Two changes to the reward:
1. **The jam penalty escalates.** Each consecutive jammed step costs
   `rew_jam * (1 + 0.15 * steps jammed so far)`, so after 2 s the press costs
   4x as much. Holding a jam gets steadily more expensive.
2. **The kick bonus (+1, once per jam) pays only for your recipe.** Within
   2 s after the jam ends, the robot must first have **driven backwards**,
   and the ball must then move off (> 0.10 m/s) while it sits **to one side
   of the robot's front** (5-20 cm off the centreline, i.e. in front of a
   front wheel). A square-on shove that happens to free the ball earns
   nothing.

**Why a condition-heavy bonus instead of just "ball moved = good"?** RL
optimises what you pay for, not what you meant. "Ball moved after a jam"
would also pay a robot that jams, shoves harder and gets lucky. Encode the
behaviour, then watch that the term fires for the right reason.

This is a **fine-tune** from the v2.2 checkpoint (model_549), not training
from scratch. The policy already scores; we only reshape how it gets out of
jams. It starts with 0.25 kick reward per episode because v2.2 sometimes did
this by chance, and PPO makes it more likely.

**A bug found on the way:** every training run since B1 printed "Patch
buffer overflow" once, at the first reset. PhysX was dropping contacts on
that step: the v2 rooms needed 1.08M contact patches and got 262k. That was
harmless here (one step, before learning starts), but a dropped contact can
let a ball fall through a wall. The v2 configs now reserve 2^21. **Read the
errors in your logs even when training "works".**
