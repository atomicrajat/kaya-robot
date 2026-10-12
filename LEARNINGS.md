# The learnings: an index

Everything this project learned is already written down, but in four places
and in the order it happened. This file is the map: one line per lesson, in
teaching order, pointing at where the full account lives (a file and section,
or a commit hash to open with `git show <hash>`). It ends with the chapter
outline for the explainer video.

The four sources:

| source | what it holds |
|---|---|
| `git log` (116 commits) | the navigation project, 2026-08-27 to 2026-09-10. Each commit body is a write-up: the fault, how it was found, the measurement, the fix |
| `ROADMAP.md` | the design, how navigation got to work, what is open, and the settings table |
| `kaya_lab/LEARNING_LOG.md` | the simulation and RL project, 2026-09-27 to 2026-09-29, written to teach, jargon defined |
| `SIM_RL_PLAN.md` | every RL result and every real run, in one record |

Read `kaya_lab/LEARNING_LOG.md` end to end first; it is the only one written as
teaching. Then use the tables below to go into the navigation commits.

---

## Part I: the robot

| lesson | where |
|---|---|
| The module is an Orin Nano 8 GB, not the NX the carrier claims; the SKU fuse (`0xD5`) and zero DLA cores prove it. Every budget decision follows | `robot_setup/SETUP_LOG.md` §2 |
| `nvpmodel -q` reports a mode it has not applied; only the GPU clock table tells the truth, and only a `-super` reflash adds the clock points | `SETUP_LOG.md` §3 |
| Thirteen setup faults with fixes: constrained builds, the wifi driver and its `.zst` firmware, the camera driver patch, the uninstallable nvblox metapackage | `SETUP_LOG.md` §8.3 table |
| Frames: `camera_link` is the left IR lens, not the body; depth sat 5 cm right of the truth for the whole navigation project | `5f5fa43`, `kaya_description/cad/CAD_CONTEXT.md` §2 |
| One parent per frame: cuVSLAM on `base_link` split the TF tree and Nav2 would not start | `67400b9` |
| Omni kinematics, the radial axle, and which servo is which wheel (pulse each one and look) | `ef20a4d`, `371f15b`, `kaya_drive/tools/wheel_probe.py` |
| The rotation constant L is calibrated from a ten-turn spin (138 mm), not read from CAD (143 mm); the two are kept apart | `371f15b`, `cad/kaya_dimensions.yaml` |
| The servo SDK flushes only its output buffer; a stale acknowledgement corrupts the next reply | `b73f12f`, `kaya_drive/kaya_drive/feetech_bus.py` header |
| Measured side by side, the camera's IMU beats the external MPU-6050 on every axis; the MPU was removed | `41b051b`, `README.md` history |
| Gyro bias walks with temperature (7 deg/min from a stale calibration); re-zero while the robot is still, judged from the gyro itself | `d249280` |
| Fusing wheel yaw rate alongside the gyro stops the pure-integral drift (2.25 to 0.83 deg/min); the production answer is an estimator that carries bias as a state, which is cuVSLAM | `17ba587`, `d249280` |

## Part II: seeing where you are

| lesson | where |
|---|---|
| Two routes to SLAM: RTAB-Map on RGB-D, then cuVSLAM on the stereo IR pair. Stereo IR avoids registration, software alignment and resolution mismatch structurally; 6.1 ms per frame against 160 | `d475b7d`, `bcd05d2` |
| Distorted images fed as rectified, and a software aligner passing 15% of frames unaligned: both leave 2D matching intact and kill 3D verification | `8937ce0` |
| The IR projector's dots move with the camera; they restore place recognition but not odometry; off by default | `1dec231`, `1ed1f18` |
| **The `time_domain` regression**: four environmental hypotheses measured and disproved, then one config diff; 14.5-32% drift to 0.63%. Invisible at rest, and validated into place by counters that cannot measure accuracy | `cc4b040`, `eaaa63a`, `ROADMAP.md` settings table |
| The ground-plane constraint made odometry worse (14.5% to 32%) and moved the Z error into the SLAM layer; over-constraining injects error | `64a9087`, `ab1cb7b` |
| Texture, not brightness, predicts VIO drift; the lighting theory was written and retracted twice | `1dec231`, `9f4efe8`, `4fe8cef` |
| The full-resolution point cloud starved the whole camera pipeline (0.46 s to 0.03 s lag after 4x decimation); a safety layer on 1.8 s old data is decorative | `e8fb2bd` |
| Colour at 1280x720 against depth at 848x480 cost 15% of depth frames, twice, because resolution is set per launch file | `9b3a4ce` |
| cuVSLAM's dropped-frame warning fires at 34 ms on a 33.3 ms period: half of them are noise, and a "15% drop rate" was really 7.5% | `9b3a4ce`, `kaya_bringup/tools/README.md` |
| The colour stream starved the LEFT eye only; `stereo_sync` said 0% unmatched and was right; only comparing the two eyes' rates showed it | `523b533`, `8725ad0`, `b2fd345` |
| "The camera stalls under load" was wrong: unbounded IR auto-exposure settled at 50 ms and a sensor cannot expose longer than its frame period | `3aa071a`, `a820bbd` |
| Launch silently drops `enable_ir_long_exposure`; a node that sets it and reads it back exists because of that | `482c450`, `kaya_bringup/kaya_bringup/camera_tuner_node.py` |
| `enable_frame_sync` was set twice in one dict and the later `true` won for 17 days; found by flake8, proven by an A/B on frame gaps, not averages | `d1571e1`, `LEARNING_LOG.md` "frame-sync bug" |
| The IR pair arrives 0.47 s late whenever depth is on, from the on-device depth engine; eight software knobs ruled out | `a820bbd`, `ROADMAP.md` "still open" |
| After a hard power loss the GPU can fail its secure bootstrap; CUDA sees 0 devices while the camera streams | `86e26ef` |
| VIO does not survive being carried; restart after moving the robot by hand | `2917ec0` |

## Part III: maps and navigation

| lesson | where |
|---|---|
| nvblox: TSDF to ESDF to a 2D slice; the stock slice looked a metre over Kaya's head; the heights are written floor-referenced | `5ae406c`, `3bfcf97` |
| nvblox was subscribed to nothing (per-camera topic names) and reported healthy for an hour | `3bfcf97` |
| Dynamic mapping decays voxels: 134 passes at 0.95 is 27 s, about 4 m, and the map was deleted behind the robot; mapping and navigating are separate configs | `02ed67f`, `1febc98` |
| A new config file needs a rebuild before it exists in the install space; check the values the node reports | `1febc98` |
| A saved map is two independent things, and only one was being reloaded | `9f1543c`, `kaya_bringup/scripts/kaya_save_map` |
| Relocalisation: 7 mm from a 1.8 m wrong hint, 5.9 cm from 5 m, then a shrinking radius whose cause is unresolved | `a165a76`, `524a007`, `ROADMAP.md` "still open" |
| cuVSLAM reports localisation outcomes only to its logger, and can say "Successfully localized" 2.4 m from truth; the anchor node reads `/rosout` and checks the geometry before believing it | `8019c62` |
| Five Nav2 template defaults that stopped the robot moving, all silent | `95127be`, `ROADMAP.md` |
| An ESDF cannot say "I have not looked there": nvblox out of the global costmap, `VoxelLayer` locally, static map in both costmaps | `82ba82b`, `f31b8a7`, `5ccdabe` |
| The dynamic mapper never had the slice settings; polished tile fused as walls | `5e6b850` |
| MPPI: a longer horizon made it worse, batch size is not free, `PreferForwardCritic` is for non-omni bases | `90d133e`, `5e6b850` |
| The goal checker latched, then hunted; tolerances come from the measured 10.9 cm rotational excursion; 0.5 rad/s because rotation is what loses tracking | `e9bc7c3`, `9953450` |
| The false "Reached the goal!": Nav2 compares against an uninitialised (0,0,0) when the transform is too old | `a820bbd`, `f515514` |
| The behaviour tree aborted every long goal a second in (20 ms acknowledge timeout) so recovery never ran; the first goal of a session is first-contact discovery | `f87b9ef`, `1c0fbf5` |
| A pose guard that stops the robot when the pose stops being real, armed only after anchoring, with limits from a measured turn | `934f6a4`, `efe9c3b`, `2917ec0` |
| Orphaned containers: ten nvblox processes filled the board and faked two cuVSLAM bugs; launch into a process group | `d3ed387`, `kaya_bringup/tools/kaya_run.sh` |
| `pkill -f` matches the shell that runs it; three ssh sessions died that way | `83c823d`, `f223e5b`, `LEARNING_LOG.md` B1 |
| Every colour frame is bandwidth taken from the stereo pair: 6.3 cm off, ~20 cm at 30 fps | `b2fd345` |

## Part IV: the dashboard and meaning

| lesson | where |
|---|---|
| Foxglove cannot render the custom nvblox and Isaac types; the custom dashboard came forward | `e6752ef`, `dashboard/README.md` |
| A TF buffer returns the last transform forever: a dead robot read as live for ten minutes. Poses carry an age, stale reads null, a goal to an empty topic is a 503 | `ea112a5` |
| `/trajectories` is 7656 sphere markers with empty `points[]`, and must be subscribed RELIABLE; GPU load is per-mille | `cdb5309` |
| Metal with nothing to reflect renders black; the stage is in centimetres and Z-up | `412f907` |
| Remember scenes and let a model choose, rather than rebuild meaning from 80 class labels | `b36836e`, `ROADMAP.md` design |
| The keyframe trigger fired on appearance alone: 428 frames for 62 places; a distance floor fixed it | `38ded7a` |
| Caption once, query many: 1m58s once, then one second per question, flat in store size | `e6b4a3e` |
| A keyframe pose is where the camera was, not where the thing is: `here` versus `beyond`; a domain rule made it worse; a wrong answer named inside a prompt gets reproduced | `1a91641` |
| Where the model runs is decided by latency sensitivity, not preference | `e6b4a3e`, `ROADMAP.md` design |
| Keyframe poses live in the session frame; valid only because the drive started on the mark | `826e77f`, `fbe70e1` |
| Open-vocabulary prompts match appearance, not identity; a 0.12 threshold accepts curtains | `b63ba23` |
| A workstation subscriber on the full-rate depth topic stopped the robot by watching it; throttle at the source | `efe41d9`, `2c52566` |
| Teleop is a deadman, not a latch, and goes through the collision monitor | `29d0b3c` |

## Part V: simulation and learning

Everything here is in `kaya_lab/LEARNING_LOG.md`, in this order; the
results are in `SIM_RL_PLAN.md`. Section names below are the log's.

| lesson | where |
|---|---|
| What the simulated robot cannot carry in its USD: 240 Hz physics, a floor material, position servos fed a moving target, spheres for rollers | `kaya_description/usd/README.md`, `a0a6420`, `781a284` |
| A sim earns trust by matching real measurements: mass, tilt, step response, the IMU at rest, 13 checks on CPU and GPU | `usd/README.md` "Calibrated from the real robot" |
| Can the camera see the ball while pushing it; check the measuring method, not just the measurement; depth has a minimum range | LOG "Can the camera see the ball" |
| M0: speed is what makes RL possible; solver iterations; a second Isaac instance halves throughput; test the port against the original | LOG "M0" |
| M1: the RL loop in one paragraph, every reward term and why, how to read the training log, headless by default, the diagonal loophole | LOG "M1" |
| The ablation: what omni buys, measured (99.8% vs 62% at iteration 25) | LOG "M1 vs the vy = 0 ablation" |
| B1: curriculum learning, read the level not the success rate; a checker that could not see success; the box beats the frame | LOG "B1" |
| B2: a detector modelled, not rendered; memory without a recurrent network; the asymmetric critic; visualise what the policy believes | LOG "B2" |
| B4: four bugs caught on rendered frames before the robot; numbers said something was wrong, images said what | LOG "B4" |
| B5: measure the physical thing first (14 cm, not 15.3); perception failed in order; wheel odometry over VIO for a short game; retrain speed, do not scale | LOG "B5" |
| v2: a policy is only good at situations it trained on; give it the sensor the task needs; configs in config-only modules | LOG "v2" |
| v2.1: a wrong hypothesis (the ball physics) tested before training on it | LOG "v2 on the real robot, and v2.1" |
| Stall detection from the IMU when wheels and camera both lie; feed the policy what the sim would | LOG "stall detection" |
| v2.2 and v2.3: pay for the recipe, not the outcome; the contact patch buffer error that printed on every run | LOG "v2.2", "v2.3" |
| The video beats the log (run 20); the measured ball; why a ball locks in a corner; an eval that nearly lied | LOG "The video beats the log" |
| Sixteen filmed runs: every input is a claim; guards cause the next failure, name the run; a queue of 1 is a bug waiting for load | LOG "Night of 2026-09-29" |
| v2.4: 3 real goals in 4; v2.5 written, not trained | `SIM_RL_PLAN.md` §2b |

## Part VI: the method

The lessons that repeat across both projects, each with the moment it was
learned the hard way:

1. **Verify against the thing, not the counter.** `vo_state` and track time
   reported perfect health through 15-32% drift (`cc4b040`).
2. **Measure distributions, not averages.** Frame sync ON and OFF both
   averaged 30 Hz; only the gaps differed (LOG "frame-sync bug").
3. **Compare both eyes.** The left IR ran 15% slower and pairing reported 0%
   unmatched, correctly (`523b533`).
4. **A stationary test cannot see a motion-proportional error.** The
   `time_domain` bug held 1.4 mm over 94 s at rest (`eaaa63a`).
5. **Diff the configuration against the last commit that worked** before the
   fifth environmental hypothesis (`cc4b040`).
6. **A result suspiciously close to a threshold is the measurement's fault.**
   Every ball stopping at +7.3 cm against +7.65 (LOG B1).
7. **Sanity-check a new tool on the case where you know the answer.** The
   sphere fit's sign error showed on the easy whole-ball frame (LOG, ball).
8. **Two independent measurements agreeing is how a simulator earns trust.**
   The real camera frames and the CAD-built sim gave the same 12 cm pocket.
9. **Read the errors even when training works.** "Patch buffer overflow"
   printed once per run for days (LOG v2.3).
10. **Guards against one failure cause the next one; write down the run each
    one came from** (LOG, night of 2026-09-29).
11. **Film the robot.** The log said 1.5 m of dribbling; the video showed a
    robot pinned for 25 s (`62e7062`).
12. **Measure the physical thing before debugging the model of it.** The
    ball was 14 cm; three theories died with a tape measure (LOG B5).
13. **Do not stack guesses.** A fix that made things worse was the moment to
    look at the data, not to add another fix (LOG M0 servo port).
14. **Retract in writing.** The lighting explanation was recorded, disproved
    and retracted in the same history (`9f4efe8`, `4fe8cef`, `f31b8a7`).
15. **A display that reports health over a dead source is the recurring bug.**
    The anchor node, cuVSLAM's verdict, the dashboard pose (`ea112a5`).
16. **Kill the process group, not the launch** (`d3ed387`); **commit as you
    verify**, because the battery has stranded work once (`ROADMAP.md`).

---

## The explainer video: chapter outline

Each chapter has one demo clip to show, one number to put on screen, and the
source to script from. Run videos and contact sheets are in
`kaya_lab/data/b5_runs_20260928/run*_video/`; the navigation sessions have
logs but no video, so those chapters use the dashboard and RViz recordings.

| # | chapter | show | the number | script from |
|---|---|---|---|---|
| 1 | What Kaya is, and the 8 GB constraint | the robot, the SKU fuse readout | 0xD5; 0 DLA cores | SETUP_LOG §2-3 |
| 2 | Frames and the omni drive | wheel probe pulses; a pure strafe | 0.5 m sideways, -0.1 deg | `f515514`, `wheel_probe.py` |
| 3 | Why the camera's IMU | the side-by-side table | 0.19 deg/min | `41b051b` |
| 4 | VIO, and the bug that hid at rest | drift logs before and after | 32% to 0.63% | `cc4b040`, `eaaa63a` |
| 5 | The camera pipeline as a shared resource | `cam_rates.py` output, both eyes | left 24.5 Hz vs right 28.8 | `523b533`, `3aa071a` |
| 6 | Maps: TSDF, ESDF, decay, and the map that deleted itself | two meshes, 18 m² and 132 m² | 27 s, about 4 m | `1febc98` |
| 7 | Relocalisation and the mark | the 5 m wrong-hint recovery | 5.9 cm | `524a007` |
| 8 | Making Nav2 omni: five silent defaults and five costmap faults | the stalled traverse table | 1.00 m short to goal reached | `95127be`, `ROADMAP.md` |
| 9 | The false "Reached the goal!" | the Nav2 code path | 23 cm into 5 m, every time | `a820bbd` |
| 10 | The dashboard that refuses to lie | offline banner, 503 | 10 minutes of fiction | `ea112a5` |
| 11 | "Take me where I can cook" | the Ask box, the drive to the kitchen | 6 of 6; 5.1 m to 14 cm | `e6b4a3e`, `1a91641` |
| 12 | A calibrated twin in Isaac Sim | tilt test, step test, 13 checks | 15 deg real, 15.6 sim | `usd/README.md` |
| 13 | Thousands of Kayas: Isaac Lab in ten minutes | the 4096-env grid | 16k decisions/s | LOG M0 |
| 14 | Drive to a point, and the loophole | the diagonal drive replay | 0.21 m/s vs 0.15 | LOG M1 |
| 15 | Seeing the ball without rendering it | the belief disc, green and red | 83% in view, 99.9% | LOG B2 |
| 16 | The first real goal | run 1 and run 7 videos | 29 s; 14 cm not 15.3 | LOG B5 |
| 17 | Play like a footballer: walls, jams, kicks | runs 11, 24, 32 | 0 of 8 to 3 of 4 | LOG v2 to v2.4 |
| 18 | The video beats the log | run 20 contact sheet vs its log | 1.5 m logged, 0 m moved | LOG "video beats the log" |
| 19 | The method | the sixteen lessons above | | Part VI |

What the video should not claim: that relocalisation works after furniture
moves (it does not, cause unresolved), that the 0.47 s camera lag was fixed
(it was configured around), or that v2.5 was trained.
