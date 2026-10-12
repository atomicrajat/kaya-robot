# kaya_lab: training Kaya with reinforcement learning in Isaac Lab

An Isaac Lab 3.0 extension that trains the Kaya robot with RSL-RL PPO, and the
code that runs the trained policy on the real robot. Two tasks exist:

| task | what the policy learns | result |
|---|---|---|
| `Kaya-PointGoal-v0` | drive to a random point on an empty floor and stop | 100% from iteration 50; 1.5 cm final error |
| `Kaya-BallGoal-*` | find a yellow ball with the camera, dribble it, push it into a cardboard box | sim 99.9%; **real robot: 3 goals in 4 with v2.4** (`policies/policy_v24.npz`) |

Each task also has a `-DiffDrive-v0` variant with sideways velocity forced to
zero, so what the omni wheels buy is measured rather than asserted.

The story of how these were built, every mistake included, is in
[`LEARNING_LOG.md`](LEARNING_LOG.md) (chronological) and
[`../LEARNINGS.md`](../LEARNINGS.md) (by topic). The plan and every recorded
result is [`../SIM_RL_PLAN.md`](../SIM_RL_PLAN.md).

---

## What you need

| | this project used |
|---|---|
| Isaac Sim | **6.0.1** at `~/isaac-sim` (`$ISAAC_SIM`) |
| Isaac Lab | **3.0.0**, branch `release/3.0.0-beta2`, at `~/IsaacLab` (`$ISAACLAB`), linked to that Isaac Sim, with RSL-RL installed |
| GPU | RTX 5090 Laptop, 24 GB. 4096 environments fit; fewer work, slower |
| the sim robot | `../kaya_description/usd/kaya.usda`, calibrated against the real robot. **Run its checks first:** `$ISAAC_SIM/python.sh ../kaya_description/usd/kaya_sim.py --test --device cuda` (13/13) |

Isaac Lab 3.0 is a beta on Isaac Sim 6.0's experimental APIs. Pin these
versions; the environment code uses `isaaclab_physx.physics.PhysxCfg` and the
`--external_callback` task registration, which are 3.0 features.

Nothing here is pip-installed. `scripts/rl.sh` puts `kaya_lab/` on
`PYTHONPATH` and registers the tasks through Isaac Lab's
`--external_callback kaya_lab.tasks.register`.

## Layout

    kaya_lab/
      kaya_lab/                 the extension (importable as `kaya_lab`)
        assets.py               Kaya as an ArticulationCfg: points at kaya.usda, keeps its drive gains
        servos.py               the real servo chain (30 Hz latch, 27.5 ms delay, 2nd-order response), batched in torch
        ball.py                 the ball: 14 cm, 112 g, measured squash and grip; the goal box
        detector.py             the training-time stand-in for the camera detector (FOV, cut-off, noise, dropouts, latency)
        ball_detector.py        the REAL detector (HSV -> blob -> sphere fit), shared by the B4 check and the robot
        tasks/point_goal/       M1: drive to a point
        tasks/ball_goal/        B1 teacher, B2 camera-only, v2 "footballer" rooms, and the v2.1-v2.5 configs
      scripts/                  train/play wrapper, throughput and fidelity checks, policy export
      robot/                    what runs ON the robot: ball_policy_node.py and its modules; probes/ for one-off measurements
      policies/                 exported policies as numpy .npz (the robot has no PyTorch)
      data/                     real camera frames, the printed ArUco markers, and every real run's log and video contact sheet

The ball-task configs for v2 to v2.5 are deliberately one file
(`tasks/ball_goal/ball_goal_env_cfg.py`): each version is a small subclass
whose docstring names the real run that motivated it. Read them in order and
you have the history of the sim-to-real work. The environment code
(`ball_goal_v2_env.py`) has accreted reward terms the same way; each is
switched on by a config value that is zero in the version before it.

## 1. Check the simulator is fast enough (M0)

    cd ~/IsaacLab
    ./isaaclab.sh -p <repo>/kaya_lab/scripts/m0_batched.py --num_envs 1024 --headless

Runs the robot's checks (mass, rest height, forward / strafe / rotate through
the servo chain) on every copy at once and prints the physics throughput.
Expect about 16k policy decisions per second at 4096 environments with 8
solver iterations on the 5090. **Close every other Isaac Sim first:** a second
instance on the GPU halved throughput.

## 2. Train the first task: drive to a point (M1)

    kaya_lab/scripts/rl.sh train --task Kaya-PointGoal-v0 --num_envs 256 --viz kit      # watch a few iterations
    kaya_lab/scripts/rl.sh train --task Kaya-PointGoal-v0 --num_envs 4096 --max_iterations 200   # headless, ~25 min
    kaya_lab/scripts/rl.sh play  --task Kaya-PointGoal-v0 --num_envs 32 --viz kit \
        --checkpoint ~/IsaacLab/logs/rsl_rl/kaya_point_goal/<run>/model_200.pt

Isaac Lab 3.0 is **headless by default**; `--viz kit` opens the window.
Rendering is what costs: 256 robots with the window took 31 s per iteration,
4096 headless take 7 s. Logs and checkpoints land in
`~/IsaacLab/logs/rsl_rl/<experiment_name>/<date>[_<run_name>]/`. `play` also
writes `exported/policy.onnx` and `policy.pt`.

What to watch in the log: `Metrics/success_rate` (target > 0.95),
`Metrics/final_distance_m`, `Mean action std` falling as the policy commits,
and each `Episode_Reward/*` term separately, which is how a reward is debugged.

Then the ablation, same command with `--task Kaya-PointGoal-DiffDrive-v0`.

## 3. Train the ball task

The chain that produced the deployed policy, each step fine-tuned from the
one before with RSL-RL's resume flags
(`--resume --load_run <run folder> --checkpoint model_N.pt`):

| step | task | what changed | sim result |
|---|---|---|---|
| B1 teacher | `Kaya-BallGoal-v0` | true ball state, curriculum from "ball at the mouth" to "anywhere" | 99.9% at level 1, 300 it., ~75 min |
| B2 student | `Kaya-BallGoal-Det-v0` | sees only the simulated detector + odometry memory; asymmetric critic | 99.9%, 14.3 s per goal |
| B2 14 cm | same, `--resume` | the real ball measured 14.0 cm, not 15.3 | 100%, 50 it. |
| v2 | `Kaya-BallGoal-V2-v0` | rooms with walls, corners, obstacles; a 16-ray depth scan | 97.5% |
| v2.1 | `-V21-v0` | jam penalty; starts beside the box's side panels | 99% |
| v2.2 | `-V22-v0` | penalty for pushing a centred ball into a surface; 0.10 m/s | |
| v2.3 | `-V23-v0` | escalating jam penalty; a kick bonus for back-off-then-strike | |
| **v2.4** | `-V24-v0` | measured ball (2,200 N/m, grip 1.0); depth blind < 0.3 m; starts beside the box; post penalty | beside the box 89%, corner 83% |
| v2.5 | `-V25-v0` | goal panels count as bumps; robot-in-goal penalty | written, smoke-tested, **not trained** |

A B1 run: `rl.sh train --task Kaya-BallGoal-v0 --num_envs 4096 --max_iterations 300`.
A fine-tune: `rl.sh train --task Kaya-BallGoal-V24-v0 --num_envs 4096 --max_iterations 300 --resume --load_run 2026-09-28_22-20-52_v23 --checkpoint model_798.pt --run_name v24`.

Replaying a ball policy needs the curriculum told where to start, because the
level lives in the environment, not the policy:
`play ... env.initial_level=1.0 "env.goal_width_range=[0.46,0.46]"`.

Evaluation scripts, all run with `PYTHONPATH=$PWD/kaya_lab ~/IsaacLab/isaaclab.sh -p ...`
from the repo root (each file's docstring has the full command):

| script | what it checks |
|---|---|
| `b0_ball_contact_probe.py` | where the sim ball touches the sim Kaya, against the real camera frames |
| `b1_scripted_check.py` | a scripted "drive forward" can score at all (the success test is right) |
| `b4_eval.py` | the B2 policy on **rendered** camera frames with the real image detector: 90.6% |
| `v2_corner_eval.py` | v2.x policies from the hardest start (ball in the corner beside the box), 512 episodes |
| `test_servos_match.py`, `test_detector_vs_real.py`, `test_ball_detector_real_frames.py` | the batched servo chain against the numpy original (2e-6 rad/s); both detectors against the real frames |

## 4. Put a policy on the robot

**Export.** The robot has no PyTorch or ONNX runtime; the network
(31 -> 256 -> 128 -> 64 -> 3) runs in numpy. After one `play` of the run:

    $ISAAC_SIM/python.sh kaya_lab/scripts/export_policy_npz.py ~/IsaacLab/logs/rsl_rl/kaya_ball_v2/<run> model_1097.pt
    cp ~/IsaacLab/logs/rsl_rl/kaya_ball_v2/<run>/exported/policy.npz kaya_lab/policies/policy_vNN.npz

The export checks numpy against Isaac Lab's TorchScript export (matches to
1e-5) and commits the policy to the repo, so the robot gets it with `git pull`.

**The YOLO engine, once per robot.** The ball detector finds the ball with a
pretrained YOLO11s ("sports ball", COCO class 32) on the Jetson's GPU, then
measures it with the sphere fit. Export the ONNX on the workstation (ultralytics
8.4, in a venv) and build the TensorRT engine on the robot; engines are
machine-specific and not committed:

    pip install ultralytics && yolo export model=yolo11s.pt format=onnx imgsz=640   # workstation
    scp yolo11s.onnx <robot>:~/ros2_ws/src/kaya/kaya_lab/robot/
    /usr/src/tensorrt/bin/trtexec --onnx=yolo11s.onnx --saveEngine=yolo11s_fp16.engine --fp16   # robot, ~7 min

**The arena.** A 46 cm wide, 30 cm tall, 30 cm deep open-fronted cardboard box
on its side, with the three printed ArUco markers from `data/aruco/` (150 mm,
DICT_4X4_50: ID 0 inside the back panel, 1 and 2 on the outside of the side
panels; measure the printed size, this print came out at 146 mm). A 14 cm,
~110 g soft yellow ball. Clear floor around it, about 2.5 m each way.

**Run it.** On the robot, from the repo checkout (nothing is copied):

    cd ~/ros2_ws/src/kaya/kaya_lab/robot
    ./run_stack.sh start                       # camera + cuVSLAM + drive, projector off
    # wait ~30 s, put the robot on its start mark, keep it still for 2 s (gyro bias)
    python3 ball_policy_node.py --ros-args \
        -p policy:=../policies/policy_v24.npz -p max_lin_vel:=0.10 \
        -p goal_source:=marker -p goal_x:=1.57 -p arena_side:=2.5 -p arena_back:=2.5 \
        -p odom_source:=fused -p use_vio:=false -p use_scan:=true \
        -p yolo_engine:=yolo11s_fp16.engine -p dry_run:=false
    ./run_stack.sh stop

`dry_run:=true` (the default) computes and logs everything without moving.
`goal_x` is the box mouth's distance ahead of the start mark; with
`goal_source:=marker` it is only the guess until the first marker sighting.
`max_lin_vel` must be the speed the policy was **trained** with (v2.2 onward:
0.10); scaling actions on the robot instead breaks what the policy learned about
its own motion. Every rejected detection is saved as `reject_*.png` beside the
node; the logs of all 38 real runs are in `data/b5_runs_20260928/`.

**Film every run.** The video overruled the log more than once
(`LEARNING_LOG.md`, "The video beats the log").

## The two ideas worth taking away

1. **The policy never sees an image.** Training computes what a detector would
   output (bearing, range, visible or not, with noise and dropouts); the robot
   runs a real detector that outputs the same numbers. The detector can be
   swapped, as it was from colour to YOLO, without retraining.
2. **Everything the robot feeds the policy is a claim.** In sim the inputs are
   true by construction. On the robot each came from a sensor plus code, and
   each lied at least once: the depth scan, the ball detector, the heading, a
   filter. Most of `ball_policy_node.py` exists to check those claims, and
   every guard's comment names the run that needed it.
