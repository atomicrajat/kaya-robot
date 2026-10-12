# Chapter 5: a twin in simulation, and thousands of them learning

*Part of the Kaya learning narrative. Index: [`../../LEARNINGS.md`](../../LEARNINGS.md).
Sources: `kaya_description/usd/README.md`, commits `a0a6420`, `781a284`, `5f5fa43`;
`kaya_lab/LEARNING_LOG.md` sections "M0", "M1", "M1 vs the vy = 0 ablation";
`SIM_RL_PLAN.md` sections 0, 1.*

## 5.1 Why build a twin at all

Reinforcement learning needs millions of attempts. A real robot gives you a
few hundred a day and breaks. A simulator gives thousands of robots at once,
but only a policy trained on a robot that behaves like the real one will
work on the real one. So the first job was not learning; it was building a
simulated Kaya and proving, with measurements, that it behaves like the one
on the floor.

## 5.2 From CAD to a USD asset

Isaac Sim describes a robot in USD (Universal Scene Description): a tree of
prims (nodes) carrying geometry, mass, joints, drives and sensors. Kaya's
asset is **generated**, never edited by hand: `build_kaya_usd.py` takes the
shapes from the Fusion 360 export and every number from the URDF, which in
turn comes from `kaya_dimensions.yaml`. The chain means the simulator cannot
drift from the robot's own description, and a measurement changed in one
place propagates everywhere.

The asset is 40 rigid bodies and 39 joints: the chassis, three wheels, and
36 free-spinning rollers (twelve per wheel). The Gemini 335L is NVIDIA's own
asset, vendored so the stage loads offline, with three corrections applied
as overrides: its depth clipped at 1 m where the real camera works from 0.17
m; its colour aperture matched to the robot's calibration; and its image
cameras were rendering their own lens barrel because the near clip was
behind the glass.

**Four things the USD cannot carry**, so a scene must supply them, and a
scene that skips any gets a different robot:

| needed | why |
|---|---|
| a stage in metres, Z-up | a reference does not rescale or rotate |
| **240 Hz physics** | at Isaac's default 60 Hz the 3 g rollers chatter |
| a floor with a friction material | the robot's grip is min(roller 0.40, floor) |
| drive through `KayaDriver` | plain velocity targets creep on slopes and drop the real servo lag |

`kaya_isaac.py` does all four and is what every later script uses.

## 5.3 Choices made by measurement, not preference

- **Rollers are spheres, not the CAD barrel hull.** The hull looks more
  exact and measured worse: 13% speed loss and rollers spinning at 110
  rad/s, against 2% loss and a physically plausible 16 rad/s with spheres.
- **Wheels are position servos fed a moving target.** The real ST3020 closes
  its loop on its encoder and holds when stopped. A velocity-only drive in
  PhysX crept at 0.08 rad/s on a slope whatever its damping; that is solver
  residual, and only position feedback cancels it. PhysX reports continuous
  joint angles wrapped into (-2π, 2π], so the driver counts the wraps
  itself, and it integrates targets on the physics step, not the render
  clock.
- **The servo is not instant.** The real one was characterised with a step
  test: a 30 Hz command loop, 27.5 ms of bus-and-firmware delay, then a
  second-order response (natural frequency 31 rad/s, damping ratio 0.60,
  which gives a 16% overshoot). The sim replays the real recording: start
  0.101 s against 0.101, stop 0.134 against 0.134, 201.8 mm against 202.1.
- **On the GPU, read poses from the physics tensors.** USD is not written
  back while GPU physics runs.
- **Friction, from a tilt test.** The real robot slides at 15 degrees on a
  table with the camera downhill. The sim reproduces that at roller μ 0.40:
  14.6 degrees on CPU, 15.6 on GPU (the GPU solver holds static friction
  about a degree longer). The calibration drifted once when roller masses
  changed after it was done, so it is re-run after any mass or collider
  change.

## 5.4 How the sim earned trust

Thirteen checks run headless in about a minute, on CPU and on GPU, and each
fails if the asset drifts: total mass 2.088 kg as PhysX loads it (a wheel
weighs 112 g, not the CAD's 58); rest height 37.0 mm, level; the simulated
IMU at rest reads (-0.160, -9.769, +0.836) m/s² against the real
(-0.161, -9.753, +0.834); forward, strafe and rotate each go the right way at
the right speed; it holds on a 10 degree slope; the stereo baseline and depth
focal length match the camera's own calibration; and the depth camera sees
the floor and a wall at the right distances with the 4.89 degree nose-up
pitch.

The IMU check is the one that caught the Chapter 1 roll-sign error: the sim
matched the real gravity vector only with the sign flipped. And the
`camera_link` offset (the lens, not the body) was found while checking depth
geometry against a known target. Two independent measurements agreeing is
how a simulator earns trust; a disagreement is how it finds real-robot bugs.

## 5.5 Isaac Lab: the vocabulary

**Isaac Sim** simulates physics and renders cameras. **Isaac Lab** sits on
top of it for learning: it runs thousands of copies of a scene in parallel on
the GPU and talks to RL libraries. One copy of the task (one Kaya, its floor,
later a ball) is an **environment**; N of them are stepped together, so every
quantity in the code is a tensor of shape (N, ...).

An **articulation** is a robot of rigid bodies connected by joints. An
**actuator** drives a joint; an *implicit* actuator is a PD controller solved
inside the physics engine. Kaya's actuator gains are set to `None` in the
Isaac Lab config on purpose, so it keeps the values already in the USD, which
were calibrated against the real servo. Re-typing them would be a second
source of truth, and they would drift.

Physics runs at 240 Hz; the policy decides at 10 Hz, the rate it will run at
on the robot. **Decimation** is the number of physics steps per policy step:
24. Between decisions the batched servo model (`kaya_lab/servos.py`, the
torch port of the numpy original) runs the real chain every physics step:
30 Hz latch, 27.5 ms delay, second-order response, position targets.

**The port was tested against the original, carefully.** Fed identical
commands with no simulator, the first run disagreed by exactly one physics
step; a "fix" made it worse. The real cause was in the *original*: it
accumulated time with `t += dt`, and floating-point rounding let its 30 Hz
tick land a step early or late now and then, while the new version ticks on
an exact grid. The right test compares with the best whole-tick alignment,
and matches to 2e-6 rad/s. When two implementations disagree, do not assume
the new one is wrong, and do not stack guesses after a fix makes things
worse: go and look at the data.

## 5.6 Speed is what makes learning possible

**Throughput** is simulated robot-steps per second. PPO needs tens to
hundreds of millions of samples, so this decides hours against days.
Measured on the RTX 5090 with the GPU otherwise idle:

| solver iterations | N = 64 | N = 1024 | N = 4096 |
|---|---|---|---|
| 32 (the asset's) | 270 | 4.1k | 7.4k decisions/s |
| 8 | 535 | 7.9k | 16.2k |

Two findings. **A second Isaac Sim instance on the GPU halved throughput**
(a leftover GUI); close everything before measuring or training. And the
asset's 32 solver iterations exist only to hold on a 10 degree slope; on a
flat floor, 8 gives the same driving accuracy (98-100% of command) at 2.2x
the speed. Different settings for different jobs: 8 for training, 32 for
validation. Going from 64 to 1024 robots costs almost nothing, because each
step is dominated by fixed launch overhead rather than robot count. That is
why RL uses thousands of parallel environments: 16k samples per second is
about 100 million in under two hours.

## 5.7 The RL loop, and the first task

The **policy** is a small neural network. Ten times a second it reads an
**observation** (numbers describing the situation) and outputs an
**action**. The environment applies the action for 0.1 s of physics and
returns a **reward**. An **episode** is one attempt, from reset to success,
failure or timeout. **PPO** alternates two phases: collect experience from
every environment (the rollout), then nudge the network toward actions that
did better than expected. "Better than expected" is judged by a second
network, the **critic**, which learns to predict future reward; the
difference between what happened and its prediction is the **advantage**.
The "proximal" part limits how far each update may move the policy.

**Task 1: drive to a point, then stop.** Every design choice has a reason:

| piece | what | why |
|---|---|---|
| observation (8) | goal in the robot's frame / 3 m; body velocity / limits; last action | only things the real robot knows from odometry, scaled near [-1, 1] |
| action (3) | vx, vy, ω in [-1, 1] times (0.15, 0.15, 1.0) | exactly `/cmd_vel`, so the policy deploys unchanged |
| progress reward | distance closed per step, normalised | a dense signal every step, not only at the end |
| success | within 5 cm **and** below 2 cm/s: +50 | the speed test makes it stop rather than drive through |
| fail | more than 5 m away: -10 | stops wandering forever |
| time, action rate | -0.05 per step; -0.05 x squared change | dawdling costs; jerky commands are hard on real servos |
| randomisation | servo delay ±30%, response ±20%, per episode | the real robot is slightly different from the model |

PPO settings worth knowing: actor and critic are two hidden layers of 64;
observation normalisation keeps inputs scaled; 24 steps per environment per
iteration; exploration noise starts at 0.5 and shrinks as the policy
commits; gamma 0.99 means rewards 10 s ahead still count; the learning rate
adapts to a target KL divergence of 0.01.

**How to read the log:** mean reward rising; `success_rate` (target above
0.95); `final_distance_m`; each reward term separately, which is how a
reward is debugged; and `Mean action std` falling as the policy commits.
Isaac Lab 3.0 is headless by default (the first "GUI" run trained with no
window), and rendering is what costs: 256 robots with the window took 31 s
per iteration, 4096 headless take 7 s. So: watch a few iterations in the GUI,
train headless, replay in the GUI. The floor is sized from the environment
grid (4096 at 4 m spacing span 256 m, the default floor is 100 m), and a
"fell off the world" metric exists so that cannot go unnoticed.

**Result:** 99.8% success by iteration 25, about three minutes; 100% from
50; by 200, final distance 1.5 cm and 8.9 s per goal.

## 5.8 The loophole, and the ablation

The episodes were shorter than the speed limit allowed. The policy had
discovered that vx and vy are clipped separately, so full speed on both is
0.21 m/s diagonally, 41% faster than straight. The real drive node clips the
same way, so this is a genuine advantage of the omni base, not a sim bug. But
it is the general lesson in miniature: **RL finds any loophole in how the
task is defined. When a number looks too good, find out why before
celebrating.**

The same task with sideways velocity forced to zero is the **ablation**, the
measurement of what holonomy buys:

| iteration | omni | differential (vy = 0) |
|---|---|---|
| 25, success | 99.8% | 62% |
| 200, success | 100% | 100% |
| 200, time per goal | 8.9 s | 12.6 s |

Both reach every goal. Omni learns faster and arrives about 30% sooner: it
does not turn first, and it has the diagonal. On an easy task the gap is
speed; the ball task is where it was expected to widen.
