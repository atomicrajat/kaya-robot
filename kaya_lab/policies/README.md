# Deployable policies (numpy .npz, for robot/numpy_policy.py)

| file | what | real robot |
|---|---|---|
| `policy_v24.npz` | **best (2026-09-29)**: v2.4 ball-to-goal, trained at 0.10 m/s, 31 observations (15 + the 16-ray depth scan) | 3 goals in 4 (runs 35-38): beside the box, from behind, from the panel/wall corner |

Source checkpoint: `~/IsaacLab/logs/rsl_rl/kaya_ball_v2/2026-09-29_00-25-07_v24/model_1097.pt`
(task `Kaya-BallGoal-V24-v0`), exported with `scripts/export_policy_npz.py`,
which checks the numpy network against Isaac Lab's TorchScript export (1e-5).

Running it on the robot: `kaya_lab/README.md`, section 4. The policy file is
referenced by path from the node (`-p policy:=../policies/policy_v24.npz` when
run from `kaya_lab/robot/`), so a `git pull` on the robot is the whole deploy.
