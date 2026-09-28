# Deployable policies (numpy .npz, for robot/numpy_policy.py)

| file | what | real robot |
|---|---|---|
| policy_v24.npz | **best (2026-09-29)**: v2.4 ball-to-goal, 0.10 m/s, 31 obs (15 + 16-ray depth scan) | 3 goals in 4 (runs 35-38): beside the box, from behind, from the panel/wall corner |

Source checkpoint: `~/IsaacLab/logs/rsl_rl/kaya_ball_v2/2026-09-29_00-25-07_v24/model_1097.pt`
(task `Kaya-BallGoal-V24-v0`), exported with `scripts/export_policy_npz.py`.

On the robot:
1. Copy the file to `~/b5/node/`.
2. Start the stack with `robot/run_stack.sh start` (installed as `~/b5/run.sh`).
3. Start `robot/ball_policy_node.py` with the parameters in SIM_RL_PLAN.md
   section 2b.
