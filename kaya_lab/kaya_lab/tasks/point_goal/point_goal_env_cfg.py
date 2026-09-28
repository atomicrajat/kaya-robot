"""M1: drive Kaya to a random point on an empty floor, then stop.

Everything the policy sees is something the real robot knows from odometry,
and its action is exactly a /cmd_vel twist, so a trained policy deploys
to the robot as a /cmd_vel publisher.
"""

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils.configclass import configclass

from kaya_lab.assets import KAYA_CFG, PHYSICS_HZ

# 8 solver position iterations: same driving accuracy as the asset's 32 on a
# flat floor, 2.2x the throughput (M0, SIM_RL_PLAN.md). The 32 exist only to
# hold still on a 10 deg slope.
TRAIN_SOLVER_POS_ITERS = 8


@configclass
class KayaPointGoalEnvCfg(DirectRLEnvCfg):
    # --- timing: physics 240 Hz, policy 10 Hz (like a /cmd_vel publisher) ---
    decimation = PHYSICS_HZ // 10            # 24 physics steps per policy step
    episode_length_s = 30.0                  # 3 m at 0.15 m/s is 20 s
    sim: SimulationCfg = SimulationCfg(dt=1.0 / PHYSICS_HZ, render_interval=8)   # render at 30 Hz

    # --- spaces ---
    action_space = 3                         # vx, vy, w in [-1, 1], scaled below
    observation_space = 8                    # goal (2), body velocity (3), last action (3)
    state_space = 0

    # --- scene ---
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)
    robot: ArticulationCfg = KAYA_CFG.replace(
        prim_path="/World/envs/env_.*/Kaya",
        spawn=KAYA_CFG.spawn.replace(
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                solver_position_iteration_count=TRAIN_SOLVER_POS_ITERS
            )
        ),
    )
    viewer: ViewerCfg = ViewerCfg(eye=(2.0, -9.0, 6.5), lookat=(2.0, 2.0, 0.0))   # the middle of the grid
    debug_vis = True

    # --- action: the drive node's limits ---
    max_lin_vel = 0.15                       # m/s
    max_ang_vel = 1.0                        # rad/s
    holonomic = True                         # False: vy forced to 0 (the differential-drive ablation)

    # --- task ---
    goal_radius_range = (0.5, 3.0)           # m from the start
    success_dist = 0.05                      # m
    success_speed = 0.02                     # m/s: must have stopped, not driven through
    fail_dist = 5.0                          # m from the goal: wandered off
    randomize_servos = True                  # delay +-30%, wn/zeta +-20% (SIM_RL_PLAN D4)

    # --- reward scales ---
    rew_progress = 1.0                       # per step, 1.0 = closing at full speed
    rew_success = 50.0
    rew_fail = -10.0
    rew_time = -0.05                         # per step
    rew_action_rate = -0.05                  # * |a_t - a_{t-1}|^2
