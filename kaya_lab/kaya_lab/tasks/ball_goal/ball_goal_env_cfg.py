"""B1: push the ball into the cardboard goal (teacher: true ball state).

Env frame: the goal is on a wall at the +x end; its goal line is at
x = GOAL_LINE_X, mouth centred on y = 0. Ball and robot start in front of it.
"""

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab.utils.configclass import configclass

from kaya_lab.assets import KAYA_CFG, PHYSICS_HZ
from kaya_lab.ball import BALL_CFG, BALL_ROLL_C_RANGE, GOAL_WIDTH_RANGE

from ..point_goal.point_goal_env_cfg import TRAIN_SOLVER_POS_ITERS


@configclass
class KayaBallGoalEnvCfg(DirectRLEnvCfg):
    decimation = PHYSICS_HZ // 10            # policy at 10 Hz
    episode_length_s = 45.0
    # 4096 robots + balls landing at the first reset overflowed PhysX's default GPU contact
    # patch buffer (163,840; asked for 179,927) - only before iteration 0, but contacts are
    # dropped when it happens, so give it room.
    sim: SimulationCfg = SimulationCfg(dt=1.0 / PHYSICS_HZ, render_interval=8,
                                       physics=PhysxCfg(gpu_max_rigid_patch_count=2**18))

    action_space = 3
    # ball pos (2), ball vel (2), goal point (2), goal direction (2), goal half-width (1),
    # body vel (3), last action (3)
    observation_space = 15
    state_space = 0

    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=5.0, replicate_physics=True)
    robot: ArticulationCfg = KAYA_CFG.replace(
        prim_path="/World/envs/env_.*/Kaya",
        spawn=KAYA_CFG.spawn.replace(articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=TRAIN_SOLVER_POS_ITERS)),
    )
    ball: RigidObjectCfg = BALL_CFG.replace(prim_path="/World/envs/env_.*/Ball")
    viewer: ViewerCfg = ViewerCfg(eye=(-3.0, -6.0, 5.0), lookat=(1.0, 0.0, 0.0))

    max_lin_vel = 0.15
    max_ang_vel = 1.0
    holonomic = True

    # --- arena (env frame, m) ---
    goal_line_x = 2.0
    arena_x = (-2.0, 2.45)                   # leaving this ends the episode; the box interior (to 2.30) is inside
    arena_y = (-2.0, 2.0)

    # --- curriculum: level 0 = ball near the goal, robot right behind it ---
    ball_goal_dist = (0.35, 2.3)              # ball distance from the goal line at level 0 .. 1 (upper bound)
    ball_half_width = (0.25, 1.6)             # |y| range of the ball
    robot_ball_dist = (0.30, 1.5)            # robot from ball (centre to centre), upper bound grows
    robot_side_angle_deg = (15.0, 180.0)     # robot position around the ball, off the ball->goal line
    robot_heading_noise_deg = (15.0, 180.0)
    curriculum_up = 0.75                     # success rate that raises the level
    curriculum_down = 0.30
    curriculum_step = 0.05
    initial_level = 0.0                      # replay a trained policy with env.initial_level=1.0

    # --- goal width: randomised around the real 46 cm box so the policy aims ---
    goal_width_range = GOAL_WIDTH_RANGE

    # --- ball physics randomisation ---
    ball_roll_c = BALL_ROLL_C_RANGE
    randomize_servos = True

    # --- rewards ---
    rew_approach = 0.3                       # robot closing on the ball (full speed = 1 per step)
    rew_ball_progress = 1.0                  # ball closing on the goal mouth
    rew_possession = 0.05                    # per step with the ball in the front pocket
    rew_success = 100.0
    rew_fail = -20.0
    rew_time = -0.02
    rew_action_rate = -0.05

    # front pocket, in the robot's base frame (from b0_ball_contact_probe: the
    # ball centre touches the flat front (the battery) at x = 0.0825 + r = 0.1525 for the
    # 14 cm ball; the wheels funnel within |y| < 0.12)
    pocket_x = (0.12, 0.21)
    pocket_half_y = 0.12
    touch_dist = 0.153                       # centre-to-centre distance at contact (front)


@configclass
class KayaBallGoalDetEnvCfg(KayaBallGoalEnvCfg):
    """B2: the policy sees the simulated camera detector (kaya_lab/detector.py)
    instead of the true ball; the critic still gets the truth."""
    # remembered ball (2), age (1), seen now (1), goal point (2), goal direction (2),
    # goal half-width (1), body vel (3), last action (3)
    observation_space = 15
    # critic: the policy's 15 + true ball position (2) and velocity (2)
    state_space = 19
    debug_vis = True                         # show where the policy believes the ball is
