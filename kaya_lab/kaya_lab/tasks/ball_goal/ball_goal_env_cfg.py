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


# v2 "footballer" (ball_goal_v2_env.py). The config lives here, not with the env:
# Isaac Lab imports a task's config module BEFORE the simulator starts, and the env
# module imports USD, which then crashes SimulationApp's startup (seen 2026-09-28).
@configclass
class KayaBallGoalV2EnvCfg(KayaBallGoalDetEnvCfg):
    episode_length_s = 60.0
    # the room walls and obstacles add contacts: the first reset at 4096 envs asked PhysX for
    # ~1.08M patches (v2 - v2.2 logs), so the 2**18 buffer above dropped contacts on that reset
    sim: SimulationCfg = SimulationCfg(dt=1.0 / PHYSICS_HZ, render_interval=8,
                                       physics=PhysxCfg(gpu_max_rigid_patch_count=2**21))
    # B2's 15 + the 16-ray depth scan
    observation_space = 31
    # + true ball position and velocity for the critic
    state_space = 35
    debug_vis = False

    def __post_init__(self):
        super().__post_init__()
        self.scene.env_spacing = 8.0

    # safety bounds only: the walls keep everything in
    arena_x = (-4.0, 3.0)
    arena_y = (-3.5, 3.5)

    # the room (env frame; the goal wall is fixed at the +x end)
    room_back_x = (-3.0, -1.5)            # back wall's inner face
    room_side_y = (1.2, 1.9)              # each side wall's inner face, |y|, drawn independently
    # obstacle slots, each a cube of its own size (rooms are clones of one USD, so a size cannot
    # vary per room; random presence mixes them). The chance each is present rises with the
    # curriculum level: ~1 per room at level 0, ~3 at level 1 (user, 2026-09-28: "more obstacles")
    obstacle_sizes = (0.15, 0.22, 0.28, 0.35)
    n_obstacles = 4
    obstacle_prob = (0.2, 0.7)
    obstacle_height = 0.30

    # curriculum extras: fraction of starts with the ball against a wall or in a corner, level 0..1
    wall_ball_frac = (0.0, 0.4)
    ball_goal_dist = (0.35, 3.0)

    # the depth scan
    n_rays = 16
    ray_fov_deg = 92.0
    ray_max = 3.0
    ray_noise = 0.02                      # fraction of range

    # extra rewards
    rew_ball_speed = -0.3                 # per m/s above ball_speed_free
    ball_speed_free = 0.3
    # contact, not proximity: recovering a ball from a wall NEEDS the robot close to the wall.
    # (The first version used an 18 cm radius and 3 cm margin: robots 2-3 cm away counted.)
    rew_bump = -0.05                      # per step with the robot within bump_dist of a wall/obstacle
    bump_dist = 0.01
    # v2.1 knobs (0 = off in v2)
    rew_jam = 0.0                         # per step pushing a ball that is jammed against something
    jam_ball_speed = 0.03                 # [m/s] "not moving"
    side_panel_frac = 0.0                 # of wall starts: ball against the OUTSIDE of a box side panel
    side_panel_x = (0.05, 0.25)           # [m] behind the mouth; ~0.23 = in the corner with the goal wall
    rew_center_push = 0.0                 # per step pushing a CENTRED ball that is right next to a surface
    center_push_half_y = 0.03             # [m] "centred in the pocket"
    center_push_clear = 0.05              # [m] ball surface to wall / obstacle / box panel
    jam_escalate = 0.0                    # jam penalty x (1 + jam_escalate * consecutive jammed steps)
    rew_kick = 0.0                        # once per jam: back off, then strike it off-centre so it moves
    kick_speed = 0.10                     # [m/s] ball speed that counts as freed
    kick_min_y = 0.05                     # [m] ball centre this far to the side = a front-wheel strike
    kick_window = 20                      # [policy steps] 2 s after the jam ends to back off and strike


@configclass
class KayaBallGoalV21EnvCfg(KayaBallGoalV2EnvCfg):
    """v2.1: a penalty for pushing a jammed ball, and starts beside the box's side panels
    (the real runs 12b/12c: the robot freed a corner ball, then kept pushing it into the
    outside of a side panel). The ball physics is v2's: a scripted push test showed the
    rigid sim ball ALREADY jams against a panel (stopped 90% of the pushing time; a
    squashier, grippier ball jammed less, 72%), so the gap was the missing situation and
    the missing reason to stop pushing, not the physics."""

    rew_jam = -0.2
    side_panel_frac = 0.3


@configclass
class KayaBallGoalV22EnvCfg(KayaBallGoalV21EnvCfg):
    """v2.2 (the user's observation on the real robot, 2026-09-28): the ball jams when Kaya
    pushes it square-on, centred between the front wheels, into a wall or the box's side:
    the front wheels trap it sideways. A footballer strikes it off-centre so it squirts out
    along the wall. Penalise pushing a centred ball that is right next to a surface;
    off-centre (front-wheel) strikes are free. At the gentler 0.10 m/s."""

    rew_center_push = -0.1
    max_lin_vel = 0.10


@configclass
class KayaBallGoalV23EnvCfg(KayaBallGoalV22EnvCfg):
    """v2.3 (the user, watching the real robot pin a ball against the box's side panel, 2026-09-28:
    "the robot should learn to slightly go back, relax and then hit, not push all the time - it has
    to kick and then take the ball"). Holding a jammed ball gets steadily more expensive (the
    penalty grows with every consecutive jammed step), and freeing it earns a bonus - but only the
    way the user described it: back off first, then strike the ball off-centre with a front wheel
    so it moves off, then come back and play the next touch."""

    jam_escalate = 0.15
    rew_kick = 1.0
