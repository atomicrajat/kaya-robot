"""The real ball and the real goal, as Isaac Lab assets.

Ball: 14.0 cm (re-measured by the user 2026-09-28, circumference 44 cm; the 15.3 cm used until then
was wrong and made every real-frame fit disagree), 112 g, air-filled rubber, matte yellow
with black stripes. Grips the tile and rolls easily.

Goal (the user's build, 2026-09-28): an open-fronted cardboard box lying on
its side, mouth 46 cm wide x 30 cm tall, 30 cm deep, cardboard on all
inner faces (back, sides, top). It replaced a 120 x 60 x 12 cm frame: the ball
(14 cm) fits completely inside 30 cm, so "scored" is strict - the WHOLE
ball behind the mouth, between the side panels. It stands free (no wall
needed); a wall behind it is kept in sim as the room's wall.
"""

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg

BALL_RADIUS = 0.140 / 2
BALL_MASS = 0.112          # weighed by the user (112 g)
# thin shell: I = 2/3 m r^2 (a solid ball would be 2/5 m r^2 - it would roll
# off the robot's front differently)
BALL_INERTIA = 2.0 / 3.0 * BALL_MASS * BALL_RADIUS**2
BALL_FRICTION = (0.8, 0.7)      # rubber on tile: grips (user: "does not slip")
BALL_RESTITUTION = 0.3          # air-filled rubber, low bounce; not measured yet
# Rolling resistance as a constant decelerating force c * m * g against the
# ball's floor velocity. PhysX has no rolling resistance of its own. c is
# randomised until the roll-out distance on the tiles is measured.
BALL_ROLL_C_RANGE = (0.01, 0.05)

GOAL_WIDTH = 0.46
GOAL_HEIGHT = 0.30
GOAL_DEPTH = 0.30
CARDBOARD_T = 0.01              # panel thickness (m)
WALL_LEN = 4.0                  # the wall behind the goal, per env

BALL_CFG = RigidObjectCfg(
    prim_path="/World/envs/env_.*/Ball",
    spawn=sim_utils.SphereCfg(
        radius=BALL_RADIUS,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            linear_damping=0.0, angular_damping=0.0, max_depenetration_velocity=1.0),
        mass_props=sim_utils.MassPropertiesCfg(mass=BALL_MASS),
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.004, rest_offset=0.0),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=BALL_FRICTION[0], dynamic_friction=BALL_FRICTION[1],
            restitution=BALL_RESTITUTION),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.85, 0.05), roughness=0.9),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, BALL_RADIUS)),
)


def set_hollow_inertia(stage, ball_prim_path: str):
    """Override the solid-sphere inertia PhysX would compute from the mass."""
    from pxr import Gf, UsdPhysics

    prim = stage.GetPrimAtPath(ball_prim_path)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(BALL_INERTIA, BALL_INERTIA, BALL_INERTIA))


GOAL_WIDTH_RANGE = (0.40, 0.70)  # randomised in training so the policy aims; the real box is 0.46


def _cardboard():
    return sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.38, 0.22), roughness=1.0)


CARDBOARD_MAT = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4, restitution=0.2)


def post_cfg(name: str) -> RigidObjectCfg:
    """A goal side panel (30 cm deep, 30 cm tall). Kinematic: the env moves it
    at reset to set that episode's goal width, and nothing can push it."""
    return RigidObjectCfg(
        prim_path=f"/World/envs/env_.*/{name}",
        spawn=sim_utils.CuboidCfg(
            size=(GOAL_DEPTH, CARDBOARD_T, GOAL_HEIGHT),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=CARDBOARD_MAT,
            visual_material=_cardboard(),
        ),
    )


def spawn_goal(env_path: str, goal_line_x: float):
    """The fixed parts of the goal in one env (call on env_0 before cloning):
    the wall, the box's back panel and its top panel (spanning the widest
    randomised mouth; Kaya is ~22 cm tall, so it can nose in under the 30 cm
    top). The side panels are kinematic
    bodies (post_cfg) placed per episode. Env frame: the goal faces -x, mouth
    centred on y = 0, goal line at x = goal_line_x, the back panel's inner face
    at goal_line_x + GOAL_DEPTH."""
    back_x = goal_line_x + GOAL_DEPTH
    grey = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.75, 0.75, 0.72), roughness=1.0)
    wall_mat = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4, restitution=0.3)

    def box(name, size, pos, vis, mat, collide=True):
        cfg = sim_utils.CuboidCfg(size=size, visual_material=vis,
                                  collision_props=sim_utils.CollisionPropertiesCfg() if collide else None,
                                  physics_material=mat if collide else None)
        cfg.func(f"{env_path}/{name}", cfg, translation=pos)

    box("Wall", (0.05, WALL_LEN, 0.5), (back_x + CARDBOARD_T + 0.025, 0.0, 0.25), grey, wall_mat)
    span = GOAL_WIDTH_RANGE[1] + 2 * CARDBOARD_T
    box("GoalBack", (CARDBOARD_T, span, GOAL_HEIGHT),
        (back_x + CARDBOARD_T / 2, 0.0, GOAL_HEIGHT / 2), _cardboard(), CARDBOARD_MAT)
    box("GoalTop", (GOAL_DEPTH, span, CARDBOARD_T),
        (goal_line_x + GOAL_DEPTH / 2, 0.0, GOAL_HEIGHT + CARDBOARD_T / 2), _cardboard(), CARDBOARD_MAT)


def rolling_resistance(vel_w: torch.Tensor, c: torch.Tensor) -> torch.Tensor:
    """Force [N], (N, 3), opposing the ball's horizontal velocity: c * m * g,
    faded in below 1 cm/s so a resting ball is not pushed around."""
    v = vel_w[:, :2]
    speed = torch.linalg.norm(v, dim=1, keepdim=True)
    f = -c[:, None] * BALL_MASS * 9.81 * v / speed.clamp(min=1e-6) * (speed / 0.01).clamp(max=1.0)
    return torch.cat([f, torch.zeros_like(f[:, :1])], dim=1)


def scored(ball_pos_env: torch.Tensor, goal_line_x: float, half_width: torch.Tensor) -> torch.Tensor:
    """The whole ball inside the box: behind the mouth by a radius, between the sides."""
    return (ball_pos_env[:, 0] > goal_line_x + BALL_RADIUS) & (ball_pos_env[:, 1].abs() < half_width)

