"""Kaya as an Isaac Lab articulation.

The physics values (masses, drive gains, roller friction, solver iterations)
all live in `kaya_description/usd/kaya.usda`, measured against the real robot.
This config only points at that file. Actuator stiffness/damping/limits are
`None` on purpose: Isaac Lab then keeps the USD's values instead of
overwriting them with a second, drifting copy.
"""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

REPO = Path(__file__).resolve().parents[2]
KAYA_USD = REPO / "kaya_description" / "usd" / "kaya.usda"

PHYSICS_HZ = 240            # the ~3 g rollers chatter at 60 Hz (see the USD README)
FLOOR_MU = (0.40, 0.32)     # static, dynamic: the home tile is <= the table's 0.40
WHEEL_JOINTS = ["wheel_1_wheel_joint", "wheel_2_wheel_joint", "wheel_3_wheel_joint"]
MASS_KG = 2.088             # weighed

KAYA_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(usd_path=str(KAYA_USD)),
    # the asset's origin is base_footprint (the floor); 2 mm up avoids a contact kick at start
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.002)),
    actuators={
        # position servos fed a moving target by KayaServos (servos.py)
        "wheels": ImplicitActuatorCfg(joint_names_expr=["wheel_[1-3]_wheel_joint"], stiffness=None, damping=None),
        # free-spinning rollers: bearing damping only, from the USD
        "rollers": ImplicitActuatorCfg(joint_names_expr=["wheel_[1-3]_roller_.*"], stiffness=None, damping=None),
    },
)


def floor_cfg(mu=FLOOR_MU, size=(100.0, 100.0)) -> sim_utils.GroundPlaneCfg:
    """Ground with a friction material. Rollers combine friction with "min",
    so the floor only matters when it is slipperier than the rollers' 0.40.
    size must cover every env: 4096 envs at 4 m spacing span ~256 m."""
    return sim_utils.GroundPlaneCfg(
        size=size,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=mu[0], dynamic_friction=mu[1], restitution=0.0
        )
    )
