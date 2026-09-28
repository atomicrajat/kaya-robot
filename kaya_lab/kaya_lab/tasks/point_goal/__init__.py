"""M1: drive to a point. Holonomic, and the vy = 0 ablation."""

import gymnasium as gym

from isaaclab.utils.configclass import configclass

from .point_goal_env_cfg import KayaPointGoalEnvCfg


@configclass
class KayaPointGoalDiffDriveEnvCfg(KayaPointGoalEnvCfg):
    holonomic = False       # vy forced to 0: what holonomy buys is measured against this


def register():
    for task_id, env_cfg, agent_cfg in [
        ("Kaya-PointGoal-v0", "KayaPointGoalEnvCfg", "KayaPointGoalPPORunnerCfg"),
        ("Kaya-PointGoal-DiffDrive-v0", "KayaPointGoalDiffDriveEnvCfg", "KayaPointGoalDiffDrivePPORunnerCfg"),
    ]:
        if task_id in gym.registry:
            continue
        gym.register(
            id=task_id,
            entry_point=f"{__name__}.point_goal_env:KayaPointGoalEnv",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": f"{__name__}:{env_cfg}",
                "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:{agent_cfg}",
            },
        )
