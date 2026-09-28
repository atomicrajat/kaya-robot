"""B1: push the ball into the goal. Holonomic, and the vy = 0 ablation."""

import gymnasium as gym

from isaaclab.utils.configclass import configclass

from .ball_goal_env_cfg import KayaBallGoalDetEnvCfg, KayaBallGoalEnvCfg  # noqa: F401


@configclass
class KayaBallGoalDiffDriveEnvCfg(KayaBallGoalEnvCfg):
    holonomic = False


def register():
    for task_id, env_cfg, agent_cfg, entry in [
        ("Kaya-BallGoal-v0", "KayaBallGoalEnvCfg", "KayaBallGoalPPORunnerCfg", "ball_goal_env:KayaBallGoalEnv"),
        ("Kaya-BallGoal-DiffDrive-v0", "KayaBallGoalDiffDriveEnvCfg", "KayaBallGoalDiffDrivePPORunnerCfg",
         "ball_goal_env:KayaBallGoalEnv"),
        # B2: the camera detector instead of the true ball
        ("Kaya-BallGoal-Det-v0", "KayaBallGoalDetEnvCfg", "KayaBallGoalDetPPORunnerCfg",
         "ball_goal_det_env:KayaBallGoalDetEnv"),
    ]:
        if task_id in gym.registry:
            continue
        gym.register(
            id=task_id,
            entry_point=f"{__name__}.{entry}",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": f"{__name__}:{env_cfg}",
                "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:{agent_cfg}",
            },
        )
