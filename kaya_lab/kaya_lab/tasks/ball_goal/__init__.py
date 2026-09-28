"""B1: push the ball into the goal. Holonomic, and the vy = 0 ablation."""

import gymnasium as gym

from isaaclab.utils.configclass import configclass

from .ball_goal_env_cfg import (KayaBallGoalDetEnvCfg, KayaBallGoalEnvCfg, KayaBallGoalV2EnvCfg,  # noqa: F401
                                KayaBallGoalV21EnvCfg, KayaBallGoalV22EnvCfg,
                                KayaBallGoalV23EnvCfg)


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
        # v2: a room with walls, corners and obstacles, and a depth scan ("footballer")
        ("Kaya-BallGoal-V2-v0", "KayaBallGoalV2EnvCfg", "KayaBallGoalV2PPORunnerCfg",
         "ball_goal_v2_env:KayaBallGoalV2Env"),
        # v2.1: a ball that jams when pushed into walls; fine-tuned from v2 (same runner/experiment)
        ("Kaya-BallGoal-V21-v0", "KayaBallGoalV21EnvCfg", "KayaBallGoalV2PPORunnerCfg",
         "ball_goal_v2_env:KayaBallGoalV2Env"),
        # v2.2: strike off-centre near walls (no square-on pushes into a surface), 0.10 m/s
        ("Kaya-BallGoal-V22-v0", "KayaBallGoalV22EnvCfg", "KayaBallGoalV2PPORunnerCfg",
         "ball_goal_v2_env:KayaBallGoalV2Env"),
        # v2.3: back off and strike a jammed ball instead of holding the press (escalating jam penalty + kick)
        ("Kaya-BallGoal-V23-v0", "KayaBallGoalV23EnvCfg", "KayaBallGoalV2PPORunnerCfg",
         "ball_goal_v2_env:KayaBallGoalV2Env"),
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
