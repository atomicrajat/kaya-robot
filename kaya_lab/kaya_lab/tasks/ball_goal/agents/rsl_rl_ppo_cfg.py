"""PPO for the ball task. Bigger networks and longer rollouts than M1: the
task is longer-horizon (find, get behind, push, steer) and less forgiving."""

from isaaclab.utils.configclass import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class KayaBallGoalPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32              # 3.2 s per env per iteration
    max_iterations = 1500
    save_interval = 50
    experiment_name = "kaya_ball_goal"
    obs_groups = {"actor": ["policy"], "critic": ["policy"]}
    actor = RslRlMLPModelCfg(
        hidden_dims=[128, 128, 64],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.5),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[128, 128, 64], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.995,                    # longer task: look ~20 s ahead
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class KayaBallGoalDiffDrivePPORunnerCfg(KayaBallGoalPPORunnerCfg):
    experiment_name = "kaya_ball_goal_diffdrive"


@configclass
class KayaBallGoalDetPPORunnerCfg(KayaBallGoalPPORunnerCfg):
    experiment_name = "kaya_ball_goal_det"
    # asymmetric actor-critic: the critic's "critic" group holds the policy
    # observations plus the true ball state
    obs_groups = {"actor": ["policy"], "critic": ["critic"]}


@configclass
class KayaBallGoalV2PPORunnerCfg(KayaBallGoalDetPPORunnerCfg):
    """v2: 31 inputs (B2's 15 + a 16-ray depth scan), a longer and harder task."""
    experiment_name = "kaya_ball_v2"
    max_iterations = 2000
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.5),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True)
