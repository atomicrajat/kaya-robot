"""PPO for the point-goal task (RSL-RL, Isaac Lab's default).

One PPO iteration = every env plays num_steps_per_env policy steps (the
"rollout"), then the networks are updated on that batch. With 4096 envs and
24 steps that is ~98k samples per iteration.
"""

from isaaclab.utils.configclass import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class KayaPointGoalPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24          # 2.4 s of experience per env per iteration
    max_iterations = 600
    save_interval = 50
    experiment_name = "kaya_point_goal"
    # both networks see the same observations for now (the ball task will give
    # the critic privileged extras: an "asymmetric" actor-critic)
    obs_groups = {"actor": ["policy"], "critic": ["policy"]}
    # actor: observations -> mean twist; exploration noise starts at std 0.5 of full scale
    actor = RslRlMLPModelCfg(
        hidden_dims=[64, 64],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.5),
    )
    # critic: observations -> "how much reward is still to come from here"
    critic = RslRlMLPModelCfg(hidden_dims=[64, 64], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,                 # 10 Hz: rewards ~10 s ahead still count
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class KayaPointGoalDiffDrivePPORunnerCfg(KayaPointGoalPPORunnerCfg):
    experiment_name = "kaya_point_goal_diffdrive"
