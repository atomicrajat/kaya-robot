"""Can the ball task be scored at all? A scripted "drive straight forward"
policy at curriculum level 0 (robot just behind the ball, facing it). If this
never scores, the success test or the geometry is wrong, not the learning.

    kaya_lab/scripts/rl.sh ... is for RL; run this one directly:
    PYTHONPATH=kaya_lab ~/IsaacLab/isaaclab.sh -p kaya_lab/scripts/b1_scripted_check.py
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=256)
parser.add_argument("--seconds", type=float, default=15.0)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402

from kaya_lab.ball import BALL_RADIUS  # noqa: E402
from kaya_lab.tasks.ball_goal.ball_goal_env import KayaBallGoalEnv  # noqa: E402
from kaya_lab.tasks.ball_goal.ball_goal_env_cfg import KayaBallGoalEnvCfg  # noqa: E402


def main():
    cfg = KayaBallGoalEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.episode_length_s = args.seconds + 5
    env = KayaBallGoalEnv(cfg)
    env.reset()
    n = env.num_envs
    ever = torch.zeros(n, dtype=torch.bool, device=env.device)
    best_x = torch.full((n,), -9.0, device=env.device)
    act = torch.zeros(n, 3, device=env.device)
    act[:, 0] = 0.7                                   # forward, ~0.1 m/s
    steps = int(args.seconds / env.step_dt)
    for _ in range(steps):
        b = env._ball_env()
        best_x = torch.maximum(best_x, b[:, 0])
        _, rew, terminated, truncated, _ = env.step(act)
        # the env resets a scored episode inside step() and clears _success, so read
        # the +100 success reward, which lands in this step's reward before the reset
        ever |= rew > 0.5 * cfg.rew_success
    gl = cfg.goal_line_x
    print(f"[check] scripted forward push, level 0, {n} envs, {args.seconds:.0f} s")
    print(f"[check] scored: {ever.float().mean().item()*100:.1f}% of envs")
    print(f"[check] furthest ball x seen before any reset: median {((best_x - gl)*100).median().item():+.1f} cm, "
          f"max {((best_x - gl)*100).max().item():+.1f} cm (scoring needs > +{BALL_RADIUS*100:.1f} cm)")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
