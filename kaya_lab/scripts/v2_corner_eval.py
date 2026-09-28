"""Compare v2 policies on one hard start: the ball against the OUTSIDE of a box side panel,
in the corner with the goal wall - where the real robot pinned the ball for 25 s (run 20's
video). Same seed, same starts for every policy; deterministic actions, as on the robot.

    cd ~/work_projects/kaya_robot/kaya
    PYTHONPATH=$PWD/kaya_lab ~/IsaacLab/isaaclab.sh -p kaya_lab/scripts/v2_corner_eval.py \
        --ckpt v22=<run_dir>/model_549.pt --ckpt v23=<run_dir>/model_798.pt [--panel_x 0.23 0.23]

--panel_x: how far behind the mouth the ball starts, m (0.23 = touching the goal wall).
Reports per policy: success rate, timeouts, mean time to score, the fraction of steps with
the ball jammed, and kicks (backed off, then freed it off-centre) per episode.
"""

import argparse
import json
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--ckpt", action="append", required=True, help="name=path/to/model_N.pt")
parser.add_argument("--episodes", type=int, default=512, help="rounded to whole episodes per env")
parser.add_argument("--num_envs", type=int, default=256)
parser.add_argument("--panel_x", type=float, nargs=2, default=(0.23, 0.23))
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--measured_ball", action="store_true", help="the real ball's measured squash and grip")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402

from kaya_lab.tasks.ball_goal.ball_goal_env_cfg import KayaBallGoalV23EnvCfg  # noqa: E402
from kaya_lab.tasks.ball_goal.ball_goal_v2_env import KayaBallGoalV2Env  # noqa: E402


def load_actor(path, device):
    """The actor MLP straight from an RSL-RL checkpoint, as numpy_policy.py computes it
    (normaliser eps 0.01, ELU), so no play/export run is needed first."""
    sd = torch.load(path, map_location=device, weights_only=False)["actor_state_dict"]
    idx = sorted({int(k.split(".")[1]) for k in sd if k.startswith("mlp.") and k.endswith(".weight")})
    mean, std = sd["obs_normalizer._mean"][0], sd["obs_normalizer._std"][0]
    layers = [(sd[f"mlp.{i}.weight"], sd[f"mlp.{i}.bias"]) for i in idx]

    def act(obs):
        x = (obs - mean) / (std + 0.01)
        for j, (w, b) in enumerate(layers):
            x = x @ w.T + b
            if j < len(layers) - 1:
                x = torch.nn.functional.elu(x)
        return x
    return act


def evaluate(env, cfg, act):
    torch.manual_seed(args.seed)
    obs, _ = env.reset()
    # count exactly the first K episodes of EVERY env: stopping at a total instead would drop
    # the slow episodes still running and flatter the success rate
    K = max(1, args.episodes // env.num_envs)
    n_done = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
    succ = fail = timeout = 0
    t_succ, jam_steps, steps, kicks = [], 0, 0, 0
    ep_t = torch.zeros(env.num_envs, device=env.device)
    while bool((n_done < K).any()):
        live = n_done < K
        prev_kicked = env._kicked.clone()
        obs, rew, term, trunc, _ = env.step(act(obs["policy"]))
        ep_t += 1
        steps += int(live.sum())
        jam_steps += int(((env._jam_run > 0) & live).sum())
        kicks += int((~prev_kicked & env._kicked & (env._since_jam <= cfg.kick_window) & live).sum())
        s = (rew > 0.5 * cfg.rew_success) & live
        succ += int(s.sum())
        t_succ += (ep_t[s] / 10.0).tolist()
        fail += int((term & ~s & live).sum())
        timeout += int((trunc & ~term & live).sum())
        done = term | trunc
        n_done += (done & live).long()
        ep_t[done] = 0
    total = succ + fail + timeout
    return {"episodes": total, "success_rate": round(succ / total, 3), "timeout_rate": round(timeout / total, 3),
            "fail_rate": round(fail / total, 3),
            "mean_time_to_score_s": round(sum(t_succ) / max(len(t_succ), 1), 1),
            "jammed_step_frac": round(jam_steps / steps, 3), "kicks_per_episode": round(kicks / total, 2)}


def main():
    cfg = KayaBallGoalV23EnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.seed = args.seed
    cfg.initial_level = 1.0
    cfg.curriculum_step = 0.0                     # hold the level: an eval, not training
    cfg.wall_ball_frac = (1.0, 1.0)               # every start against a wall ...
    cfg.side_panel_frac = 1.0                     # ... namely a box side panel
    cfg.side_panel_x = tuple(args.panel_x)
    if args.measured_ball:
        from kaya_lab.ball import ball_cfg_measured
        cfg.ball = ball_cfg_measured().replace(prim_path=cfg.ball.prim_path)
    env = KayaBallGoalV2Env(cfg)
    # one inference_mode for every policy: the env's buffers are created as inference tensors
    # on the first step, and a reset outside the mode may not write to them
    with torch.inference_mode():
        for spec in args.ckpt:
            name, path = spec.split("=", 1)
            t0 = time.time()
            out = {"policy": name, "panel_x": list(args.panel_x), "measured_ball": args.measured_ball, **evaluate(env, cfg, load_actor(path, env.device)),
                   "wall_s": round(time.time() - t0, 1)}
            print("[corner] RESULT " + json.dumps(out), flush=True)
    env.close()


if __name__ == "__main__":
    main()
    app.close()
