"""B4: evaluate a trained B2 policy at full fidelity, and compare with the
same policy on the training-time stand-in detector - same script, same
starts, only the detector differs.

    cd ~/work_projects/kaya_robot/kaya
    PYTHONPATH=$PWD/kaya_lab ~/IsaacLab/isaaclab.sh -p kaya_lab/scripts/b4_eval.py \
        --policy ~/IsaacLab/logs/rsl_rl/kaya_ball_goal_det/2026-09-28_01-58-14/exported/policy.pt \
        --detector image --episodes 100 --enable_cameras [--viz kit]

--detector model : the stand-in (detector.py), 4096-env-style fidelity check of the export
--detector image : rendered camera + ball_detector.py (B4 proper)
"""

import argparse
import json
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--policy", required=True, help="exported TorchScript policy.pt")
parser.add_argument("--detector", choices=["model", "image"], default="image")
parser.add_argument("--episodes", type=int, default=100)
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--seed", type=int, default=0)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402

from kaya_lab.tasks.ball_goal.ball_goal_b4_env import KayaBallGoalB4Env, KayaBallGoalB4EnvCfg  # noqa: E402
from kaya_lab.tasks.ball_goal.ball_goal_det_env import KayaBallGoalDetEnv  # noqa: E402


def main():
    torch.manual_seed(args.seed)
    cfg = KayaBallGoalB4EnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.seed = args.seed
    if args.detector == "model":
        # identical starts and physics, but the stand-in detector and no camera
        cfg.camera = None
        env = KayaBallGoalDetEnv(cfg)
    else:
        env = KayaBallGoalB4Env(cfg)
    policy = torch.jit.load(args.policy, map_location=env.device).eval()
    obs, _ = env.reset()
    n = env.num_envs
    succ = fail = timeout = 0
    t0 = time.time()
    with torch.inference_mode():
        while succ + fail + timeout < args.episodes:
            act = policy(obs["policy"])
            obs, rew, term, trunc, _ = env.step(act)
            s = rew > 0.5 * cfg.rew_success               # scored this step (the env resets inside step)
            f = term & ~s
            succ += int(s.sum())
            fail += int(f.sum())
            timeout += int((trunc & ~term).sum())
    total = succ + fail + timeout
    out = {"detector": args.detector, "episodes": total, "success_rate": round(succ / total, 3),
           "fail_rate": round(fail / total, 3), "timeout_rate": round(timeout / total, 3),
           "wall_s": round(time.time() - t0, 1)}
    if args.detector == "image" and env.det_log:
        L = np.array(env.det_log, dtype=np.float64)
        tx, ty, vis, ok, ex, ey, trunc, rms = L.T
        vis, ok, trunc = vis.astype(bool), ok.astype(bool), trunc.astype(bool)
        err = np.hypot(ex - tx, ey - ty)
        rng = np.hypot(tx, ty)
        both = vis & ok
        out["detector_stats"] = {
            "frames": int(len(L)),
            "detected_when_visible": round(float(ok[vis].mean()), 3) if vis.any() else None,
            "false_detections_when_not_visible": round(float(ok[~vis].mean()), 3) if (~vis).any() else None,
            "pos_err_median_cm": round(float(np.median(err[both]) * 100), 2),
            "pos_err_p90_cm": round(float(np.percentile(err[both], 90) * 100), 2),
            "pos_err_cut_off_median_cm": round(float(np.median(err[both & trunc]) * 100), 2) if (both & trunc).any() else None,
            "err_by_range_cm": {f"{lo:.1f}-{hi:.1f} m": round(float(np.median(err[both & (rng >= lo) & (rng < hi)]) * 100), 2)
                                for lo, hi in [(0, 0.3), (0.3, 0.7), (0.7, 1.5), (1.5, 4)] if (both & (rng >= lo) & (rng < hi)).any()},
            # detections the geometric "visible" test called impossible: if their error is
            # small they are real glimpses (a sliver of ball at the image edge), not phantoms
            "detections_when_not_visible_err_median_cm": round(float(np.median(err[ok & ~vis]) * 100), 1) if (ok & ~vis).any() else None,
            "detections_when_not_visible_err_p90_cm": round(float(np.percentile(err[ok & ~vis], 90) * 100), 1) if (ok & ~vis).any() else None,
            "phantoms_over_50cm_frac_of_all_frames": round(float((ok & ~vis & (err > 0.5)).mean()), 4),
            "stand_in_model_sigma_cm": "1 + 3/m x range (+2 if cut off)",
        }
        out["camera_K"] = np.round(env.cam_K, 2).tolist()
        out["camera_in_base"] = np.round(env.cam_T_base_optical[:3, 3], 4).tolist()
        R = env.cam_T_base_optical[:3, :3]
        out["optical_axis_in_base"] = np.round(R[:, 2], 4).tolist()      # expect ~(0.996, 0, 0.085): forward, 4.89 deg up
        out["optical_x_in_base"] = np.round(R[:, 0], 4).tolist()         # expect ~(0, -1, 0): image right = robot right
    print("[b4] RESULT " + json.dumps(out))
    env.close()


if __name__ == "__main__":
    main()
    app.close()
