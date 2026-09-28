"""Checkpoint (model_N.pt) -> policy.npz for the robot's numpy policy, and a check
that numpy gives the SAME actions as Isaac Lab's own TorchScript export.

    ~/isaac-sim/python.sh kaya_lab/scripts/export_policy_npz.py <run_dir> <model_N.pt>
(run play once first so <run_dir>/exported/policy.pt exists for the check)"""
import sys
from pathlib import Path
import numpy as np
import torch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
from numpy_policy import NumpyPolicy   # noqa: E402

run, ckpt = Path(sys.argv[1]), sys.argv[2]
sd = torch.load(run / ckpt, map_location="cpu", weights_only=False)["actor_state_dict"]
idx = sorted({int(k.split(".")[1]) for k in sd if k.startswith("mlp.") and k.endswith(".weight")})
out = {"obs_mean": sd["obs_normalizer._mean"].numpy()[0], "obs_std": sd["obs_normalizer._std"].numpy()[0],
       "n_layers": np.array(len(idx))}
for j, i in enumerate(idx):
    out[f"w{j}"] = sd[f"mlp.{i}.weight"].numpy()
    out[f"b{j}"] = sd[f"mlp.{i}.bias"].numpy()
obs = torch.randn(2000, out["w0"].shape[1]) * 1.5
ref = None
jit = run / "exported" / "policy.pt"
if jit.exists():
    ref = torch.jit.load(str(jit)).eval()(obs).detach().numpy()
best = None
(run / "exported").mkdir(exist_ok=True)
# RSL-RL's normaliser divides by (std + eps); eps = 0.01, found by matching the
# TorchScript export (2026-09-28). Without an export to compare against, use that.
for eps in ((0.0, 1e-8, 1e-5, 1e-2) if ref is not None else (1e-2,)):
    out["eps"] = np.array(eps)
    np.savez(run / "exported" / "policy.npz", **out)
    act = NumpyPolicy(str(run / "exported" / "policy.npz"))(obs.numpy())
    if ref is None:
        break
    err = np.abs(act - ref).max()
    if best is None or err < best[0]:
        best = (err, eps)
if ref is not None:
    out["eps"] = np.array(best[1])
    np.savez(run / "exported" / "policy.npz", **out)
    print(f"numpy vs TorchScript: max |action diff| = {best[0]:.2e} (eps {best[1]}) over 2000 random observations")
    assert best[0] < 1e-4, "numpy policy does not match the TorchScript export"
print("wrote", run / "exported" / "policy.npz")
