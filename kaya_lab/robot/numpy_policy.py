"""The trained policy as plain numpy: the robot has no PyTorch or ONNX runtime,
and the network is small (15 -> 128 -> 128 -> 64 -> 3, ELU), so a few matrix
multiplications are all it takes. Weights come from export_policy_npz.py."""
import numpy as np


class NumpyPolicy:
    def __init__(self, npz_path: str):
        z = np.load(npz_path)
        self.mean, self.std, self.eps = z["obs_mean"], z["obs_std"], float(z["eps"])
        self.layers = [(z[f"w{i}"], z[f"b{i}"]) for i in range(int(z["n_layers"]))]

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        """obs (15,) or (N, 15) -> mean action (3,) or (N, 3); the deterministic action,
        as used at deployment (no exploration noise)."""
        x = (np.asarray(obs, dtype=np.float64) - self.mean) / (self.std + self.eps)
        for i, (w, b) in enumerate(self.layers):
            x = x @ w.T + b
            if i < len(self.layers) - 1:
                x = np.where(x > 0, x, np.expm1(x))          # ELU
        return x
