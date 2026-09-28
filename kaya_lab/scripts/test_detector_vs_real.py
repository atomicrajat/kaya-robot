"""The simulated detector's geometry against the real frames of 2026-09-27.
Ball positions come from the sim contact probe (touching) and the real frames'
sphere fit; the real image numbers from the HSV masks of those frames."""
import sys
from pathlib import Path
import torch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kaya_lab.detector import project, detect   # noqa: E402
from kaya_lab.ball import BALL_RADIUS           # noqa: E402

z = BALL_RADIUS - 0.037            # ball centre height in base_link (base_link is 3.7 cm up)
cases = [   # name, ball centre in base_link (x, y), real top edge row, real centre column
    ("touching, centred (sim contact x 15.9 cm)", 0.159, 0.000, 383, 481),
    ("touching, one wheel",                       0.159, 0.060, 387, 334),
    ("touching, other wheel",                     0.159, -0.060, 381, 664),
]
b = torch.tensor([[x, y, z] for _, x, y, _, _ in cases])
u, v, R, _ = project(b)
ok = True
for i, (name, x, y, top, col) in enumerate(cases):
    t = (v[i] - R[i]).item()
    print(f"{name:42s} top edge row sim {t:5.0f} real {top}   centre col sim {u[i].item():5.0f} real {col}")
    ok &= abs(t - top) < 25
# whole-ball frame at ~0.5 m: real blob 124 px wide, bottom rows ~432
b5 = torch.tensor([[0.024 + 0.50, -0.02, z]])
u5, v5, R5, _ = project(b5)
print(f"{'~0.5 m ahead':42s} diameter sim {2*R5.item():5.0f} px real 124-125   bottom row sim {(v5+R5).item():5.0f} real ~432")
valid, meas, trunc = detect(torch.tensor([[0.159, 0.0, z], [-0.3, 0.0, z], [0.3, 1.0, z], [2.0, 0.0, z]]))
print("visible? touching", bool(valid[0] or trunc[0]), "| behind", bool(valid[1]), "| 73 deg to the side", bool(valid[2]), "| 2 m ahead", bool(valid[3]))
print("PASS" if ok else "CHECK")
