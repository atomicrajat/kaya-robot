"""What the robot's yellow-ball detector reports, simulated without rendering.

The real pipeline (B5): HSV threshold on the 848x480 colour image -> the
biggest yellow blob -> fit a sphere silhouette -> ball position on the floor.
Training cannot render 4096 cameras, so this module produces the SAME output
from the true ball position:

  * visible only if the ball's disc lands on the image (92 x 61 deg FOV,
    camera pitched 4.89 deg up, 17.5 cm above the floor);
  * a ball cut off by the image edge is still detected (touching the front it
    is a cap in the bottom ~20% of the image - measured 2026-09-27), just
    noisier;
  * range noise grows with distance; bearing is precise;
  * random dropouts;
  * 100 ms latency (applied by the env: it uses the previous step's output).

Camera numbers: the colour lens pose in base_link measured from the USD
(b0_ball_contact_probe: x 2.4, y 2.4, z 13.8 cm) and the robot's own
/camera/color/camera_info (fx 404.85, fy 405.08, cx 423.5, cy 237.0).
"""

import math

import torch

from .ball import BALL_RADIUS

W, H = 848, 480
FX, FY, CX, CY = 404.85, 405.08, 423.5, 237.0
LENS_B = (0.024, 0.024, 0.138)          # colour lens in base_link [m]
PITCH_UP = math.radians(4.89)           # measured with the real IMU
EDGE_MIN_PX = 12                        # at least this much of the ball on the image to detect it

# noise, from the sphere-fit results on real frames (fits good to 0.4-0.6 deg
# on a whole ball, "a few cm" on a cut-off one)
RANGE_SIGMA = (0.01, 0.03)              # [m] + [m per m of distance]
TRUNC_SIGMA = 0.02                      # [m] extra when the ball is cut by the image edge
BEARING_SIGMA = math.radians(0.7)
DROPOUT = 0.05


def project(ball_b: torch.Tensor):
    """Ball centre in base_link (N, 3) -> image centre u, v, radius R [px] and
    depth along the optical axis z [m]."""
    t = torch.tensor(LENS_B, device=ball_b.device)
    p = ball_b - t
    c, s = math.cos(PITCH_UP), math.sin(PITCH_UP)
    fwd = p[:, 0] * c + p[:, 2] * s                # optical z
    up = -p[:, 0] * s + p[:, 2] * c
    x_c, y_c = -p[:, 1], -up                       # optical x right, y down
    z = fwd.clamp(min=1e-3)
    u = CX + FX * x_c / z
    v = CY + FY * y_c / z
    d = torch.linalg.norm(p, dim=1)
    R = FX * BALL_RADIUS / torch.sqrt((d**2 - BALL_RADIUS**2).clamp(min=1e-6))
    return u, v, R, fwd


def detect(ball_b: torch.Tensor, rng: torch.Generator | None = None):
    """Returns (valid, meas_xy, truncated): the detector's estimate of the
    ball's floor position in base_link, (N, 2), where valid is True."""
    u, v, R, fwd = project(ball_b)
    on_image = (fwd > 0.05) & (u + R > EDGE_MIN_PX) & (u - R < W - EDGE_MIN_PX) & \
               (v - R < H - EDGE_MIN_PX) & (v + R > EDGE_MIN_PX)
    truncated = (u - R < 0) | (u + R > W) | (v + R > H) | (v - R < 0)
    n = ball_b.shape[0]
    dropped = torch.rand(n, device=ball_b.device, generator=rng) < DROPOUT
    valid = on_image & ~dropped
    # polar noise around the lens (range grows noisier with distance)
    rel = ball_b[:, :2] - torch.tensor(LENS_B[:2], device=ball_b.device)
    rng_ = torch.linalg.norm(rel, dim=1)
    bearing = torch.atan2(rel[:, 1], rel[:, 0])
    sig_r = RANGE_SIGMA[0] + RANGE_SIGMA[1] * rng_ + TRUNC_SIGMA * truncated.float()
    rng_n = (rng_ + sig_r * torch.randn(n, device=ball_b.device, generator=rng)).clamp(min=0.05)
    bear_n = bearing + BEARING_SIGMA * torch.randn(n, device=ball_b.device, generator=rng)
    meas = torch.stack([rng_n * torch.cos(bear_n), rng_n * torch.sin(bear_n)], 1) + \
        torch.tensor(LENS_B[:2], device=ball_b.device)
    return valid, meas, truncated
