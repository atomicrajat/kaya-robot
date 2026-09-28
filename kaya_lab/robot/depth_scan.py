"""The v2 policy's 16-ray depth scan, from the real depth image.

In simulation (ball_goal_v2_env._scan) 16 rays leave the colour lens LEVEL, at
camera height (17.5 cm), spread across the camera's 92 deg view, and return the
distance to the first wall / box / obstacle, capped at 3 m. They pass over the
14 cm ball.

Here: depth pixels -> 3D points in base_link -> keep the points in a height
band around the rays (15-20 cm above the floor, so the ball and the floor are
excluded) -> group by bearing from the colour lens into 16 bins -> the nearest
point per bin. Bins with nothing in them read the cap, as a ray that hit
nothing does in sim.

Known gap: the Gemini's depth starts at 0.17 m, so something closer than that
reads as nothing (the cap) here but as close in sim.
"""

import numpy as np

CAM_B = np.array([0.024, 0.024])     # the sim's scan origin: the colour lens in base_link
BASE_LINK_Z = 0.037                  # base_link above the floor


class DepthScan:
    def __init__(self, K, T_base_depth_optical, n_rays=16, fov_deg=92.0, max_range=3.0,
                 band_floor=(0.15, 0.20), stride=4):
        self.n, self.max = n_rays, max_range
        half = np.radians(fov_deg / 2)
        # bin edges in bearing, left (+) to right (-), matching the sim's ray order
        self.edges = np.linspace(half, -half, n_rays + 1)
        self.R = np.asarray(T_base_depth_optical)[:3, :3]
        self.t = np.asarray(T_base_depth_optical)[:3, 3]
        self.band = (band_floor[0] - BASE_LINK_Z, band_floor[1] - BASE_LINK_Z)   # in base_link z
        self.K = np.asarray(K)
        self.stride = stride
        self._rays = None

    def _unit_rays(self, h, w):
        """Pixel rays (z = 1) in the optical frame for the strided grid, cached."""
        v, u = np.mgrid[0:h:self.stride, 0:w:self.stride]
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        return np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, dtype=np.float64)], -1).reshape(-1, 3)

    def scan(self, depth_m: np.ndarray) -> np.ndarray:
        """depth image in metres (0 = no reading) -> (n_rays,) distances in metres."""
        h, w = depth_m.shape
        if self._rays is None:
            self._rays = self._unit_rays(h, w)
        z = depth_m[::self.stride, ::self.stride].reshape(-1)
        ok = (z > 0.17) & (z < 6.0)
        p = (self._rays[ok] * z[ok, None]) @ self.R.T + self.t          # points in base_link
        keep = (p[:, 2] > self.band[0]) & (p[:, 2] < self.band[1])
        p = p[keep, :2] - CAM_B
        out = np.full(self.n, self.max)
        if len(p) == 0:
            return out
        bearing = np.arctan2(p[:, 1], p[:, 0])
        dist = np.hypot(p[:, 0], p[:, 1])
        # bin i spans edges[i] (left) .. edges[i+1] (right)
        idx = np.searchsorted(-self.edges, -bearing) - 1
        inside = (idx >= 0) & (idx < self.n)
        np.minimum.at(out, idx[inside], np.minimum(dist[inside], self.max))
        return out


GOAL_DEPTH = 0.30
CARDBOARD_T = 0.01


def box_scan(origin_g, heading_g, half_w, n_rays=16, fov_deg=92.0, max_range=3.0):
    """The same 16 rays cast against the goal box's two side panels and the wall behind it,
    as the sim's _scan does, from the lens origin and heading in the GOAL frame (x into the
    box, origin at the mouth centre). Run 22: pressed against the outside of a side panel,
    the real depth reads nothing (plain cardboard closer than ~0.3 m, projector off) and the
    policy, told "3 m clear", pushed the ball into the panel for 16 s. The box's pose is
    known from the markers, so its part of the scan can be computed instead of sensed."""
    half = np.radians(fov_deg / 2)
    ang = heading_g + np.linspace(half, -half, n_rays)
    d = np.stack([np.cos(ang), np.sin(ang)], -1)
    d = np.where(np.abs(d) < 1e-9, 1e-9, d)
    o = np.asarray(origin_g, float)
    t = np.full(n_rays, max_range)
    # the wall behind the box (the box stands against it), facing the robot
    tw = (GOAL_DEPTH + CARDBOARD_T - o[0]) / d[:, 0]       # the wall's face, behind the back panel
    t = np.where((tw > 0) & (d[:, 0] > 0), np.minimum(t, tw), t)
    # the side panels as thin slabs (slab test, as in the sim)
    for y0, y1 in ((half_w, half_w + CARDBOARD_T), (-half_w - CARDBOARD_T, -half_w)):
        tx0, tx1 = (0.0 - o[0]) / d[:, 0], (GOAL_DEPTH - o[0]) / d[:, 0]
        ty0, ty1 = (y0 - o[1]) / d[:, 1], (y1 - o[1]) / d[:, 1]
        tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
        tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
        hit = (tmax >= tmin) & (tmin > 0)
        t = np.where(hit, np.minimum(t, tmin), t)
    return np.minimum(t, max_range)
