"""The yellow-ball detector that runs on camera IMAGES: in the full-fidelity
sim (B4) on rendered frames, and on the robot (B5) on real ones. Plain numpy +
OpenCV + SciPy, no Isaac imports, so the robot can use this file unchanged.

    image (RGB) -> HSV threshold -> close the stripes -> biggest blob
      -> its silhouette edge (image-border pixels dropped: not a real edge)
      -> undistort (camera_info D) -> rays in the robot's base frame
      -> least-squares fit of a sphere of known radius resting on the floor
      -> ball centre (x, y) in base_link

Why a sphere fit and not "blob width -> distance": a ball cut off by the
image edge (always, when it is being pushed) has no measurable width, and a
circle is the wrong shape off-axis. Both mistakes were made and caught on the
real frames of 2026-09-27 (LEARNING_LOG.md). The training-time stand-in for
this code is detector.py.
"""

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares

# OpenCV hue 0-180. Measured 2026-09-28: the real ball is H 20-28 (median 23-26),
# S median ~200 in all five real frames; brown cardboard rendered in sim is H 17-19,
# S ~100, and the first threshold (H >= 18, S >= 90) took the goal box for the ball.
# Real cardboard under real light must be re-checked on the robot (B5).
HSV_LO = (20, 120, 80)
HSV_HI = (34, 255, 255)
CLOSE_PX = 15                  # closes the ball's black stripes
MIN_AREA_PX = 150
MAX_CANDIDATES = 3            # fit this many of the biggest blobs; keep the roundest
YOLO_CONF = 0.30              # real ball: 0.74-0.79 (YOLO11s, pretrained, 2026-09-28)
NEAR_X = 0.35                 # [m] touching zone for the colour-only fallback
MIN_X_BASE = 0.12             # [m] no ball centre closer than this in front of base_link (real touching: ~13.8)


@dataclass
class Detection:
    ok: bool
    x: float = 0.0             # ball centre in base_link [m]
    y: float = 0.0
    rms_deg: float = 0.0       # silhouette fit residual
    area_px: int = 0
    truncated: bool = False
    source: str = ""           # "yolo", "near" (touching-zone colour fallback) or "colour" (no YOLO given)


class BallDetector:
    def __init__(self, K, D, T_base_optical, ball_radius: float, ball_centre_z_base: float, yolo=None):
        """K (3x3), D (distortion, any OpenCV length) from camera_info;
        T_base_optical (4x4): the optical frame (x right, y down, z forward)
        in base_link, from TF on the robot or the camera pose in sim."""
        self.K = np.asarray(K, dtype=np.float64)
        self.D = np.zeros(5) if D is None else np.asarray(D, dtype=np.float64)
        self.R = np.asarray(T_base_optical, dtype=np.float64)[:3, :3]
        self.t = np.asarray(T_base_optical, dtype=np.float64)[:3, 3]
        self.r = ball_radius
        self.zc = ball_centre_z_base
        self.kernel = np.ones((CLOSE_PX, CLOSE_PX), np.uint8)
        self.yolo = yolo           # robot/yolo_trt.YoloTRT, or None for colour only (sim)

    def mask(self, rgb: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        m = cv2.inRange(hsv, HSV_LO, HSV_HI)
        return cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel)

    def _rays(self, px: np.ndarray) -> np.ndarray:
        """Pixels (N, 2) -> unit rays (N, 3) in base_link."""
        n = cv2.undistortPoints(px.reshape(-1, 1, 2).astype(np.float64), self.K, self.D).reshape(-1, 2)
        d = np.column_stack([n, np.ones(len(n))])
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        return d @ self.R.T

    def detect(self, rgb: np.ndarray) -> Detection:
        """The best ball among the few biggest yellow blobs. Taking only THE biggest
        blob failed on the robot (run 3, 2026-09-28): a lit patch of a big cardboard
        box in the room was bigger than the real ball, got rejected as not round,
        and the real ball in the same frame was never looked at."""
        h, w = rgb.shape[:2]
        m = self.mask(rgb)
        if self.yolo is not None:
            return self._detect_hybrid(rgb, m, h, w)
        n, lab, st, cen = cv2.connectedComponentsWithStats(m)
        if n < 2:
            return Detection(False)
        order = 1 + np.argsort(-st[1:, cv2.CC_STAT_AREA])[:MAX_CANDIDATES]
        best = Detection(False)
        for i in order:
            area = int(st[i, cv2.CC_STAT_AREA])
            if area < MIN_AREA_PX:
                break
            d = self._fit_blob((lab == i).astype(np.uint8), area, h, w)
            if d.ok and (not best.ok or d.rms_deg < best.rms_deg):
                best = d
                best.source = "colour"
        if not best.ok and st[order[0], cv2.CC_STAT_AREA] >= MIN_AREA_PX:
            best.area_px = int(st[order[0], cv2.CC_STAT_AREA])
        return best

    def _detect_hybrid(self, rgb, m, h, w) -> Detection:
        """Step 1 by YOLO (which object is the ball), step 2 by the sphere fit on the
        yellow pixels inside its box (where exactly). YOLO misses a ball cut off by
        the image edge (touching the robot), so with no YOLO box a colour blob is
        accepted ONLY in the touching zone: on the bottom edge, fitted within NEAR_X
        of the robot. Cardboard and furniture, up and away, can never qualify."""
        for conf, x1, y1, x2, y2 in self.yolo.detect(rgb, conf=YOLO_CONF)[:3]:
            pad = 0.1 * max(x2 - x1, y2 - y1)
            X1, Y1 = max(int(x1 - pad), 0), max(int(y1 - pad), 0)
            X2, Y2 = min(int(x2 + pad), w), min(int(y2 + pad), h)
            sub = np.zeros_like(m)
            sub[Y1:Y2, X1:X2] = m[Y1:Y2, X1:X2]
            n, lab, st, _ = cv2.connectedComponentsWithStats(sub)
            if n < 2:
                continue
            i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
            area = int(st[i, cv2.CC_STAT_AREA])
            if area < MIN_AREA_PX:
                continue
            d = self._fit_blob((lab == i).astype(np.uint8), area, h, w)
            if d.ok:
                d.source = "yolo"
                return d
        n, lab, st, _ = cv2.connectedComponentsWithStats(m)
        for i in 1 + np.argsort(-st[1:, cv2.CC_STAT_AREA])[:MAX_CANDIDATES]:
            area = int(st[i, cv2.CC_STAT_AREA])
            if area < MIN_AREA_PX:
                break
            if st[i, cv2.CC_STAT_TOP] + st[i, cv2.CC_STAT_HEIGHT] < h - 2:
                continue                                     # not on the bottom edge
            d = self._fit_blob((lab == i).astype(np.uint8), area, h, w)
            if d.ok and d.x <= NEAR_X:
                d.source = "near"
                return d
        return Detection(False)

    def _fit_blob(self, blob: np.ndarray, area: int, h: int, w: int) -> Detection:
        cs, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # A sphere's silhouette is convex, so any dent in the blob is not a real edge: on
        # the real ball, the black stripes and the glare patch on top notch the outline
        # (touching frames fitted at 4-6 deg rms and ~20 cm wrong). Fit the convex hull,
        # resampled densely along its sides.
        hull = cv2.convexHull(max(cs, key=len))[:, 0, :].astype(np.float64)
        pts = []
        for a, b in zip(hull, np.roll(hull, -1, axis=0)):
            k = max(1, int(np.hypot(*(b - a))))
            pts.append(a + (b - a) * (np.arange(k)[:, None] / k))
        c = np.concatenate(pts)
        on_border = (c[:, 0] <= 2) | (c[:, 0] >= w - 3) | (c[:, 1] <= 2) | (c[:, 1] >= h - 3)
        edge = c[~on_border]
        truncated = bool(on_border.any())
        if len(edge) < 12:
            return Detection(False, area_px=area, truncated=truncated)
        if len(edge) > 200:                       # plenty for a fit; keeps it fast on the Jetson
            edge = edge[np.linspace(0, len(edge) - 1, 200).astype(int)]
        rays = self._rays(edge)

        def res(q):
            C = np.array([q[0], q[1], self.zc]) - self.t
            d = np.linalg.norm(C)
            return np.arccos(np.clip(rays @ (C / d), -1, 1)) - np.arcsin(min(self.r / d, 1.0))

        # Least squares only finds the nearest valley; for a cut-off ball the obvious
        # starting guesses (blob centroid, top point) sit in the wrong one (seen on the
        # real frames: a touching ball fitted at 30 cm). So first a coarse grid over the
        # floor ahead of the camera, scored in one vectorised pass, then refine the best.
        # x starts at MIN_X_BASE: the ball's centre cannot be inside the robot (it touches
        # the flat front at x = 0.0825 m + radius: 0.1525 for the 14 cm ball), so a fit there is a wrong valley
        gx, gy = np.meshgrid(np.arange(MIN_X_BASE, 3.0, 0.05), np.arange(-1.5, 1.5, 0.05), indexing="ij")
        G = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, self.zc)]) - self.t
        dG = np.linalg.norm(G, axis=1)
        ang = np.arccos(np.clip((G / dG[:, None]) @ rays.T, -1, 1))                   # (grid, edge)
        cost = ((ang - np.arcsin(np.minimum(self.r / dG, 1.0))[:, None]) ** 2).sum(1)
        g0 = (G[np.argmin(cost)] + self.t)[:2]
        best = least_squares(res, g0, bounds=([MIN_X_BASE, -3.0], [5.0, 3.0]), max_nfev=200)
        rms = float(np.degrees(np.sqrt(np.mean(best.fun**2))))
        return Detection(True, float(best.x[0]), float(best.x[1]), rms, area, truncated)


def optical_from_pitch(lens_base, pitch_up_rad: float) -> np.ndarray:
    """T_base_optical for a camera at lens_base, pitched up, no roll/yaw
    (optical z forward, x right, y down; base x forward, y left, z up)."""
    c, s = np.cos(pitch_up_rad), np.sin(pitch_up_rad)
    fwd = np.array([c, 0.0, s])
    right = np.array([0.0, -1.0, 0.0])
    down = np.cross(fwd, right)           # z x x = y (right-handed optical frame)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = right, down, fwd
    T[:3, 3] = lens_base
    return T
