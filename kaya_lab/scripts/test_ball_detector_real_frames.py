"""The image detector on the REAL frames of 2026-09-27. No ground truth for the
ball position exists beyond how it was placed, so check: it detects every frame,
the fit residual is small, and the positions agree with the placements
(0.5 m away; 10 cm gap; touching: centred, at each front wheel)."""
import sys
from pathlib import Path
import cv2
import numpy as np
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from kaya_lab.ball_detector import BallDetector, optical_from_pitch   # noqa: E402

K = [[404.85430908, 0, 423.50286865], [0, 405.0791626, 237.02305603], [0, 0, 1]]
T = optical_from_pitch((0.024, 0.024, 0.138), np.radians(4.89))
det = BallDetector(K, None, T, ball_radius=0.0765, ball_centre_z_base=0.0765 - 0.037)
d = HERE / "data" / "ball_frames_20260927"
for f, note in [("ball_0p5m.png", "~0.5 m ahead"), ("ball_10cm_gap.png", "10 cm gap"),
                ("ball_touch_centre.png", "touching, centred (sim contact x 15.9)"),
                ("ball_touch_wheel_a.png", "touching + wheel"), ("ball_touch_wheel_b.png", "touching + other wheel")]:
    rgb = cv2.cvtColor(cv2.imread(str(d / f)), cv2.COLOR_BGR2RGB)
    r = det.detect(rgb)
    print(f"{note:40s} ok={r.ok} base x {r.x*100:5.1f} y {r.y*100:+5.1f} cm  fit rms {r.rms_deg:.2f} deg  "
          f"area {r.area_px} px  cut-off {r.truncated}")
