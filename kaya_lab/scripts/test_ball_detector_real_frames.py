"""The image detector on the REAL frames of 2026-09-27. No ground truth for the
ball position exists beyond how it was placed, so check: it detects every frame,
the fit residual is small, and the positions agree with the placements
(0.5 m away; 10 cm gap; touching: centred, at each front wheel)."""
import sys
from pathlib import Path
import pathlib
import cv2
import numpy as np
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from kaya_lab.ball_detector import BallDetector, optical_from_pitch   # noqa: E402

sys.path.insert(0, str(HERE / "scripts"))
import robot_camera_calib as cal   # noqa: E402
K = [[404.85430908, 0, 423.50286865], [0, 405.0791626, 237.02305603], [0, 0, 1]]
T_doc = optical_from_pitch((0.024, 0.024, 0.138), np.radians(4.89))
variants = {
    "documented pose, no distortion": BallDetector(K, None, T_doc, 0.070, 0.070 - 0.037),
    "robot TF + real distortion": BallDetector(cal.K, cal.D, cal.T_base_optical(), 0.070, 0.070 - 0.037),
}
# The real frames are camera images of a home and are kept OUTSIDE the public repo
# (KAYA_PRIVATE_MEDIA, default ../private_media beside the repo), see kaya_lab/data/README.md.
import os
d = pathlib.Path(os.environ.get("KAYA_PRIVATE_MEDIA", HERE.parent.parent / "private_media")) / "kaya_lab" / "data" / "ball_frames_20260927"
for f, note in [("ball_0p5m.png", "~0.5 m ahead"), ("ball_10cm_gap.png", "10 cm gap"),
                ("ball_touch_centre.png", "touching, centred (sim contact x 15.3)"),
                ("ball_touch_wheel_a.png", "touching + wheel"), ("ball_touch_wheel_b.png", "touching + other wheel")]:
    rgb = cv2.cvtColor(cv2.imread(str(d / f)), cv2.COLOR_BGR2RGB)
    for name, det in variants.items():
        r = det.detect(rgb)
        print(f"{note:32s} [{name:31s}] base x {r.x*100:5.1f} y {r.y*100:+5.1f} cm  rms {r.rms_deg:.2f} deg")
