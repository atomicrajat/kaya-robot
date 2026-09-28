"""B4: the ball task at full fidelity, for EVALUATING a trained B2 policy.

What changes from B2 (training):
  * the robot's real colour camera (the USD's Camera_rgb: 848x480, the robot's
    intrinsics) is RENDERED every policy step;
  * the image detector (ball_detector.py: HSV -> blob -> sphere fit), the
    same code the robot will run, replaces the stand-in detector.py;
  * all 32 solver position iterations (the asset's own value), nominal
    servo chain, no randomisation.
If the policy still scores here, the stand-in detector and the training
shortcuts were good enough. The env also records how wrong the image
detector is, to check the stand-in's noise model.
"""

from __future__ import annotations

import numpy as np
import torch

from isaaclab.sensors import Camera, CameraCfg
from isaaclab.utils.configclass import configclass

from kaya_lab.assets import KAYA_CFG
from kaya_lab.ball import BALL_RADIUS
from kaya_lab.ball_detector import BallDetector
from kaya_lab.detector import project

from .ball_goal_det_env import KayaBallGoalDetEnv
from .ball_goal_env_cfg import KayaBallGoalDetEnvCfg

RGB_PRIM = "Kaya/base_link/camera_link/gemini_335L/camera_rgb/Camera_rgb"


@configclass
class KayaBallGoalB4EnvCfg(KayaBallGoalDetEnvCfg):
    robot = KAYA_CFG.replace(prim_path="/World/envs/env_.*/Kaya")       # the asset's 32 iterations
    camera: CameraCfg = CameraCfg(
        prim_path=f"/World/envs/env_.*/{RGB_PRIM}",
        spawn=None,                          # use the robot's own camera prim and intrinsics
        data_types=["rgb"],
        width=848,
        height=480,
        update_period=0.0,
    )
    randomize_servos = False
    debug_vis = False                        # markers are rendered INTO the robot's camera image
    initial_level = 1.0
    goal_width_range = (0.46, 0.46)          # the real box

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        # arenas far apart: every camera renders the whole world, and at 5 m spacing it
        # saw the NEIGHBOURS' balls (95% false detections in the first run). At 40 m a
        # neighbour's ball is a few px, far below the detector's minimum blob size.
        self.scene.env_spacing = 40.0
        self.sim.render_interval = self.decimation    # render once per policy step: the camera frame


class KayaBallGoalB4Env(KayaBallGoalDetEnv):
    cfg: KayaBallGoalB4EnvCfg

    def _setup_scene(self):
        super()._setup_scene()
        self.camera = Camera(self.cfg.camera)
        self.scene.sensors["camera"] = self.camera
        self._detectors = None
        self.det_log = []       # per detection: (truth x, y, visible by geometry, ok, est x, y, truncated, rms)

    def _make_detectors(self):
        """K from the rendered camera; the optical frame's pose in base_link from the
        robot's USD (a fixed transform: the camera is bolted on). Not from the camera
        sensor's pose, which Isaac Lab does not refresh after the robot is moved at reset
        (update_latest_camera_pose=False): combining it with the robot's new pose put
        the camera 1.9 m away in the first B4 run."""
        from pxr import UsdGeom

        K = self.camera.data.intrinsic_matrices.torch[0].cpu().numpy()
        stage = self.sim.stage
        xc = UsdGeom.XformCache()
        base = xc.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/envs/env_0/Kaya/base_link"))
        cam = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(f"/World/envs/env_0/{RGB_PRIM}"))
        rel = cam * base.GetInverse()                 # USD row vectors: p_base = p_cam * rel
        M = np.array(rel, dtype=np.float64)
        R_base_usdcam = M[:3, :3].T                   # column-vector rotation
        # a USD camera looks down its -Z with +Y up; the optical frame looks down +Z with +Y down
        R = R_base_usdcam @ np.diag([1.0, -1.0, -1.0])
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, M[3, :3]
        self.cam_T_base_optical, self.cam_K = T, K
        self._detectors = [BallDetector(K, None, T, BALL_RADIUS, BALL_RADIUS - 0.037) for _ in range(self.num_envs)]

    def _detect(self):
        if self._detectors is None:
            self._make_detectors()
        rgb = self.camera.data.output["rgb"].torch[..., :3].to(torch.uint8).cpu().numpy()
        truth_b = self._to_body(self.ball.data.root_pos_w.torch)
        u, v, R, fwd = project(truth_b)
        vis = ((fwd > 0.05) & (u + R > 12) & (u - R < 848 - 12) & (v - R < 480 - 12)).cpu().numpy()
        tb = truth_b.cpu().numpy()
        valid = np.zeros(self.num_envs, dtype=bool)
        meas = np.zeros((self.num_envs, 2), dtype=np.float32)
        for i in range(self.num_envs):
            d = self._detectors[i].detect(rgb[i])
            valid[i] = d.ok
            meas[i] = (d.x, d.y)
            self.det_log.append((tb[i, 0], tb[i, 1], bool(vis[i]), d.ok, d.x, d.y, d.truncated, d.rms_deg))
        dev = self.device
        return torch.from_numpy(valid).to(dev), torch.from_numpy(meas).to(dev)
