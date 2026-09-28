"""B2: the ball task seen through the (simulated) camera detector.

Same task, rewards and curriculum as B1. Only what the POLICY sees changes:

  B1 (teacher)                          B2 (this)
  true ball position, always            last DETECTED position, 100 ms late,
                                        noisy, only when in the camera's view
  true ball velocity                    - (not measured by the detector)
                                        + how long since the ball was last seen
                                        + whether it is seen right now

Memory: when the detector sees the ball, its position is stored in the odom
(world) frame; every step it is re-expressed in the robot's current frame,
so the estimate stays put while the robot turns or drives away. The real
robot can do exactly this with its odometry. "Age" tells the policy how
stale the estimate is: after the ball rolls away, the memory is wrong and
the policy must learn to go and look.

The CRITIC (training only) additionally gets the true ball position and
velocity: an asymmetric actor-critic. Better value estimates speed up
learning, and the critic is thrown away at deployment.
"""

from __future__ import annotations

from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.math import quat_apply, quat_apply_inverse, yaw_quat

from kaya_lab.ball import BALL_RADIUS
from kaya_lab.detector import detect

from .ball_goal_env import KayaBallGoalEnv
from .ball_goal_env_cfg import KayaBallGoalDetEnvCfg

AGE_CAP_S = 5.0      # "never seen" / "seen long ago" saturate here


class KayaBallGoalDetEnv(KayaBallGoalEnv):
    cfg: KayaBallGoalDetEnvCfg

    def __init__(self, cfg: KayaBallGoalDetEnvCfg, render_mode: str | None = None, **kwargs):
        n = cfg.scene.num_envs
        super().__init__(cfg, render_mode, **kwargs)
        dev = self.device
        self._est_w = torch.zeros(n, 3, device=dev)        # remembered ball position (world)
        self._age = torch.full((n,), AGE_CAP_S, device=dev)
        self._pending_valid = torch.zeros(n, dtype=torch.bool, device=dev)   # 100 ms latency buffer
        self._pending_w = torch.zeros(n, 3, device=dev)
        self._seen_now = torch.zeros(n, dtype=torch.bool, device=dev)
        self._ever_seen = torch.zeros(n, dtype=torch.bool, device=dev)
        self._seen_steps = torch.zeros(n, device=dev)
        self.set_debug_vis(self.cfg.debug_vis)

    def _detect(self):
        """The detector: (valid (N,), ball xy in base_link (N, 2)). Training uses the
        fast stand-in (detector.py); B4 overrides this with the image detector."""
        valid, meas_b, _ = detect(self._to_body(self.ball.data.root_pos_w.torch))
        return valid, meas_b

    def _detector_step(self):
        """Apply last step's detection (latency), then run the detector now."""
        v = self._pending_valid
        self._est_w[v] = self._pending_w[v]
        self._age = torch.where(v, torch.zeros_like(self._age), (self._age + self.step_dt).clamp(max=AGE_CAP_S))
        self._seen_now = v
        self._seen_steps += v.float()
        self._ever_seen |= v
        # detect now, in base_link; store in world for next step
        valid, meas_b = self._detect()
        q = self.robot.data.root_link_quat_w.torch
        meas3 = torch.cat([meas_b, torch.full_like(meas_b[:, :1], BALL_RADIUS - 0.037)], 1)
        self._pending_w = self.robot.data.root_link_pos_w.torch + quat_apply(q, meas3)
        self._pending_valid = valid

    def _get_observations(self) -> dict:
        self._detector_step()
        q = yaw_quat(self.robot.data.root_link_quat_w.torch)
        est_b = self._to_body(self._est_w)[:, :2]
        est_b = torch.where(self._ever_seen[:, None], est_b, torch.zeros_like(est_b))
        goal_b = self._to_body(self._goal_point_env() + self.scene.env_origins)[:, :2]
        goal_dir_b = quat_apply_inverse(q, self._goal_dir_w)[:, :2]
        v = self.robot.data.root_link_lin_vel_b.torch
        w = self.robot.data.root_link_ang_vel_b.torch
        policy = torch.cat([
            est_b / 2.0,
            (self._age / AGE_CAP_S)[:, None],
            self._seen_now.float()[:, None],
            goal_b / 3.0,
            goal_dir_b,
            (self._half_w / 0.23)[:, None],
            v[:, :2] / self.cfg.max_lin_vel,
            w[:, 2:3] / self.cfg.max_ang_vel,
            self._actions,
        ], dim=-1)
        ball_b = self._to_body(self.ball.data.root_pos_w.torch)[:, :2]
        ball_v_b = quat_apply_inverse(q, self.ball.data.root_lin_vel_w.torch)[:, :2]
        critic = torch.cat([policy, ball_b / 2.0, ball_v_b / 0.5], dim=-1)
        return {"policy": policy, "critic": critic}

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        env_ids = torch.as_tensor(env_ids, device=self.device)
        # how much of each finished episode the ball was in view (read before the reset)
        done = self.episode_length_buf[env_ids] > 0
        seen_frac = None
        if done.any():
            ids = env_ids[done]
            seen_frac = (self._seen_steps[ids] / self.episode_length_buf[ids].float()).mean().item()
        super()._reset_idx(env_ids)
        if seen_frac is not None and "log" in self.extras:
            self.extras["log"]["Metrics/ball_seen_frac"] = seen_frac
        self._seen_steps[env_ids] = 0.0
        self._age[env_ids] = AGE_CAP_S
        self._est_w[env_ids] = 0.0
        self._pending_valid[env_ids] = False
        self._seen_now[env_ids] = False
        self._ever_seen[env_ids] = False

    # ---------------------------------------------------------- visualisation
    def _set_debug_vis_impl(self, debug_vis: bool):
        """A small disc where the policy BELIEVES the ball is (its remembered
        detection): green = seen right now, red = working from memory."""
        if debug_vis and not hasattr(self, "_belief_vis"):
            def disc(rgb):
                return sim_utils.CylinderCfg(radius=0.05, height=0.004,
                                             visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=rgb))
            self._belief_vis = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path="/Visuals/ball_belief",
                markers={"memory": disc((0.9, 0.1, 0.1)), "seen": disc((0.1, 0.9, 0.2))}))
        if hasattr(self, "_belief_vis"):
            self._belief_vis.set_visibility(debug_vis)

    def _debug_vis_callback(self, event):
        # the callback is registered by the base class, so it can fire before this env's
        # buffers exist (during __init__) or after close() has torn the scene down
        if not hasattr(self, "_est_w") or not hasattr(self, "scene"):
            return
        p = self._est_w.clone()
        p[:, 2] = self.scene.env_origins[:, 2] + 0.003
        # never seen yet: park the marker under the floor
        p[~self._ever_seen, 2] = -1.0
        self._belief_vis.visualize(translations=p, marker_indices=self._seen_now.long())
