"""M1: drive to a point. See point_goal_env_cfg.py for every number.

One policy step (10 Hz):
  _pre_physics_step   action [-1, 1]^3 -> twist (vx, vy, w) -> servo command
  _apply_action       x24 physics steps: the servo chain -> wheel targets
  _get_dones          success / wandered off / timeout
  _get_rewards        progress + success - fail - time - jerk
  _get_observations   goal in the robot frame, body velocity, last action
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.sim.spawners.from_files import spawn_ground_plane
from isaaclab.utils.math import quat_from_euler_xyz, subtract_frame_transforms

from kaya_lab.assets import WHEEL_JOINTS, floor_cfg
from kaya_lab.servos import KayaServos

from .point_goal_env_cfg import KayaPointGoalEnvCfg


class KayaPointGoalEnv(DirectRLEnv):
    cfg: KayaPointGoalEnvCfg

    def __init__(self, cfg: KayaPointGoalEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._wheel_ids, _ = self.robot.find_joints(WHEEL_JOINTS, preserve_order=True)
        self.servos = KayaServos(self.num_envs, self.physics_dt, self.device)
        self._limits = torch.tensor([cfg.max_lin_vel, cfg.max_lin_vel if cfg.holonomic else 0.0, cfg.max_ang_vel],
                                    device=self.device)
        self._actions = torch.zeros(self.num_envs, 3, device=self.device)
        self._prev_actions = torch.zeros_like(self._actions)
        self._goal_w = torch.zeros(self.num_envs, 3, device=self.device)
        self._prev_dist = torch.zeros(self.num_envs, device=self.device)
        self._success = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._failed = torch.zeros_like(self._success)
        self._fell = torch.zeros_like(self._success)
        self._sums = {k: torch.zeros(self.num_envs, device=self.device)
                      for k in ("progress", "success", "fail", "time", "action_rate")}
        self.set_debug_vis(self.cfg.debug_vis)

    # ------------------------------------------------------------------ scene
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)
        span = self.cfg.scene.env_spacing * math.ceil(math.sqrt(self.cfg.scene.num_envs)) + 20.0
        spawn_ground_plane(prim_path="/World/ground", cfg=floor_cfg(size=(span, span)))
        self.scene.clone_environments(copy_from_source=False)
        # robots in different envs must never touch, on GPU as well as CPU
        self.scene.filter_collisions(global_prim_paths=["/World/ground"])
        self.scene.articulations["kaya"] = self.robot
        light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        light.func("/World/light", light)

    # ----------------------------------------------------------------- action
    def _pre_physics_step(self, actions: torch.Tensor):
        self._prev_actions[:] = self._actions
        self._actions[:] = actions.clamp(-1.0, 1.0)
        self.servos.command_twist(self._actions * self._limits)

    def _apply_action(self):
        # once per PHYSICS step: the servo chain advances on the physics clock
        pos, vel = self.servos.step(self.robot.data.joint_pos.torch[:, self._wheel_ids])
        self.robot.set_joint_position_target_index(target=pos, joint_ids=self._wheel_ids)
        self.robot.set_joint_velocity_target_index(target=vel, joint_ids=self._wheel_ids)

    # ------------------------------------------------------------ observation
    def _goal_in_body(self):
        goal_b, _ = subtract_frame_transforms(self.robot.data.root_link_pos_w.torch,
                                              self.robot.data.root_link_quat_w.torch, self._goal_w)
        return goal_b[:, :2]

    def _get_observations(self) -> dict:
        v = self.robot.data.root_link_lin_vel_b.torch
        w = self.robot.data.root_link_ang_vel_b.torch
        obs = torch.cat([
            self._goal_in_body() / self.cfg.goal_radius_range[1],   # roughly [-1, 1]
            v[:, :2] / self.cfg.max_lin_vel,
            w[:, 2:3] / self.cfg.max_ang_vel,
            self._actions,
        ], dim=-1)
        return {"policy": obs}

    # ------------------------------------------------------------ termination
    def _dist(self):
        return torch.linalg.norm(self._goal_in_body(), dim=1)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        d = self._dist()
        speed = torch.linalg.norm(self.robot.data.root_link_lin_vel_b.torch[:, :2], dim=1)
        self._success = (d < self.cfg.success_dist) & (speed < self.cfg.success_speed)
        self._failed = d > self.cfg.fail_dist
        # fell through / off the floor: must never happen; logged so it can't hide
        self._fell = self.robot.data.root_link_pos_w.torch[:, 2] < 0.0
        self._failed |= self._fell
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return self._success | self._failed, time_out

    # ----------------------------------------------------------------- reward
    def _get_rewards(self) -> torch.Tensor:
        d = self._dist()
        # distance closed this step, as a fraction of what full speed would close
        progress = (self._prev_dist - d) / (self.cfg.max_lin_vel * self.step_dt)
        self._prev_dist = d
        r = {
            "progress": self.cfg.rew_progress * progress,
            "success": self.cfg.rew_success * self._success.float(),
            "fail": self.cfg.rew_fail * self._failed.float(),
            "time": torch.full_like(d, self.cfg.rew_time),
            "action_rate": self.cfg.rew_action_rate * (self._actions - self._prev_actions).square().sum(1),
        }
        for k, v in r.items():
            self._sums[k] += v
        return torch.stack(list(r.values())).sum(0)

    # ------------------------------------------------------------------ reset
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = torch.arange(self.num_envs, device=self.device)
        env_ids = torch.as_tensor(env_ids, device=self.device)

        # log how the finished episodes went (only envs that actually ran)
        done = self.episode_length_buf[env_ids] > 0
        if done.any():
            ids = env_ids[done]
            log = {f"Episode_Reward/{k}": v[ids].mean().item() for k, v in self._sums.items()}
            log["Metrics/success_rate"] = self._success[ids].float().mean().item()
            log["Metrics/fell_rate"] = self._fell[ids].float().mean().item()
            log["Metrics/final_distance_m"] = self._dist()[ids].mean().item()
            log["Metrics/episode_time_s"] = (self.episode_length_buf[ids].float() * self.step_dt).mean().item()
            self.extras["log"] = log
        for v in self._sums.values():
            v[env_ids] = 0.0

        self.robot.reset(env_ids)
        super()._reset_idx(env_ids)
        if len(env_ids) == self.num_envs:
            # stagger episodes so all envs don't reset at the same moment
            self.episode_length_buf[:] = torch.randint_like(self.episode_length_buf, int(self.max_episode_length))

        n = len(env_ids)
        origins = self.scene.env_origins[env_ids]
        # robot: at the env origin, random heading
        yaw = (2 * torch.rand(n, device=self.device) - 1) * math.pi
        pose = self.robot.data.default_root_pose.torch[env_ids].clone()
        pose[:, :3] += origins
        pose[:, 3:7] = quat_from_euler_xyz(torch.zeros_like(yaw), torch.zeros_like(yaw), yaw)
        self.robot.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros(n, 6, device=self.device), env_ids=env_ids)
        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        self.robot.write_joint_position_to_sim_index(position=jp, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(jp), env_ids=env_ids)

        # servos hold where the wheels now are; new chain parameters per episode
        self.servos.reset(env_ids, jp[:, self._wheel_ids])
        if self.cfg.randomize_servos:
            self.servos.randomize(env_ids)
        self._actions[env_ids] = 0.0
        self._prev_actions[env_ids] = 0.0

        # goal: uniform direction, radius in goal_radius_range
        r = torch.empty(n, device=self.device).uniform_(*self.cfg.goal_radius_range)
        a = (2 * torch.rand(n, device=self.device) - 1) * math.pi
        self._goal_w[env_ids, 0] = origins[:, 0] + r * torch.cos(a)
        self._goal_w[env_ids, 1] = origins[:, 1] + r * torch.sin(a)
        self._goal_w[env_ids, 2] = origins[:, 2]
        self._prev_dist[env_ids] = r
        self._success[env_ids] = False
        self._failed[env_ids] = False

    # ---------------------------------------------------------- visualisation
    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis and not hasattr(self, "_goal_vis"):
            self._goal_vis = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path="/Visuals/goal",
                markers={
                    "far": sim_utils.CylinderCfg(radius=0.08, height=0.004, visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(1.0, 0.15, 0.1))),
                    "near": sim_utils.CylinderCfg(radius=0.08, height=0.004, visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(0.1, 0.9, 0.2))),
                }))
        if hasattr(self, "_goal_vis"):
            self._goal_vis.set_visibility(debug_vis)

    def _debug_vis_callback(self, event):
        near = (self._dist() < 0.15).long()     # turns green when the robot is close
        self._goal_vis.visualize(translations=self._goal_w + torch.tensor([0, 0, 0.002], device=self.device),
                                 marker_indices=near)
