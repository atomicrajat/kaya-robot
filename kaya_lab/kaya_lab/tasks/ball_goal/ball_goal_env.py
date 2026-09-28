"""B1: push the ball into the goal. Numbers live in ball_goal_env_cfg.py.

Per policy step (10 Hz): twist action -> servo chain (x24 physics steps, with
rolling resistance on the ball every physics step) -> dones -> rewards ->
observations. The teacher sees the TRUE ball state; B2 will replace it with
a simulated camera detector.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import spawn_ground_plane
from isaaclab.utils.math import quat_apply_inverse, quat_from_euler_xyz, subtract_frame_transforms, yaw_quat

from kaya_lab.assets import WHEEL_JOINTS, floor_cfg
from kaya_lab.ball import (BALL_RADIUS, CARDBOARD_T, GOAL_DEPTH, GOAL_HEIGHT, post_cfg, rolling_resistance, scored,
                           set_hollow_inertia, spawn_goal)
from kaya_lab.servos import KayaServos

from .ball_goal_env_cfg import KayaBallGoalEnvCfg


def lerp(r, t):
    return r[0] + (r[1] - r[0]) * t


class KayaBallGoalEnv(DirectRLEnv):
    cfg: KayaBallGoalEnvCfg

    def __init__(self, cfg: KayaBallGoalEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self._wheel_ids, _ = self.robot.find_joints(WHEEL_JOINTS, preserve_order=True)
        self.servos = KayaServos(n, self.physics_dt, dev)
        self._limits = torch.tensor([cfg.max_lin_vel, cfg.max_lin_vel if cfg.holonomic else 0.0, cfg.max_ang_vel],
                                    device=dev)
        self._actions = torch.zeros(n, 3, device=dev)
        self._prev_actions = torch.zeros_like(self._actions)
        self._roll_c = torch.zeros(n, device=dev)
        self._half_w = torch.full((n,), cfg.goal_width_range[1] / 2, device=dev)   # this episode's goal half-width
        self._prev_d_rb = torch.zeros(n, device=dev)
        self._prev_d_bg = torch.zeros(n, device=dev)
        self._success = torch.zeros(n, dtype=torch.bool, device=dev)
        self._failed = torch.zeros_like(self._success)
        self._possess_steps = torch.zeros(n, device=dev)
        self.level = cfg.initial_level                 # curriculum level, 0..1 (global)
        self._succ_ema = 0.0
        self._sums = {k: torch.zeros(n, device=dev) for k in
                      ("approach", "ball_progress", "possession", "success", "fail", "time", "action_rate")}
        # the goal direction (+x of the env frame) is the same in every env
        self._goal_dir_w = torch.tensor([1.0, 0.0, 0.0], device=dev).repeat(n, 1)

    # ------------------------------------------------------------------ scene
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)
        self.ball = RigidObject(self.cfg.ball)
        self.posts = [RigidObject(post_cfg("PostLeft")), RigidObject(post_cfg("PostRight"))]
        set_hollow_inertia(self.sim.stage, "/World/envs/env_0/Ball")
        spawn_goal("/World/envs/env_0", self.cfg.goal_line_x)
        span = self.cfg.scene.env_spacing * math.ceil(math.sqrt(self.cfg.scene.num_envs)) + 20.0
        spawn_ground_plane(prim_path="/World/ground", cfg=floor_cfg(size=(span, span)))
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=["/World/ground"])
        self.scene.articulations["kaya"] = self.robot
        self.scene.rigid_objects["ball"] = self.ball
        self.scene.rigid_objects["post_left"] = self.posts[0]
        self.scene.rigid_objects["post_right"] = self.posts[1]
        light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.85, 0.85, 0.85))
        light.func("/World/light", light)

    # ----------------------------------------------------------------- action
    def _pre_physics_step(self, actions: torch.Tensor):
        self._prev_actions[:] = self._actions
        self._actions[:] = actions.clamp(-1.0, 1.0)
        self.servos.command_twist(self._actions * self._limits)

    def _apply_action(self):
        pos, vel = self.servos.step(self.robot.data.joint_pos.torch[:, self._wheel_ids])
        self.robot.set_joint_position_target_index(target=pos, joint_ids=self._wheel_ids)
        self.robot.set_joint_velocity_target_index(target=vel, joint_ids=self._wheel_ids)
        f = rolling_resistance(self.ball.data.root_lin_vel_w.torch, self._roll_c)
        self.ball.permanent_wrench_composer.set_forces_and_torques_index(forces=f[:, None, :], is_global=True)

    # -------------------------------------------------------------- geometry
    def _ball_env(self):
        return self.ball.data.root_pos_w.torch - self.scene.env_origins

    def _robot_env(self):
        return self.robot.data.root_link_pos_w.torch - self.scene.env_origins

    def _to_body(self, p_w):
        b, _ = subtract_frame_transforms(self.robot.data.root_link_pos_w.torch,
                                         self.robot.data.root_link_quat_w.torch, p_w)
        return b

    def _goal_point_env(self):
        """Nearest point of the goal mouth to the ball (the ball may go in anywhere)."""
        b = self._ball_env()
        half = (self._half_w - BALL_RADIUS).clamp(min=0.0)
        g = torch.zeros_like(b)
        g[:, 0] = self.cfg.goal_line_x + BALL_RADIUS     # a radius inside the mouth
        g[:, 1] = torch.maximum(torch.minimum(b[:, 1], half), -half)
        return g

    def _distances(self):
        rb = torch.linalg.norm(self._robot_env()[:, :2] - self._ball_env()[:, :2], dim=1)
        d_rb = (rb - self.cfg.touch_dist).clamp(min=0.0)
        d_bg = torch.linalg.norm(self._goal_point_env()[:, :2] - self._ball_env()[:, :2], dim=1)
        return d_rb, d_bg

    def _in_pocket(self):
        b = self._to_body(self.ball.data.root_pos_w.torch)
        return (b[:, 0] > self.cfg.pocket_x[0]) & (b[:, 0] < self.cfg.pocket_x[1]) & \
               (b[:, 1].abs() < self.cfg.pocket_half_y)

    # ------------------------------------------------------------ observation
    def _get_observations(self) -> dict:
        q = yaw_quat(self.robot.data.root_link_quat_w.torch)
        ball_b = self._to_body(self.ball.data.root_pos_w.torch)[:, :2]
        ball_v_b = quat_apply_inverse(q, self.ball.data.root_lin_vel_w.torch)[:, :2]
        goal_b = self._to_body(self._goal_point_env() + self.scene.env_origins)[:, :2]
        goal_dir_b = quat_apply_inverse(q, self._goal_dir_w)[:, :2]
        v = self.robot.data.root_link_lin_vel_b.torch
        w = self.robot.data.root_link_ang_vel_b.torch
        obs = torch.cat([
            ball_b / 2.0,
            ball_v_b / 0.5,
            goal_b / 3.0,
            goal_dir_b,
            (self._half_w / 0.23)[:, None],         # goal half-width (1.0 = the real 46 cm box)
            v[:, :2] / self.cfg.max_lin_vel,
            w[:, 2:3] / self.cfg.max_ang_vel,
            self._actions,
        ], dim=-1)
        return {"policy": obs}

    # ------------------------------------------------------------ termination
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        b, r = self._ball_env(), self._robot_env()
        self._success = scored(b, self.cfg.goal_line_x, self._half_w)
        ax, ay = self.cfg.arena_x, self.cfg.arena_y

        def out(p):
            return (p[:, 0] < ax[0]) | (p[:, 0] > ax[1]) | (p[:, 1] < ay[0]) | (p[:, 1] > ay[1]) | (p[:, 2] < -0.05)
        self._failed = (out(b) | out(r)) & ~self._success
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return self._success | self._failed, time_out

    # ----------------------------------------------------------------- reward
    def _get_rewards(self) -> torch.Tensor:
        d_rb, d_bg = self._distances()
        full = self.cfg.max_lin_vel * self.step_dt
        pocket = self._in_pocket()
        self._possess_steps += pocket.float()
        r = {
            "approach": self.cfg.rew_approach * (self._prev_d_rb - d_rb) / full,
            "ball_progress": self.cfg.rew_ball_progress * (self._prev_d_bg - d_bg) / full,
            "possession": self.cfg.rew_possession * pocket.float(),
            "success": self.cfg.rew_success * self._success.float(),
            "fail": self.cfg.rew_fail * self._failed.float(),
            "time": torch.full_like(d_rb, self.cfg.rew_time),
            "action_rate": self.cfg.rew_action_rate * (self._actions - self._prev_actions).square().sum(1),
        }
        self._prev_d_rb, self._prev_d_bg = d_rb, d_bg
        for k, v in r.items():
            self._sums[k] += v
        return torch.stack(list(r.values())).sum(0)

    # ------------------------------------------------------------------ reset
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = torch.arange(self.num_envs, device=self.device)
        env_ids = torch.as_tensor(env_ids, device=self.device)
        cfg, dev = self.cfg, self.device

        done = self.episode_length_buf[env_ids] > 0
        if done.any():
            ids = env_ids[done]
            succ = self._success[ids].float().mean().item()
            # curriculum: a slow average of success decides the level
            self._succ_ema = 0.95 * self._succ_ema + 0.05 * succ
            if self._succ_ema > cfg.curriculum_up:
                self.level = min(1.0, self.level + cfg.curriculum_step)
                self._succ_ema = 0.5
            elif self._succ_ema < cfg.curriculum_down and self.level > 0.0:
                self.level = max(0.0, self.level - cfg.curriculum_step)
                self._succ_ema = 0.5
            _, d_bg = self._distances()
            log = {f"Episode_Reward/{k}": v[ids].mean().item() for k, v in self._sums.items()}
            log.update({
                "Metrics/success_rate": succ,
                "Metrics/fail_rate": self._failed[ids].float().mean().item(),
                "Metrics/final_ball_goal_dist_m": d_bg[ids].mean().item(),
                "Metrics/possession_frac": (self._possess_steps[ids] / self.episode_length_buf[ids].float()).mean().item(),
                "Metrics/episode_time_s": (self.episode_length_buf[ids].float() * self.step_dt).mean().item(),
                "Curriculum/level": self.level,
            })
            self.extras["log"] = log
        for v in self._sums.values():
            v[env_ids] = 0.0
        self._possess_steps[env_ids] = 0.0

        self.robot.reset(env_ids)
        self.ball.reset(env_ids)
        super()._reset_idx(env_ids)
        if len(env_ids) == self.num_envs:
            self.episode_length_buf[:] = torch.randint_like(self.episode_length_buf, int(self.max_episode_length))

        n = len(env_ids)
        L = self.level

        def U(lo, hi):
            return lo + (hi - lo) * torch.rand(n, device=dev)

        origins = self.scene.env_origins[env_ids]
        # ball: in front of the goal
        bx = cfg.goal_line_x - U(cfg.ball_goal_dist[0], lerp(cfg.ball_goal_dist, L))
        hw = lerp(cfg.ball_half_width, L)
        by = U(-hw, hw)
        bx = bx.clamp(cfg.arena_x[0] + 0.3, cfg.goal_line_x - 0.2)
        # robot: behind the ball (away from the goal), rotated off that line by up to +-side_angle
        g = torch.stack([torch.full_like(bx, cfg.goal_line_x), torch.zeros_like(by)], 1)
        away = torch.atan2(by - g[:, 1], bx - g[:, 0])                 # direction goal -> ball
        side = math.radians(lerp(cfg.robot_side_angle_deg, L))
        ang = away + U(-side, side)
        dist = U(cfg.robot_ball_dist[0], lerp(cfg.robot_ball_dist, L))
        rx = (bx + dist * torch.cos(ang)).clamp(cfg.arena_x[0] + 0.3, cfg.goal_line_x - 0.35)
        ry = (by + dist * torch.sin(ang)).clamp(cfg.arena_y[0] + 0.3, cfg.arena_y[1] - 0.3)
        # after clamping, keep at least 0.30 m between robot and ball centres
        dx, dy = rx - bx, ry - by
        d = torch.sqrt(dx**2 + dy**2).clamp(min=1e-6)
        too_close = d < 0.30
        rx = torch.where(too_close, bx - 0.30, rx)
        ry = torch.where(too_close, by, ry)
        # heading: towards the ball, plus noise
        noise = math.radians(lerp(cfg.robot_heading_noise_deg, L))
        yaw = torch.atan2(by - ry, bx - rx) + U(-noise, noise)

        pose = self.robot.data.default_root_pose.torch[env_ids].clone()
        pose[:, 0] = origins[:, 0] + rx
        pose[:, 1] = origins[:, 1] + ry
        pose[:, 2] = origins[:, 2] + 0.002
        pose[:, 3:7] = quat_from_euler_xyz(torch.zeros_like(yaw), torch.zeros_like(yaw), yaw)
        self.robot.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        self.robot.write_joint_position_to_sim_index(position=jp, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(jp), env_ids=env_ids)

        bpose = self.ball.data.default_root_pose.torch[env_ids].clone()
        bpose[:, 0] = origins[:, 0] + bx
        bpose[:, 1] = origins[:, 1] + by
        bpose[:, 2] = origins[:, 2] + BALL_RADIUS + 0.001
        self.ball.write_root_pose_to_sim_index(root_pose=bpose, env_ids=env_ids)
        self.ball.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self._roll_c[env_ids] = U(*cfg.ball_roll_c)

        # goal width for this episode; the side panels move to match
        self._half_w[env_ids] = U(*cfg.goal_width_range) / 2
        for sign, post in ((1.0, self.posts[0]), (-1.0, self.posts[1])):
            ppose = post.data.default_root_pose.torch[env_ids].clone()
            ppose[:, 0] = origins[:, 0] + cfg.goal_line_x + GOAL_DEPTH / 2
            ppose[:, 1] = origins[:, 1] + sign * (self._half_w[env_ids] + CARDBOARD_T / 2)
            ppose[:, 2] = origins[:, 2] + GOAL_HEIGHT / 2
            ppose[:, 3:7] = torch.tensor([1.0, 0.0, 0.0, 0.0], device=dev)
            post.write_root_pose_to_sim_index(root_pose=ppose, env_ids=env_ids)

        self.servos.reset(env_ids, jp[:, self._wheel_ids])
        if cfg.randomize_servos:
            self.servos.randomize(env_ids)
        self._actions[env_ids] = 0.0
        self._prev_actions[env_ids] = 0.0
        self._success[env_ids] = False
        self._failed[env_ids] = False
        # progress terms start from the new positions
        rb = torch.sqrt((rx - bx) ** 2 + (ry - by) ** 2)
        self._prev_d_rb[env_ids] = (rb - cfg.touch_dist).clamp(min=0.0)
        half = (self._half_w[env_ids] - BALL_RADIUS).clamp(min=0.0)
        by_in = torch.maximum(torch.minimum(by, half), -half)
        self._prev_d_bg[env_ids] = torch.sqrt((cfg.goal_line_x + BALL_RADIUS - bx) ** 2 + (by - by_in) ** 2)
