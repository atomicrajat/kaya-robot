"""Ball v2, "play like a footballer" (SIM_RL_PLAN 2b).

B2 (camera-detector ball, odometry memory, asymmetric critic) moved into a
ROOM:
  * walls on all four sides; the box stays on the +x wall, the other three
    walls move every episode (a 2.4-4.1 m wide room, 3.5-5.1 m deep), so the
    box sits anywhere from the middle of a wall to near a corner of it;
  * 0-2 obstacles (25 cm cubes, 30 cm tall: furniture legs, bins);
  * the ball bounces off walls instead of ending the episode, and the
    curriculum puts it AGAINST walls and IN corners (seen on the real robot
    2026-09-28: a ball in a corner was unrecoverable for the v1 policy).

New observation: a 16-ray horizontal DEPTH SCAN across the camera's 92 deg
view, at camera height (17.5 cm), so it passes over the 14 cm ball and sees
only walls, the box and obstacles. On the robot it is one row of the Gemini's
depth image (the row looking level). Computed analytically here (walls and
boxes are axis-aligned), so it costs nothing in training.

New rewards: a penalty on ball speed above 0.3 m/s (dribble, don't kick), and
on the robot scraping walls or obstacles.
"""

from __future__ import annotations

import math

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg

from kaya_lab.ball import BALL_RADIUS, CARDBOARD_T, GOAL_DEPTH

from .ball_goal_det_env import KayaBallGoalDetEnv
from .ball_goal_env_cfg import KayaBallGoalV2EnvCfg

CAM_B = (0.024, 0.024)          # colour lens in base_link (x, y), the scan's origin
ROBOT_R = 0.156                 # footprint radius incl. wheels (USD bounding box: x -0.156..0.11, y +-0.153)
WALL_T, WALL_H, WALL_L = 0.05, 0.40, 7.0


def _box_cfg(name, size, color):
    return RigidObjectCfg(
        prim_path=f"/World/envs/env_.*/{name}",
        spawn=sim_utils.CuboidCfg(
            size=size,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4,
                                                            restitution=0.3),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=1.0),
        ),
    )


class KayaBallGoalV2Env(KayaBallGoalDetEnv):
    cfg: KayaBallGoalV2EnvCfg

    def __init__(self, cfg: KayaBallGoalV2EnvCfg, render_mode: str | None = None, **kwargs):
        n = cfg.scene.num_envs
        self._room = torch.zeros(n, 4)                      # back_x, right_y (<0), left_y (>0), front_x
        self._obs_xy = torch.zeros(n, cfg.n_obstacles, 2)
        self._obs_on = torch.zeros(n, cfg.n_obstacles, dtype=torch.bool)
        super().__init__(cfg, render_mode, **kwargs)
        dev = self.device
        self._room, self._obs_xy, self._obs_on = self._room.to(dev), self._obs_xy.to(dev), self._obs_on.to(dev)
        self._room[:, 0], self._room[:, 1], self._room[:, 2] = -2.0, -1.5, 1.5
        self._room[:, 3] = cfg.goal_line_x + GOAL_DEPTH + CARDBOARD_T
        for k in ("ball_speed", "bump", "jam", "center_push", "kick"):
            self._sums[k] = torch.zeros(self.num_envs, device=dev)
        self._jam_run = torch.zeros(self.num_envs, device=dev)          # consecutive jammed steps
        self._since_jam = torch.full((self.num_envs,), 99.0, device=dev)  # steps since the last jam
        self._kicked = torch.ones(self.num_envs, dtype=torch.bool, device=dev)
        self._backed = torch.zeros(self.num_envs, dtype=torch.bool, device=dev)  # backed off since the jam
        a = math.radians(cfg.ray_fov_deg / 2)
        self._ray_ang = torch.linspace(a, -a, cfg.n_rays, device=dev)       # left to right, like the image

    # ------------------------------------------------------------------ scene
    def _add_scene_objects(self):
        grey = (0.78, 0.78, 0.75)
        self.walls = [RigidObject(_box_cfg("WallBack", (WALL_T, WALL_L, WALL_H), grey)),
                      RigidObject(_box_cfg("WallRight", (WALL_L, WALL_T, WALL_H), grey)),
                      RigidObject(_box_cfg("WallLeft", (WALL_L, WALL_T, WALL_H), grey))]
        h = self.cfg.obstacle_height
        self.obstacles = [RigidObject(_box_cfg(f"Obstacle{i}", (s, s, h), (0.25, 0.3, 0.45)))
                          for i, s in enumerate(self.cfg.obstacle_sizes[:self.cfg.n_obstacles])]
        for i, o in enumerate(self.walls + self.obstacles):
            self.scene.rigid_objects[f"v2_{i}"] = o

    def _placement_limits(self, env_ids):
        # a new room for these envs, then its inner faces are the limits
        n, dev, cfg = len(env_ids), self.device, self.cfg
        U = lambda lo, hi: lo + (hi - lo) * torch.rand(n, device=dev)     # noqa: E731
        self._room[env_ids, 0] = U(*cfg.room_back_x)
        self._room[env_ids, 1] = -U(*cfg.room_side_y)
        self._room[env_ids, 2] = U(*cfg.room_side_y)
        self._place_walls(env_ids)
        return self._room[env_ids, 0], self._room[env_ids, 1], self._room[env_ids, 2]

    def _place_walls(self, env_ids):
        o = self.scene.env_origins[env_ids]
        r = self._room[env_ids]
        q = torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.device).repeat(len(env_ids), 1)
        mid_x = 0.5 * (r[:, 0] + r[:, 3])
        for wall, (x, y) in zip(self.walls, [(r[:, 0] - WALL_T / 2, torch.zeros_like(mid_x)),
                                             (mid_x, r[:, 1] - WALL_T / 2), (mid_x, r[:, 2] + WALL_T / 2)]):
            pose = torch.cat([torch.stack([o[:, 0] + x, o[:, 1] + y, o[:, 2] + WALL_H / 2], 1), q], 1)
            wall.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)

    def _adjust_ball_start(self, env_ids, bx, by):
        """Some balls start against a wall or in a corner (more as the level rises)."""
        n, dev, cfg = len(env_ids), self.device, self.cfg
        frac = cfg.wall_ball_frac[0] + (cfg.wall_ball_frac[1] - cfg.wall_ball_frac[0]) * self.level
        pick = torch.rand(n, device=dev) < frac
        if not pick.any():
            return bx, by
        r = self._room[env_ids]
        gap = BALL_RADIUS + 0.01
        kind = torch.randint(0, 4, (n,), device=dev)     # 0 left wall, 1 right wall, 2 back wall, 3 corner
        # 4: against the OUTSIDE of a box side panel (v2.1; where the real robot jammed the ball)
        kind = torch.where(torch.rand(n, device=dev) < cfg.side_panel_frac, torch.full_like(kind, 4), kind)
        x_rand = r[:, 0] + gap + (cfg.goal_line_x - 0.2 - r[:, 0] - gap) * torch.rand(n, device=dev)
        y_rand = r[:, 1] + gap + (r[:, 2] - r[:, 1] - 2 * gap) * torch.rand(n, device=dev)
        left = torch.rand(n, device=dev) < 0.5
        y_side = torch.where(left, r[:, 2] - gap, r[:, 1] + gap)
        near_goal = torch.rand(n, device=dev) < 0.5       # corners by the goal wall too (run 4)
        x_corner = torch.where(near_goal, torch.full_like(bx, cfg.goal_line_x - 0.15), r[:, 0] + gap)
        nbx = torch.where(kind == 2, r[:, 0] + gap, torch.where(kind == 3, x_corner, x_rand))
        nby = torch.where(kind == 0, r[:, 2] - gap, torch.where(kind == 1, r[:, 1] + gap,
                          torch.where(kind == 3, y_side, y_rand)))
        hw = self._half_w[env_ids]
        from kaya_lab.ball import CARDBOARD_T as T
        panel_y = torch.where(left, hw + T + BALL_RADIUS + 0.01, -(hw + T + BALL_RADIUS + 0.01))
        panel_y = torch.maximum(torch.minimum(panel_y, r[:, 2] - gap), r[:, 1] + gap)
        panel_x = cfg.goal_line_x + 0.05 + 0.2 * torch.rand(n, device=dev)
        nbx = torch.where(kind == 4, panel_x, nbx)
        nby = torch.where(kind == 4, panel_y, nby)
        return torch.where(pick, nbx, bx), torch.where(pick, nby, by)

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        env_ids = torch.as_tensor(env_ids, device=self.device)
        self._place_obstacles(env_ids)
        self._jam_run[env_ids] = 0.0
        self._since_jam[env_ids] = 99.0
        self._kicked[env_ids] = True
        self._backed[env_ids] = False

    def _place_obstacles(self, env_ids):
        cfg, dev, n = self.cfg, self.device, len(env_ids)
        ids, bx, by, rx, ry = self._last_starts
        r = self._room[env_ids]
        o = self.scene.env_origins[env_ids]
        q = torch.tensor([1.0, 0.0, 0.0, 0.0], device=dev).repeat(n, 1)
        L = self.level
        prob = cfg.obstacle_prob[0] + (cfg.obstacle_prob[1] - cfg.obstacle_prob[0]) * L
        for k, obst in enumerate(self.obstacles):
            half = cfg.obstacle_sizes[k] / 2
            # rejection sampling, vectorised: 8 draws per env, keep the first that is clear of
            # the ball, the robot and earlier obstacles (and away from the goal mouth: x < goal - 0.6)
            best, good = None, None
            for _ in range(8):
                x = r[:, 0] + half + 0.05 + (cfg.goal_line_x - 0.6 - r[:, 0] - half) * torch.rand(n, device=dev)
                y = r[:, 1] + half + 0.05 + (r[:, 2] - r[:, 1] - 2 * half - 0.1) * torch.rand(n, device=dev)
                ok = (torch.hypot(x - bx, y - by) > 0.45) & (torch.hypot(x - rx, y - ry) > 0.55)
                for j in range(k):
                    ok &= ~self._obs_on[env_ids, j] | \
                        (torch.hypot(x - self._obs_xy[env_ids, j, 0], y - self._obs_xy[env_ids, j, 1]) > 0.5)
                cand = torch.stack([x, y], 1)
                if best is None:
                    best, good = cand, ok
                else:
                    best = torch.where((~good & ok)[:, None], cand, best)
                    good = good | ok
            # an obstacle that found no clear spot is simply left out this episode
            present = good & (torch.rand(n, device=dev) < prob)
            self._obs_xy[env_ids, k] = best
            self._obs_on[env_ids, k] = present
            z = torch.where(present, torch.full_like(x, cfg.obstacle_height / 2), torch.full_like(x, -1.0))
            pose = torch.cat([torch.stack([o[:, 0] + best[:, 0], o[:, 1] + best[:, 1], o[:, 2] + z], 1), q], 1)
            obst.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)

    # ------------------------------------------------------------- the scan
    def _scan(self):
        """Distances (N, n_rays) from the lens, level, to walls / the box / obstacles."""
        cfg = self.cfg
        p = self._robot_env()[:, :2]
        q = self.robot.data.root_link_quat_w.torch
        yaw = torch.atan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))
        c, s = torch.cos(yaw), torch.sin(yaw)
        ox = p[:, 0] + c * CAM_B[0] - s * CAM_B[1]
        oy = p[:, 1] + s * CAM_B[0] + c * CAM_B[1]
        ang = yaw[:, None] + self._ray_ang[None]
        dx, dy = torch.cos(ang), torch.sin(ang)
        big = torch.full_like(dx, cfg.ray_max)
        eps = 1e-6
        r = self._room
        t = big
        # the four walls (inner faces); a ray only hits the wall it is heading towards
        for wall_x, sgn in ((r[:, 3], 1), (r[:, 0], -1)):
            tt = (wall_x[:, None] - ox[:, None]) / torch.where(dx.abs() < eps, torch.full_like(dx, eps), dx)
            t = torch.where((tt > 0) & (sgn * dx > 0), torch.minimum(t, tt), t)
        for wall_y, sgn in ((r[:, 2], 1), (r[:, 1], -1)):
            tt = (wall_y[:, None] - oy[:, None]) / torch.where(dy.abs() < eps, torch.full_like(dy, eps), dy)
            t = torch.where((tt > 0) & (sgn * dy > 0), torch.minimum(t, tt), t)
        # boxes (slab test): the goal's side panels and the obstacles
        hw = self._half_w
        boxes = [(torch.full_like(hw, cfg.goal_line_x), torch.full_like(hw, cfg.goal_line_x + GOAL_DEPTH),
                  hw, hw + CARDBOARD_T, torch.ones_like(hw, dtype=torch.bool)),
                 (torch.full_like(hw, cfg.goal_line_x), torch.full_like(hw, cfg.goal_line_x + GOAL_DEPTH),
                  -hw - CARDBOARD_T, -hw, torch.ones_like(hw, dtype=torch.bool))]
        for k in range(cfg.n_obstacles):
            h = cfg.obstacle_sizes[k] / 2
            cx, cy = self._obs_xy[:, k, 0], self._obs_xy[:, k, 1]
            boxes.append((cx - h, cx + h, cy - h, cy + h, self._obs_on[:, k]))
        for x0, x1, y0, y1, on in boxes:
            idx = 1.0 / torch.where(dx.abs() < eps, torch.full_like(dx, eps), dx)
            idy = 1.0 / torch.where(dy.abs() < eps, torch.full_like(dy, eps), dy)
            tx0, tx1 = (x0[:, None] - ox[:, None]) * idx, (x1[:, None] - ox[:, None]) * idx
            ty0, ty1 = (y0[:, None] - oy[:, None]) * idy, (y1[:, None] - oy[:, None]) * idy
            tmin = torch.maximum(torch.minimum(tx0, tx1), torch.minimum(ty0, ty1))
            tmax = torch.minimum(torch.maximum(tx0, tx1), torch.maximum(ty0, ty1))
            hit = on[:, None] & (tmax >= tmin) & (tmin > 0)
            t = torch.where(hit, torch.minimum(t, tmin), t)
        t = t * (1 + cfg.ray_noise * torch.randn_like(t))
        return t.clamp(0.0, cfg.ray_max)

    # ---------------------------------------------------------- obs, rewards
    def _get_observations(self) -> dict:
        obs = super()._get_observations()
        scan = self._scan() / self.cfg.ray_max
        policy = torch.cat([obs["policy"], scan], dim=-1)
        return {"policy": policy, "critic": torch.cat([policy, obs["critic"][:, 15:]], dim=-1)}

    def _clearance(self):
        """Robot centre's distance to the nearest wall or obstacle face, minus its radius."""
        p, r = self._robot_env()[:, :2], self._room
        d = torch.stack([p[:, 0] - r[:, 0], r[:, 3] - p[:, 0], p[:, 1] - r[:, 1], r[:, 2] - p[:, 1]], 1).min(1).values
        for k in range(self.cfg.n_obstacles):
            h = self.cfg.obstacle_sizes[k] / 2
            ex = ((p[:, 0] - self._obs_xy[:, k, 0]).abs() - h).clamp(min=0)
            ey = ((p[:, 1] - self._obs_xy[:, k, 1]).abs() - h).clamp(min=0)
            dk = torch.where(self._obs_on[:, k], torch.hypot(ex, ey), torch.full_like(ex, 9.0))
            d = torch.minimum(d, dk)
        return d - ROBOT_R

    def _ball_clearance(self):
        """Ball surface to the nearest wall, obstacle or goal side panel [m]."""
        cfg = self.cfg
        b, r = self._ball_env()[:, :2], self._room
        d = torch.stack([b[:, 0] - r[:, 0], r[:, 3] - b[:, 0], b[:, 1] - r[:, 1], r[:, 2] - b[:, 1]], 1).min(1).values
        boxes = [(self._obs_xy[:, k, 0], self._obs_xy[:, k, 1], cfg.obstacle_sizes[k] / 2, cfg.obstacle_sizes[k] / 2,
                  self._obs_on[:, k]) for k in range(cfg.n_obstacles)]
        from kaya_lab.ball import CARDBOARD_T as T
        hw = self._half_w
        for sgn in (1.0, -1.0):          # the goal's side panels, as boxes
            boxes.append((torch.full_like(hw, cfg.goal_line_x + GOAL_DEPTH / 2), sgn * (hw + T / 2),
                          GOAL_DEPTH / 2, T / 2, torch.ones_like(hw, dtype=torch.bool)))
        for cx, cy, hx, hy, on in boxes:
            ex = ((b[:, 0] - cx).abs() - hx).clamp(min=0)
            ey = ((b[:, 1] - cy).abs() - hy).clamp(min=0)
            d = torch.minimum(d, torch.where(on, torch.hypot(ex, ey), torch.full_like(ex, 9.0)))
        return d - BALL_RADIUS

    def _get_rewards(self) -> torch.Tensor:
        rew = super()._get_rewards()
        cfg = self.cfg
        v = torch.linalg.norm(self.ball.data.root_lin_vel_w.torch[:, :2], dim=1)
        # jammed: the robot is pushing (forward) on a ball that is pressed against something
        # and not moving - the real robot's failure in runs 12b/12c
        jam = (self._in_pocket() & (self._ball_clearance() < 0.02) & (v < cfg.jam_ball_speed)
               & (self._actions[:, 0] > 0.2))
        # jam bookkeeping (v2.3). The user's recipe for a pinned ball: "step back, then strike it
        # with the right or left wheel as a kick, then come back and see the next move". So a kick
        # is rewarded once per jam, and only when, within kick_window steps of the jam ending,
        # the robot first BACKED OFF (drove backwards) and the ball then moves off after being
        # struck OFF-CENTRE (sitting in front of a front wheel, not the middle of the pocket).
        ball_b = self._to_body(self.ball.data.root_pos_w.torch)
        self._jam_run = torch.where(jam, self._jam_run + 1, torch.zeros_like(self._jam_run))
        self._since_jam = torch.where(jam, torch.zeros_like(self._since_jam), self._since_jam + 1)
        self._kicked = torch.where(jam, torch.zeros_like(self._kicked), self._kicked)
        self._backed = torch.where(jam, torch.zeros_like(self._backed),
                                   self._backed | (self._actions[:, 0] < -0.2))
        # the ball just in front (it is already leaving the pocket as it is struck), to one side
        off_centre = ((ball_b[:, 0] > 0.0) & (ball_b[:, 0] < cfg.pocket_x[1] + 0.05)
                      & (ball_b[:, 1].abs() >= cfg.kick_min_y) & (ball_b[:, 1].abs() < 0.20))
        kick = (~self._kicked & self._backed & off_centre & (self._since_jam > 0)
                & (self._since_jam <= cfg.kick_window) & (v > cfg.kick_speed))
        self._kicked = self._kicked | kick
        # square-on push into a surface: the ball centred in the pocket, next to a wall / obstacle
        # / box panel, and the robot driving forward (the real robot jams it this way)
        center_push = (self._in_pocket() & (ball_b[:, 1].abs() < cfg.center_push_half_y)
                       & (self._ball_clearance() < cfg.center_push_clear) & (self._actions[:, 0] > 0.3))
        extra = {
            "ball_speed": cfg.rew_ball_speed * (v - cfg.ball_speed_free).clamp(min=0.0),
            "bump": cfg.rew_bump * (self._clearance() < cfg.bump_dist).float(),
            "jam": cfg.rew_jam * jam.float() * (1.0 + cfg.jam_escalate * self._jam_run),
            "center_push": cfg.rew_center_push * center_push.float(),
            "kick": cfg.rew_kick * kick.float(),
        }
        for k, x in extra.items():
            self._sums[k] += x
        return rew + sum(extra.values())
