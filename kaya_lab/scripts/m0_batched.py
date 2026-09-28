"""M0: many Kayas at once on GPU physics - batched checks + throughput.

    cd ~/IsaacLab
    ./isaaclab.sh -p ~/work_projects/kaya_robot/kaya/kaya_lab/scripts/m0_batched.py --num_envs 256 --headless

Checks (the batched form of kaya_sim.py --test), every env at once:
  mass           sum of link masses == 2.088 kg in every env
  rest height    base_link settles at the wheel radius, 37 mm, level
  drive          a third of the envs go forward 0.10 m/s, a third strafe left
                 0.10 m/s, a third rotate 0.5 rad/s, through the servo chain;
                 measured body velocity must match within 5%
Then it times free-running physics with random twists (all envs moving),
and prints one JSON line with the throughput.
"""

import argparse
import json
import sys
import time
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--env_spacing", type=float, default=2.0)
parser.add_argument("--timed_steps", type=int, default=1200, help="physics steps to time (240 = 1 s of sim)")
parser.add_argument("--no_checks", action="store_true")
parser.add_argument("--pos_iters", type=int, default=None, help="override the USD's 32 solver position iterations")
parser.add_argument("--profile", action="store_true", help="time each part of a physics step")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils.configclass import configclass  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kaya_lab.assets import KAYA_CFG, MASS_KG, PHYSICS_HZ, WHEEL_JOINTS, floor_cfg  # noqa: E402
from kaya_lab.servos import WHEEL_R, KayaServos  # noqa: E402


@configclass
class KayaSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=floor_cfg())
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))
    kaya: ArticulationCfg = KAYA_CFG.replace(prim_path="{ENV_REGEX_NS}/Kaya")


def kaya_cfg():
    if args.pos_iters is None:
        return KAYA_CFG
    spawn = KAYA_CFG.spawn.replace(articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        solver_position_iteration_count=args.pos_iters))
    return KAYA_CFG.replace(spawn=spawn)


def main():
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1.0 / PHYSICS_HZ, device=args.device))
    scene_cfg = KayaSceneCfg(num_envs=args.num_envs, env_spacing=args.env_spacing)
    scene_cfg.kaya = kaya_cfg().replace(prim_path="{ENV_REGEX_NS}/Kaya")
    scene = InteractiveScene(scene_cfg)
    t0 = time.time()
    sim.reset()
    print(f"[m0] scene of {args.num_envs} built and reset in {time.time() - t0:.1f} s")
    robot = scene["kaya"]
    n, dev, dt = args.num_envs, sim.device, sim.get_physics_dt()
    wheel_ids, names = robot.find_joints(WHEEL_JOINTS, preserve_order=True)
    print(f"[m0] {robot.num_bodies} bodies, {robot.num_joints} joints; wheels {names} -> {wheel_ids}")
    servos = KayaServos(n, dt, dev)
    all_ids = torch.arange(n, device=dev)

    def jp():
        return robot.data.joint_pos.torch[:, wheel_ids]

    def physics_step():
        pos, vel = servos.step(jp())
        robot.set_joint_position_target_index(target=pos, joint_ids=wheel_ids)
        robot.set_joint_velocity_target_index(target=vel, joint_ids=wheel_ids)
        scene.write_data_to_sim()
        sim.step(render=False)
        scene.update(dt)

    servos.reset(all_ids, jp())
    results = {"num_envs": n, "pos_iters": args.pos_iters or 32}

    if args.profile:
        parts = {"servo": 0.0, "set_targets+write": 0.0, "sim.step": 0.0, "scene.update": 0.0}
        for i in range(300):
            ts = []
            for fn in (lambda: servos.step(jp()),
                       lambda: (robot.set_joint_position_target_index(target=jp(), joint_ids=wheel_ids),
                                scene.write_data_to_sim()),
                       lambda: sim.step(render=False),
                       lambda: scene.update(dt)):
                torch.cuda.synchronize(); t = time.perf_counter(); fn(); torch.cuda.synchronize()
                ts.append(time.perf_counter() - t)
            if i >= 50:
                for k, v in zip(parts, ts):
                    parts[k] += v / 250
        print("[m0] PROFILE ms/step " + json.dumps({k: round(v * 1000, 2) for k, v in parts.items()}))
        return

    if not args.no_checks:
        ok = True
        mass = robot.data.body_mass.torch.sum(dim=1)
        e = (mass - MASS_KG).abs().max().item()
        ok &= e < 1e-3
        print(f"{'PASS' if e < 1e-3 else 'FAIL'}  mass      all envs {mass.min():.4f}..{mass.max():.4f} kg (want {MASS_KG})")

        for _ in range(PHYSICS_HZ):            # settle 1 s, commanding zero
            physics_step()
        z = robot.data.root_link_pose_w.torch[:, 2]
        g = robot.data.projected_gravity_b.torch
        tilt = torch.rad2deg(torch.acos((-g[:, 2]).clamp(-1, 1)))
        good = ((z - WHEEL_R).abs() < 0.002) & (tilt < 0.5)
        ok &= bool(good.all())
        print(f"{'PASS' if good.all() else 'FAIL'}  rest      base_link z {z.min()*1000:.1f}..{z.max()*1000:.1f} mm "
              f"(want 37.0), tilt max {tilt.max():.2f} deg")

        # three groups drive at once
        twist = torch.zeros(n, 3, device=dev)
        grp = all_ids % 3
        twist[grp == 0, 0] = 0.10
        twist[grp == 1, 1] = 0.10
        twist[grp == 2, 2] = 0.50
        servos.command_twist(twist)
        acc = torch.zeros(n, 3, device=dev)
        k = 0
        for i in range(2 * PHYSICS_HZ):        # 2 s; average the last second
            physics_step()
            if i >= PHYSICS_HZ:
                v = robot.data.root_link_lin_vel_b.torch
                w = robot.data.root_link_ang_vel_b.torch
                acc += torch.stack([v[:, 0], v[:, 1], w[:, 2]], 1)
                k += 1
        meas = acc / k
        for gi, (label, col, want) in enumerate([("forward 0.10 m/s", 0, 0.10), ("strafe  0.10 m/s", 1, 0.10),
                                                 ("rotate  0.50 rad/s", 2, 0.50)]):
            m = meas[grp == gi]
            on = m[:, col] / want
            off = m.clone()
            off[:, col] = 0
            # cross-talk in the other two axes, scaled to the same units as the main axis
            xt = off.abs().max().item()
            good = bool(((on - 1).abs() < 0.05).all()) and xt < 0.05 * want
            ok &= good
            print(f"{'PASS' if good else 'FAIL'}  {label}  achieved {on.min()*100:.1f}..{on.max()*100:.1f}% "
                  f"over {len(m)} envs, worst cross-talk {xt:.4f}")
        results["checks_pass"] = bool(ok)
        print("ALL PASS" if ok else "SOME CHECKS FAILED")

    # throughput: every env driving a random twist, changed every 0.1 s like a 10 Hz policy
    servos.randomize(all_ids)
    lim = torch.tensor([0.15, 0.15, 1.0], device=dev)
    for i in range(48):                        # warm-up (CUDA kernels, caches)
        if i % 24 == 0:
            servos.command_twist((2 * torch.rand(n, 3, device=dev) - 1) * lim)
        physics_step()
    torch.cuda.synchronize()
    t0 = time.time()
    for i in range(args.timed_steps):
        if i % 24 == 0:
            servos.command_twist((2 * torch.rand(n, 3, device=dev) - 1) * lim)
        physics_step()
    torch.cuda.synchronize()
    wall = time.time() - t0
    finite = bool(torch.isfinite(robot.data.root_link_pose_w.torch).all())
    tilt = torch.rad2deg(torch.acos((-robot.data.projected_gravity_b.torch[:, 2]).clamp(-1, 1)))
    results.update(
        physics_steps_per_s=round(args.timed_steps / wall, 1),
        env_steps_per_s=round(args.timed_steps * n / wall),
        policy_samples_per_s_10hz=round(args.timed_steps * n / wall / 24),
        realtime_factor_per_env=round(args.timed_steps * dt / wall, 2),
        all_finite=finite,
        max_tilt_deg_after=round(tilt.max().item(), 2),
        gpu_mem_gb=round(torch.cuda.max_memory_allocated() / 1e9, 2),
    )
    print("[m0] RESULT " + json.dumps(results))


if __name__ == "__main__":
    main()
    app.close()
