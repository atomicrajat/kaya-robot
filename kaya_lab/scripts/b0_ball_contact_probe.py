"""Where does the ball touch the simulated Kaya? Compare with the real robot.

Each env: Kaya at the origin facing +x creeps forward at 5 cm/s through the
servo chain; a ball starts 0.45 m ahead at a different lateral offset. The
moment the ball starts moving, record its centre relative to the robot and to
the COLOUR lens (the frame the real measurements were made in).

Real robot, 2026-09-27 (kaya_lab/data/ball_frames_20260927, sphere fit):
    touching, centred        X 13.9 cm ahead of colour lens, Y -1.7 cm
    touching, one wheel      X 12.4, Y +3.6
    touching, other wheel    X 13.7, Y -8.4        -> pocket ~12 cm wide

    ./isaaclab.sh -p .../kaya_lab/scripts/b0_ball_contact_probe.py
"""

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--offsets_cm", type=str, default="-20,-15,-12,-10,-8,-6,-4,-2,0,2,4,6,8,10,12,15,20")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402
from pxr import UsdGeom  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils.configclass import configclass  # noqa: E402
from isaaclab.utils.math import subtract_frame_transforms  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kaya_lab.assets import KAYA_CFG, PHYSICS_HZ, WHEEL_JOINTS, floor_cfg  # noqa: E402
from kaya_lab.ball import BALL_CFG, BALL_RADIUS, set_hollow_inertia  # noqa: E402
from kaya_lab.servos import KayaServos  # noqa: E402

OFFS = [float(v) / 100 for v in args.offsets_cm.split(",")]


@configclass
class ProbeSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=floor_cfg())
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))
    kaya: ArticulationCfg = KAYA_CFG.replace(prim_path="{ENV_REGEX_NS}/Kaya")
    ball: RigidObjectCfg = BALL_CFG.replace(prim_path="{ENV_REGEX_NS}/Ball")


def main():
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1.0 / PHYSICS_HZ, device=args.device))
    scene = InteractiveScene(ProbeSceneCfg(num_envs=len(OFFS), env_spacing=2.0))
    stage = sim.stage
    for i in range(len(OFFS)):
        set_hollow_inertia(stage, f"/World/envs/env_{i}/Ball")
    # colour lens pose in base_link (static transform, read from USD before playing)
    xc = UsdGeom.XformCache()
    base = xc.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/envs/env_0/Kaya/base_link"))
    rgb = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(
        "/World/envs/env_0/Kaya/base_link/camera_link/gemini_335L/camera_rgb/Camera_rgb"))
    rel = rgb * base.GetInverse()     # USD row-vector convention: point * rel
    lens_b = torch.tensor(list(rel.ExtractTranslation()), dtype=torch.float32)
    sim.reset()
    robot, ball = scene["kaya"], scene["ball"]
    n, dev, dt = len(OFFS), sim.device, sim.get_physics_dt()
    lens_b = lens_b.to(dev)
    print(f"[probe] colour lens in base_link: x {lens_b[0]*100:.1f} y {lens_b[1]*100:.1f} z {lens_b[2]*100:.1f} cm")

    # place the balls
    pose = ball.data.default_root_pose.torch.clone()
    pose[:, 0] = 0.45
    pose[:, 1] = torch.tensor(OFFS, device=dev)
    pose[:, 2] = BALL_RADIUS + 0.001
    pose[:, :3] += scene.env_origins
    ball.write_root_pose_to_sim_index(root_pose=pose)
    ball.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev))

    wheel_ids, _ = robot.find_joints(WHEEL_JOINTS, preserve_order=True)
    servos = KayaServos(n, dt, dev)
    servos.reset(torch.arange(n, device=dev), robot.data.joint_pos.torch[:, wheel_ids])
    hit = torch.full((n, 2), float("nan"), device=dev)
    start = ball.data.root_pos_w.torch.clone()

    def step():
        pos, vel = servos.step(robot.data.joint_pos.torch[:, wheel_ids])
        robot.set_joint_position_target_index(target=pos, joint_ids=wheel_ids)
        robot.set_joint_velocity_target_index(target=vel, joint_ids=wheel_ids)
        scene.write_data_to_sim()
        sim.step(render=False)
        scene.update(dt)

    for _ in range(PHYSICS_HZ // 2):          # settle
        step()
    start = ball.data.root_pos_w.torch.clone()
    servos.command_twist(torch.tensor([[0.05, 0.0, 0.0]], device=dev).repeat(n, 1))
    for _ in range(PHYSICS_HZ * 10):
        step()
        moved = torch.linalg.norm(ball.data.root_pos_w.torch[:, :2] - start[:, :2], dim=1) > 0.002
        new = moved & torch.isnan(hit[:, 0])
        if new.any():
            b, _ = subtract_frame_transforms(robot.data.root_link_pos_w.torch, robot.data.root_link_quat_w.torch,
                                             ball.data.root_pos_w.torch)
            hit[new] = b[new, :2]
        if not torch.isnan(hit[:, 0]).any():
            break
    # where did each ball end up after being pushed (still in front, or slid off?)
    b_end, _ = subtract_frame_transforms(robot.data.root_link_pos_w.torch, robot.data.root_link_quat_w.torch,
                                         ball.data.root_pos_w.torch)
    print("[probe] offset | contact: ball centre in base (x, y) | rel. colour lens (X ahead, Y left) | after 10 s push, ball y")
    for i, o in enumerate(OFFS):
        x, y = hit[i].tolist()
        print(f"[probe] {o*100:+6.1f} cm | base ({x*100:6.1f}, {y*100:+6.1f}) | lens ({(x-lens_b[0].item())*100:5.1f}, "
              f"{(y-lens_b[1].item())*100:+6.1f}) | end y {b_end[i,1].item()*100:+6.1f} cm, x {b_end[i,0].item()*100:5.1f}")


if __name__ == "__main__":
    main()
    app.close()
