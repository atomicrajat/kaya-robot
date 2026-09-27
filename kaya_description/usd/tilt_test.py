#!/usr/bin/env python3
"""Re-run the real robot's tilt test in Isaac Sim, to calibrate roller friction.

    ~/isaac-sim/python.sh kaya_description/usd/tilt_test.py                  # the asset as built
    ~/isaac-sim/python.sh kaya_description/usd/tilt_test.py --mu 0.35 0.45   # sweep candidates

The real test (2026-09-27): Kaya on a table, table tilted slowly, the whole
robot SLIDES (rather than rolls) at 15 deg.

Why not just mu = tan(15) = 0.27: that holds for a block. On an omni base each
roller spins freely sideways, so a wheel can only resist along its own rolling
direction; the three wheels together resist less than mu * weight, and the
robot slides at a shallower angle than a block with the same mu would. How much
shallower depends on which way the robot faces downhill - the sweep over
--yaw covers that (3 wheels at 120 deg + mirror symmetry: 0-60 deg is all of
it).

Tilting the table is done by tilting GRAVITY, which is the same physics and
keeps the floor, the robot and the contacts exactly as they are.
"""

import argparse
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument("--usd", default=str(HERE / "kaya.usda"))
parser.add_argument("--mu", type=float, nargs="*", default=[],
                    help="roller static mu values to try (default: the asset's own)")
parser.add_argument("--yaw", type=float, nargs="*", default=[0.0, 30.0, 60.0],
                    help="robot heading relative to downhill, deg")
parser.add_argument("--rate", type=float, default=1.0, help="tilt rate, deg/s")
parser.add_argument("--start", type=float, default=5.0, help="first tilt, deg")
parser.add_argument("--end", type=float, default=35.0, help="give up at, deg")
parser.add_argument("--device", choices=["cpu", "cuda"], default="cpu",
                    help="physics on CPU (one robot: fastest) or GPU (what training uses)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import UsdGeom  # noqa: E402

import sys  # noqa: E402

sys.path.insert(0, str(HERE))
import kaya_isaac as kaya  # noqa: E402
from kaya_servos import WheelServos  # noqa: E402

G = 9.81
ROBOT = "/World/kaya"
ROLLER_MAT = f"{ROBOT}/PhysicsMaterials/roller_rubber"
SLIDE_MM = 5.0          # displacement that counts as "it slid"


def main():
    stage = kaya.new_stage()
    # A grippy floor, so the roller material (combine mode "min") is what
    # sets the coefficient - the test is calibrating the roller/table pair.
    kaya.add_floor(stage, mu=(1.0, 1.0), size=20.0)
    robot_prim = kaya.add_kaya(stage, ROBOT, usd=args.usd)
    r_op = UsdGeom.Xformable(robot_prim).GetOrderedXformOps()[1]      # the rotateZ op
    kaya.setup_physics(device=args.device)
    scene = SimulationManager.get_physics_scenes()[0]
    app_utils.update_app(steps=2)
    robot = Articulation(ROBOT)
    timeline = omni.timeline.get_timeline_interface()
    mat = stage.GetPrimAtPath(ROLLER_MAT)
    own_mu = mat.GetAttribute("physics:staticFriction").Get()
    ratio = mat.GetAttribute("physics:dynamicFriction").Get() / own_mu
    mus = args.mu or [own_mu]

    rows = []
    for mu in mus:
        for yaw in args.yaw:
            mat.GetAttribute("physics:staticFriction").Set(float(mu))
            mat.GetAttribute("physics:dynamicFriction").Set(float(mu * ratio))
            r_op.Set(float(yaw))
            scene.set_gravity((0.0, 0.0, -G))
            app_utils.play()
            app_utils.update_app(steps=3)
            # servos powered and holding position, as on the real robot
            servos = WheelServos(robot)
            wheel_idx = servos.idx
            for _ in range(60):                               # settle 1 s
                simulation_app.update()
            p0 = robot.get_world_poses()[0].numpy()[0].copy()
            q0 = robot.get_dof_positions().numpy()[0, wheel_idx].copy()
            t0 = SimulationManager.get_simulation_time()
            slid_at, wheel_turn = None, 0.0
            while True:
                tilt = args.start + args.rate * (SimulationManager.get_simulation_time() - t0)
                if tilt > args.end:
                    break
                # downhill = world +X; the robot faces `yaw` from it
                th = math.radians(tilt)
                scene.set_gravity((G * math.sin(th), 0.0, -G * math.cos(th)))
                simulation_app.update()
                p = robot.get_world_poses()[0].numpy()[0]
                if np.hypot(*(p - p0)[:2]) * 1000 > SLIDE_MM:
                    slid_at = tilt
                    dq = robot.get_dof_positions().numpy()[0, wheel_idx] - q0
                    wheel_turn = float(np.degrees(np.abs(dq)).max())
                    break
            servos.close()
            app_utils.stop()
            app_utils.update_app(steps=2)
            rows.append((mu, yaw, slid_at, wheel_turn))
            print(f"[tilt] mu {mu:.3f}  heading {yaw:4.0f} deg  -> "
                  + (f"slides at {slid_at:5.2f} deg (wheels turned {wheel_turn:.1f} deg)"
                     if slid_at else f"no slide by {args.end} deg"), flush=True)

    print("\n=========== TILT TEST (real robot: slides at 15 deg) ===========")
    print(f"block model would say {math.degrees(math.atan(mus[0])):.1f} deg for mu {mus[0]:.3f}")
    for mu in mus:
        angles = [a for m, _, a, _ in rows if m == mu and a]
        if angles:
            print(f"mu {mu:.3f}: slides at {min(angles):.2f}-{max(angles):.2f} deg "
                  f"(mean {np.mean(angles):.2f}) across headings")
    simulation_app.close()


main()
