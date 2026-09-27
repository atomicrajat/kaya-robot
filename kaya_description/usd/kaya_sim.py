#!/usr/bin/env python3
"""Spawn Kaya in Isaac Sim: floor, lights, a few props, camera views, teleop.

    ~/isaac-sim/python.sh kaya_description/usd/kaya_sim.py          # GUI
    ~/isaac-sim/python.sh kaya_description/usd/kaya_sim.py --test   # headless checks

GUI keys (click the viewport first so it has focus):
    W / S    forward / back          A / D    strafe left / right
    Q / E    rotate left / right     SPACE    stop

--test is the check that the asset is RIGHT, not just that it loads:
    * masses as PhysX sees them, summed, against the 2.088 kg scale reading
    * forward, strafe and rotate, commanded through the SAME kinematics the
      real drive node uses, and the achieved body velocity compared to the
      command - a wrong wheel angle, axis sign or roller setup shows up here
      as the robot going the wrong way
    * parked on a 10 deg slope, it must hold (the servos do)
    * the Gemini's depth against a wall at a known distance, and the depth
      gradient that proves the camera is pitched UP, not down
    It also saves what the chase view and the robot's camera see, to --out.
"""

import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true", help="headless verification run")
parser.add_argument("--usd", default=str(HERE / "kaya.usda"))
parser.add_argument("--out", default=str(HERE / "test_output"))
parser.add_argument("--device", choices=["cpu", "cuda"], default="cpu",
                    help="physics on CPU (one robot: fastest) or GPU (what training uses)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402  (must start before any omni import)

simulation_app = SimulationApp({"headless": args.test, "width": 1600, "height": 900})

import carb  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

sys.path.insert(0, str(HERE))
import kaya_isaac as kaya  # noqa: E402
from kaya_isaac import yaw_of  # noqa: E402
from kaya_servos import wheel_speeds  # noqa: E402

ROBOT = "/World/kaya"
SENSORS = kaya.cameras(ROBOT)
CAM = f"{ROBOT}/base_link/camera_link/gemini_335L"
RGB_CAM, DEPTH_CAM = SENSORS["rgb"], SENSORS["depth"]
# The real Gemini IMU at rest on the robot, /camera/gyro_accel/sample, m/s^2,
# optical frame (~/bags/phase1_imu on the robot): the source of the 4.89 deg.
REAL_IMU_AT_REST = (-0.1611, -9.7531, 0.8337)

WALL_X = 1.5        # front face of the test wall, m, in the robot's start frame


def build_world(stage):
    # Floor: shiny tile, the real home floor - see kaya_isaac.add_floor.
    kaya.add_floor(stage)

    # Lighting: a dome for soft fill everywhere + one sun for shadows/shape.
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
    dome.CreateIntensityAttr(600.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(1.0)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 25.0, -30.0))

    # Props: a wall ahead at a known distance (the depth check uses it) and a
    # few boxes so the camera has something to look at.
    def box(path, centre, size, colour):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*centre))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    box("/World/props/wall", (WALL_X + 0.05, 0.0, 1.0), (0.1, 4.0, 2.0), (0.75, 0.72, 0.65))
    box("/World/props/box_red", (0.9, 0.55, 0.15), (0.3, 0.3, 0.3), (0.8, 0.15, 0.1))
    box("/World/props/box_green", (1.1, -0.6, 0.25), (0.3, 0.3, 0.5), (0.2, 0.6, 0.25))
    box("/World/props/box_blue", (-0.9, 0.8, 0.1), (0.2, 0.2, 0.2), (0.15, 0.3, 0.8))

    kaya.add_kaya(stage, ROBOT, position=(0, 0, 0.005), usd=args.usd)

    chase = UsdGeom.Camera.Define(stage, "/World/chase_cam")
    chase.CreateFocalLengthAttr(18.0)
    chase.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
    xf = UsdGeom.Xformable(chase)
    xf.AddTranslateOp().Set(Gf.Vec3d(-0.75, -0.75, 0.55))
    # USD cameras look down -Z. Aim at the robot's centre, slightly above it.
    look = Gf.Matrix4d().SetLookAt(Gf.Vec3d(-0.75, -0.75, 0.55), Gf.Vec3d(0.15, 0, 0.08),
                                   Gf.Vec3d(0, 0, 1)).GetInverse()
    xf.AddOrientOp().Set(Gf.Quatf(look.ExtractRotationQuat()))


def setup_gui_views():
    from omni.kit.viewport.utility import create_viewport_window, get_active_viewport

    get_active_viewport().camera_path = "/World/chase_cam"
    try:
        create_viewport_window("Kaya - Gemini 335L RGB", width=640, height=420,
                               position_x=940, position_y=60, camera_path=RGB_CAM)
    except Exception as exc:  # layout is a nicety, never a reason to die
        carb.log_warn(f"second viewport not created: {exc}")


class Teleop:
    KEYS = {"W": (0.15, 0, 0), "S": (-0.15, 0, 0), "A": (0, 0.15, 0),
            "D": (0, -0.15, 0), "Q": (0, 0, 1.0), "E": (0, 0, -1.0)}

    def __init__(self):
        import omni.appwindow
        self.cmd = (0.0, 0.0, 0.0)
        inp = carb.input.acquire_input_interface()
        kb = omni.appwindow.get_default_app_window().get_keyboard()
        self._sub = inp.subscribe_to_keyboard_events(kb, self._on_key)

    def _on_key(self, e, *_):
        name = e.input.name
        if e.type == carb.input.KeyboardEventType.KEY_PRESS:
            if name in self.KEYS:
                self.cmd = self.KEYS[name]
            elif name == "SPACE":
                self.cmd = (0.0, 0.0, 0.0)
        elif e.type == carb.input.KeyboardEventType.KEY_RELEASE and name in self.KEYS:
            self.cmd = (0.0, 0.0, 0.0)
        return True


def run_test(driver):
    robot, servos = driver.robot, driver.servos
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    timeline = omni.timeline.get_timeline_interface()
    results, ok = [], True

    def check(name, passed, detail):
        nonlocal ok
        ok &= passed
        results.append(f"{'PASS' if passed else 'FAIL'}  {name:34s} {detail}")

    def pose():
        p, q = robot.get_world_poses()
        return p.numpy()[0].copy(), q.numpy()[0].copy()

    def sim_time():
        # physics time, not the timeline: see WheelServos for why they differ
        return SimulationManager.get_simulation_time()

    def run(seconds, cmd=(0, 0, 0)):
        servos.drive(*cmd)
        t0 = sim_time()
        while sim_time() - t0 < seconds:
            simulation_app.update()
        return sim_time() - t0

    # 0. timing sanity: the servos integrate on physics steps, so those must
    #    be what we think they are
    from isaacsim.core.simulation_manager import IsaacEvents
    steps = []
    uid = SimulationManager.register_callback(lambda dt, *_: steps.append(dt),
                                              IsaacEvents.PRE_PHYSICS_STEP)
    t_sim, t_tl = sim_time(), timeline.get_current_time()
    for _ in range(60):
        simulation_app.update()
    SimulationManager.deregister_callback(uid)
    results.append(f"      timing over 60 frames: {len(steps)} physics callbacks, dt "
                   f"{sorted(set(round(d, 6) for d in steps))}, sim time +{sim_time() - t_sim:.3f} s, "
                   f"timeline +{timeline.get_current_time() - t_tl:.3f} s")

    scene0 = SimulationManager.get_physics_scenes()[0]
    results.append(f"      physics device {SimulationManager.get_physics_sim_device()}, "
                   f"GPU dynamics {scene0.get_enabled_gpu_dynamics()}")

    # 1. what PhysX actually loaded
    masses = robot.get_link_masses().numpy()[0]
    check("mass (PhysX, all links)", abs(masses.sum() - 2.088) < 0.002,
          f"{masses.sum():.4f} kg vs 2.088 scale, {len(masses)} links, {robot.num_dofs} dofs")

    # 2. settle: base_link should sit one wheel radius up, level
    run(1.5)
    p, q = pose()
    tilt = math.degrees(2 * math.asin(min(1.0, math.hypot(q[1], q[2]))))
    check("rest height of base_link", abs(p[2] - 0.037) < 0.002,
          f"z = {p[2] * 1000:.1f} mm (wheel radius 37.0)")
    check("rest tilt", tilt < 1.0, f"{tilt:.2f} deg")
    acc = []
    for _ in range(30):
        simulation_app.update()
        acc.append(driver.read_imu(noisy=False)[0])
    acc = np.mean(acc, axis=0)
    diff = np.abs(acc - REAL_IMU_AT_REST).max()
    check("IMU at rest vs the real Gemini IMU", diff < 0.05,
          f"sim {np.round(acc, 3).tolist()} real {list(REAL_IMU_AT_REST)} m/s^2")

    # 3. drive tests: achieved body velocity vs command
    def drive(name, cmd, secs=3.0):
        run(0.5, cmd)                        # let it get up to speed
        p0, q0 = pose()
        dt = run(secs, cmd)
        p1, q1 = pose()
        wv = robot.get_dof_velocities().numpy()[0, servos.idx]
        tv = np.array(wheel_speeds(*cmd))
        track = np.abs(wv).sum() / np.abs(tv).sum()
        err = servos.target - servos.q_unwrapped
        results.append(f"      {name}: wheels {np.round(wv, 2).tolist()} rad/s, "
                       f"target {np.round(tv, 2).tolist()} ({track:.1%}), "
                       f"servo position error {np.round(err, 3).tolist()} rad")
        y0 = yaw_of(q0)
        d = p1 - p0
        bx = math.cos(y0) * d[0] + math.sin(y0) * d[1]    # into body frame
        by = -math.sin(y0) * d[0] + math.cos(y0) * d[1]
        dyaw = (yaw_of(q1) - y0 + math.pi) % (2 * math.pi) - math.pi
        got = (bx / dt, by / dt, dyaw / dt)
        run(0.7)                             # stop before the next test
        return got

    vx, vy, w = drive("forward", (0.10, 0, 0))
    check("forward 0.10 m/s", vx > 0.08 and abs(vy) < 0.01 and abs(w) < 0.05,
          f"vx {vx:+.3f} vy {vy:+.3f} m/s  w {w:+.3f} rad/s")
    vx, vy, w = drive("strafe", (0, 0.10, 0))
    check("strafe left 0.10 m/s", vy > 0.08 and abs(vx) < 0.01 and abs(w) < 0.05,
          f"vx {vx:+.3f} vy {vy:+.3f} m/s  w {w:+.3f} rad/s")
    vx, vy, w = drive("rotate", (0, 0, 0.5))
    check("rotate CCW 0.5 rad/s", w > 0.4 and math.hypot(vx, vy) < 0.02,
          f"vx {vx:+.3f} vy {vy:+.3f} m/s  w {w:+.3f} rad/s")

    # 4. parked on a 10 deg slope, commanded speed 0: the real servos hold, so
    #    the sim must too (a velocity-only drive crept here). The slope is
    #    ramped in over 1 s, as parking on it would be, then held for 2 s.
    scene = SimulationManager.get_physics_scenes()[0]
    t0 = sim_time()
    servos.drive(0, 0, 0)
    while (ramp := (sim_time() - t0) / 1.0) < 1.0:
        th = math.radians(10.0 * ramp)
        scene.set_gravity((9.81 * math.sin(th), 0.0, -9.81 * math.cos(th)))
        simulation_app.update()
    th = math.radians(10.0)
    scene.set_gravity((9.81 * math.sin(th), 0.0, -9.81 * math.cos(th)))
    run(0.5)
    p0, _ = pose()
    w0 = servos.q_unwrapped.copy()
    run(2.0)
    p1, _ = pose()
    turned = np.degrees(np.abs(servos.q_unwrapped - w0)).max()
    scene.set_gravity((0.0, 0.0, -9.81))
    creep = np.hypot(*(p1 - p0)[:2]) * 1000
    check("holds on a 10 deg slope", creep < 1.0,
          f"moved {creep:.2f} mm in 2 s, wheels turned {turned:.2f} deg")

    # 5. camera: put the robot back square to the wall, then read depth
    robot.set_world_poses(positions=np.array([[0, 0, 0.042]]),
                          orientations=np.array([[1.0, 0, 0, 0]]))
    servos.reset()
    run(1.5)
    import omni.replicator.core as rep
    from PIL import Image

    rps = {n: rep.create.render_product(c, res) for n, c, res in
           [("rgb", RGB_CAM, (848, 480)), ("depth", DEPTH_CAM, (848, 480)),
            ("chase", "/World/chase_cam", (1280, 720))]}
    annot = {}
    for n, kind in [("rgb", "rgb"), ("depth", "distance_to_image_plane"), ("chase", "rgb")]:
        annot[n] = rep.AnnotatorRegistry.get_annotator(kind)
        annot[n].attach([rps[n]])
    for _ in range(30):                      # let the path tracer converge
        simulation_app.update()
    for n in ("rgb", "chase"):
        Image.fromarray(annot[n].get_data()[..., :3]).save(out / f"{n}.png")
    depth = np.asarray(annot["depth"].get_data(), dtype=np.float64)
    dv = np.where(np.isfinite(depth), depth, 0)
    Image.fromarray((np.clip(dv / 3.0, 0, 1) * 255).astype(np.uint8)).save(out / "depth.png")

    # Expected: optical centre ~ x 0.024 m in base_link (camera_link x + the
    # asset's front-face offset - the sensor's 11.2 mm setback), and the wall
    # is flat, so depth along the pitched optical axis = dx / cos(pitch).
    stage = stage_utils.get_current_stage()
    cam_world = UsdGeom.Xformable(stage.GetPrimAtPath(DEPTH_CAM)).ComputeLocalToWorldTransform(0)
    cam_x = cam_world.ExtractTranslation()[0]
    fwd = cam_world.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
    # the MOUNT is what is under test, so measure the optical axis in the
    # chassis frame; how the robot happens to stand is reported separately
    base_world = UsdGeom.Xformable(stage.GetPrimAtPath(f"{ROBOT}/base_link")).ComputeLocalToWorldTransform(0)
    fwd_base = base_world.GetInverse().TransformDir(fwd).GetNormalized()
    pitch_up = math.degrees(math.asin(fwd_base[2]))
    stance = math.degrees(math.asin(base_world.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()[2]))
    left_ir = UsdGeom.Xformable(stage.GetPrimAtPath(f"{CAM}/camera_ir_left/Camera_ir_left"))
    right_ir = UsdGeom.Xformable(stage.GetPrimAtPath(f"{CAM}/camera_ir_right/Camera_ir_right"))
    baseline = (left_ir.ComputeLocalToWorldTransform(0).ExtractTranslation()
                - right_ir.ComputeLocalToWorldTransform(0).ExtractTranslation()).GetLength()
    check("stereo baseline", abs(baseline - 0.09486) < 1e-4,
          f"{baseline * 1000:.2f} mm (the camera's own calibration: 94.86)")
    dcam = UsdGeom.Camera(stage.GetPrimAtPath(DEPTH_CAM)).GetCamera(0)
    fx_sim = 848 * dcam.focalLength / dcam.horizontalAperture
    check("depth fx at 848x480", abs(fx_sim - 408.162) < 0.05,
          f"{fx_sim:.2f} px (camera_info: 408.16)")
    h, wd = depth.shape
    centre = float(np.median(depth[h // 2 - 3:h // 2 + 3, wd // 2 - 3:wd // 2 + 3]))
    expected = (WALL_X - cam_x) / fwd[0]
    check("depth to wall (centre pixel)", abs(centre - expected) < 0.01,
          f"{centre:.4f} m, expected {expected:.4f} m")
    # rows +-9 deg off-axis: both land on the wall (lower ones see the floor)
    top = float(np.median(depth[h // 4, wd // 2 - 5:wd // 2 + 5]))
    bot = float(np.median(depth[3 * h // 4, wd // 2 - 5:wd // 2 + 5]))
    floor = float(np.median(depth[h - 5, wd // 2 - 5:wd // 2 + 5]))
    check("depth sees the floor (< 1 m)", np.isfinite(floor) and floor < 1.0,
          f"bottom-centre pixel {floor:.3f} m")
    check("camera pitched UP 4.89 deg", top > bot and abs(pitch_up - 4.89) < 0.05,
          f"optical axis {pitch_up:+.2f} deg on the chassis (chassis stance "
          f"{stance:+.2f} deg), depth top {top:.3f} > bottom {bot:.3f}")

    print("\n=========== KAYA ASSET TEST ===========")
    print("\n".join(results))
    print(f"images in {out}/  (rgb.png, depth.png, chase.png)")
    print("ALL PASS" if ok else "SOME CHECKS FAILED")
    return ok


def main():
    stage = kaya.new_stage()
    build_world(stage)
    kaya.setup_physics(device=args.device)       # 240 Hz - see kaya_isaac
    app_utils.update_app(steps=2)
    if not args.test:
        setup_gui_views()
    app_utils.play()
    app_utils.update_app(steps=3)
    driver = kaya.KayaDriver(ROBOT)
    print(f"[kaya] articulation: {driver.robot.num_dofs} dofs, "
          f"wheels at dof {driver.servos.idx.tolist()}", flush=True)

    if args.test:
        ok = run_test(driver)
        app_utils.stop()
        simulation_app.close()
        raise SystemExit(0 if ok else 1)

    teleop = Teleop()
    print("[kaya] ready - click the viewport, then W/S/A/D/Q/E to drive, SPACE to stop", flush=True)
    while simulation_app.is_running():
        driver.drive(*teleop.cmd)
        driver.update()           # also re-attaches after Stop/Play or a USD "Fetch"
        simulation_app.update()
    simulation_app.close()


main()
