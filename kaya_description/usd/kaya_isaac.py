"""Put Kaya in any Isaac Sim scene, with everything the USD cannot carry.

kaya.usda carries the robot: shape, mass, joints, drive gains, colliders,
friction, the camera and IMU. What it cannot carry lives in the SCENE or in
Python, and a scene that skips any of it gets a different robot:

    stage in metres, Z up          check_stage()     wrong size / lying on its side
    240 Hz physics                 setup_physics()   rollers chatter at 60 Hz
    a floor with real friction     add_floor()       tile measured <= mu 0.40
    servos that act like ST3020s   KayaDriver        velocity-only drives creep
                                                     on slopes; no 0.1 s lag
    IMU noise                      KayaDriver.read_imu(noisy=True)

Minimal use, in a script run with Isaac Sim's python.sh (after SimulationApp):

    import kaya_isaac as kaya
    import isaacsim.core.experimental.utils.app as app_utils

    stage = kaya.new_stage()                 # or kaya.check_stage(your_stage)
    kaya.add_floor(stage)
    kaya.add_kaya(stage, "/World/kaya")
    kaya.setup_physics(device="cpu")         # "cuda" for GPU physics
    app_utils.play()
    robot = kaya.KayaDriver("/World/kaya")
    while simulation_app.is_running():
        robot.drive(0.1, 0.0, 0.0)           # vx, vy (m/s), w (rad/s), body frame
        robot.update()                       # once per frame, before stepping
        simulation_app.update()

Everything here was measured against the real robot - see README.md.
"""

import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ASSET = HERE / "kaya.usda"
PHYSICS_HZ = 240            # the rollers are 12 mm and ~3 g; 60 Hz is not enough
FLOOR_MU = (0.40, 0.32)     # static, dynamic: the tile at home is <= the table


def cameras(robot_path):
    """Prim paths of the Gemini 335L's sensors on a Kaya at robot_path."""
    cam = f"{robot_path}/base_link/camera_link/gemini_335L"
    return {
        "rgb": f"{cam}/camera_rgb/Camera_rgb",                   # 848x480
        "depth": f"{cam}/camera_ir_left/Camera_Depth_Left",      # 848x480
        "ir_left": f"{cam}/camera_ir_left/Camera_ir_left",
        "ir_right": f"{cam}/camera_ir_right/Camera_ir_right",
        "imu": f"{robot_path}/base_link/camera_link/camera_depth_frame/imu",
    }


def new_stage():
    """A fresh stage in metres, Z up, with /World."""
    import isaacsim.core.experimental.utils.stage as stage_utils
    from pxr import UsdGeom

    stage_utils.create_new_stage()
    stage = stage_utils.get_current_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    return stage


def check_stage(stage):
    """Kaya is authored in metres, Z up. A reference does not rescale or
    rotate, so any other stage gets a robot 100x too small or on its side."""
    from pxr import UsdGeom

    mpu, up = UsdGeom.GetStageMetersPerUnit(stage), UsdGeom.GetStageUpAxis(stage)
    if abs(mpu - 1.0) > 1e-9 or up != UsdGeom.Tokens.z:
        raise ValueError(f"Kaya needs a stage in metres with Z up; this one has "
                         f"metersPerUnit={mpu}, upAxis={up}")


def setup_physics(device="cpu"):
    """240 Hz physics. The timestep belongs to the scene, not the robot, and
    Isaac's default is 60 Hz."""
    from isaacsim.core.simulation_manager import SimulationManager

    SimulationManager.setup_simulation(dt=1.0 / PHYSICS_HZ, device=device)


def add_floor(stage, path="/World/ground", mu=FLOOR_MU, size=40.0, templates="wireframe-blue"):
    """A ground plane with a physics material. The rollers combine friction
    with "min", so the floor only matters when it is slipperier than 0.40 -
    which is also how to model a slippery floor, or randomise it for training."""
    from isaacsim.core.experimental.objects import GroundPlane
    from pxr import UsdPhysics, UsdShade

    GroundPlane(path, sizes=size, templates=templates)
    mat = UsdShade.Material.Define(stage, f"{path}_material")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(mu[0]))
    api.CreateDynamicFrictionAttr(float(mu[1]))
    api.CreateRestitutionAttr(0.0)
    for prim in stage.GetPrimAtPath(path).GetAllChildren():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    return mat


def add_kaya(stage, path="/World/kaya", position=(0.0, 0.0, 0.002), yaw_deg=0.0, usd=ASSET):
    """Reference the asset. Its origin is base_footprint (the floor), so z = 0
    stands it on the floor; the default 2 mm avoids a contact impulse at start.
    The whole usd/ folder must stay together: kaya.usda loads its geometry
    and the camera by relative path."""
    import isaacsim.core.experimental.utils.stage as stage_utils
    from pxr import Gf, UsdGeom

    check_stage(stage)
    stage_utils.add_reference_to_stage(usd_path=str(usd), path=path)
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
    xf.AddRotateZOp().Set(float(yaw_deg))
    return stage.GetPrimAtPath(path)


class KayaDriver:
    """Drive a Kaya like the real one, and keep driving it across GUI reloads.

    Commands go through WheelServos (kaya_servos.py): the same kinematics as
    the robot's drive node, the servo chain measured on the robot, and
    position-fed servos that hold on a slope. Call update() once per frame.

    Stop -> Play, or "Fetch" on a changed USD, rebuilds physics and kills the
    old handles. update() re-attaches. If physics is torn down WHILE playing,
    nothing errors loudly: physics keeps stepping, but the simulation view is
    gone, robot queries fail and the servo callback silently stops - the
    robot just freezes on its last target (seen in the GUI 2026-09-27, and
    reproduced headless with SimulationManager.invalidate_physics()). Isaac
    only rebuilds the view on a fresh Play, so after half a second of
    "playing without a view" update() does Stop -> Play itself (which also
    resets the robot to its start pose)."""

    RESTART_AFTER_FRAMES = 30

    def __init__(self, path="/World/kaya", ideal=False):
        self.path = path
        self.ideal = ideal
        self.robot = None
        self.servos = None
        self.cmd = (0.0, 0.0, 0.0)
        self._imu = None
        self._was_playing = False
        self._stalled = 0
        self.update()

    def _attach(self):
        from isaacsim.core.experimental.prims import Articulation

        from kaya_servos import WheelServos

        if self.servos is not None:
            self.servos.close()
        self.robot = Articulation(self.path)
        self.servos = WheelServos(self.robot, ideal=self.ideal)
        self._imu = None

    def update(self):
        import isaacsim.core.experimental.utils.app as app_utils
        from isaacsim.core.simulation_manager import SimulationManager

        playing = app_utils.is_playing()
        if not playing:
            self._was_playing = False
            self._stalled = 0
            return
        no_view = SimulationManager.get_physics_simulation_view() is None
        self._stalled = self._stalled + 1 if no_view else 0
        if self._stalled >= self.RESTART_AFTER_FRAMES:
            import carb
            carb.log_warn("[kaya] playing but the physics view is gone (USD reloaded?); "
                          "restarting the timeline")
            app_utils.stop()
            app_utils.update_app(steps=2)
            app_utils.play()
            self._stalled = 0
            self._was_playing = False
            return
        if self.servos is None or self.servos.broken or not self._was_playing:
            try:
                self._attach()
            except Exception:
                self.servos = None     # physics not ready yet: next frame
                self._was_playing = False
                return
        self._was_playing = True
        self.servos.drive(*self.cmd)

    def drive(self, vx, vy, w):
        """Body-frame command: vx forward, vy left (m/s), w counter-clockwise
        (rad/s) - what /cmd_vel means on the robot."""
        self.cmd = (float(vx), float(vy), float(w))
        if self.servos is not None and not self.servos.broken:
            self.servos.drive(*self.cmd)

    def read_imu(self, noisy=True):
        """Gemini IMU (accel m/s^2, gyro rad/s) in camera_accel_gyro_optical_frame,
        as /camera/gyro_accel/sample reports it. noisy=True adds the real
        sensor's measured noise, bias and scale (kaya_imu_noise.py)."""
        from isaacsim.sensors.experimental.physics import IMUSensor

        from kaya_imu_noise import corrupt

        if self._imu is None:
            self._imu = IMUSensor(cameras(self.path)["imu"])
        r = self._imu.get_sensor_reading()
        accel = np.array([r.linear_acceleration_x, r.linear_acceleration_y, r.linear_acceleration_z])
        gyro = np.array([r.angular_velocity_x, r.angular_velocity_y, r.angular_velocity_z])
        return corrupt(accel, gyro) if noisy else (accel, gyro)

    def close(self):
        if self.servos is not None:
            self.servos.close()


def yaw_of(quat_wxyz):
    w, x, y, z = quat_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
