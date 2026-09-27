#!/usr/bin/env python3
"""Build Kaya's Isaac Sim robot asset (USD) from the Fusion export + the URDF.

    ~/isaac-sim/python.sh kaya_description/usd/build_kaya_usd.py [--roller-collider hull|sphere]

Needs only `pxr` and numpy, both of which Isaac Sim's python.sh ships. No Kit app
is started, so it runs in a few seconds and can be re-run after every change to
the URDF or the CAD.

WHERE EACH NUMBER COMES FROM
    shape      source/kaya_robot_ttb.usdz  - Fusion 360 export. Flat: 137 meshes
               directly under the root, generic names ("Body1"), no hierarchy,
               no physics. This script sorts them into links.
    numbers    ../urdf/kaya.urdf.xacro     - masses, inertia, camera pose,
               wheel radius. Read, never retyped, so the sim cannot drift from
               what the robot runs.
    camera     sensors/orbbec_gemini_335L/ - NVIDIA's own Gemini 335L asset
               (Isaac 6.0 /Isaac/Sensors/Orbbec/Gemini335L), vendored so the
               robot loads offline and the camera model is pinned.

OUTPUT
    kaya_geometry.usdc   visual meshes + looks, already in each link's frame
    kaya.usda            THE asset to load. Sublayers the geometry and adds the
                         links, joints, mass, colliders and sensors - text, so
                         every physics setting can be read and diffed.

DECISIONS THAT ARE NOT OBVIOUS FROM THE CODE
 1. Rollers are simulated, 12 per wheel, each a link on a free revolute joint
    with sphere colliders (see ROLLER_ARMATURE for the measured comparison).
    CAD_CONTEXT.md sec. 5 planned one cylinder with anisotropic friction, but
    PhysX rigid-body contacts have no anisotropic friction, so that plan cannot
    be built. Free rollers are what NVIDIA's own Kaya and O3dyn assets do.
 2. Wheels sit at the CAD's GEOMETRIC 143 mm, not the URDF's 138 mm. 138 is the
    kinematic L calibrated on the real robot - it folds slip and compression
    into one number for odometry. The simulator has its own slip, so it gets
    the real geometry and the controller keeps using L = 138, exactly as on the
    robot. If the sim then needs a different L too, that is a finding.
 3. Wheel joint axis points INWARD (radial, toward the chassis centre), so a
    POSITIVE joint velocity is a POSITIVE servo Goal Speed, and
    v_i = -sin(th)*vx + cos(th)*vy + L*w holds with no sign flips. Derived from
    the 2026-09-03 servo pulse test: servo 3 (rear, 180 deg) positive moved the
    wheel top toward the robot's right, i.e. rotation about +X world = the
    inward radial there. The URDF's axis="1 0 0" is outward, the opposite; it
    has only ever driven RViz, where a spin direction is invisible.
 4. CAD wheel names are NOT servo ids (see kaya_dimensions.yaml servo_wheel_map),
    so wheels are identified by ANGLE: 300 -> wheel_1, 60 -> wheel_2,
    180 -> wheel_3, matching the URDF.
 5. The CAD's own camera meshes are dropped; NVIDIA's asset carries the same
    Orbbec STEP geometry plus the calibrated sensor prims.
"""

import argparse
import math
import re
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils, Vt

HERE = Path(__file__).resolve().parent
URDF = HERE.parent / "urdf" / "kaya.urdf.xacro"
CAD = HERE / "source" / "kaya_robot_ttb.usdz"
GEMINI = "sensors/orbbec_gemini_335L/orbbec_gemini_335L.usdc"
OUT_GEOM = "kaya_geometry.usdc"
OUT_ROBOT = "kaya.usda"

ROOT = "/kaya"

# servo id -> body angle, from the URDF's xacro:omni_wheel calls (verified on
# hardware 2026-09-03). Checked against the CAD below, not trusted blindly.
WHEEL_ANGLES_DEG = {"wheel_1": 300.0, "wheel_2": 60.0, "wheel_3": 180.0}

# Waveshare ST3020 at 12 V: 25 kg.cm stall, 60 RPM no-load.
#   https://www.waveshare.com/wiki/ST3020_Servo
SERVO_STALL_TORQUE_NM = 25 * 9.80665 / 100          # 2.45 N.m
SERVO_MAX_SPEED_DEG_S = 60 * 360 / 60               # 360 deg/s = 6.28 rad/s
# The servo closes its loop on a 4096-count encoder, so it is modelled as a
# POSITION servo: stiffness + damping, and a speed command is sent as a
# position target that advances at that speed (plus the matching velocity
# target) - see kaya_sim.py WheelServos.
#
# Damping alone (velocity drive, target 0) does NOT hold: measured 2026-09-27,
# on a 6 deg tilt the wheels creep at 0.08 rad/s whether damping is 57 or
# 5730 N.m.s/rad. It is solver residual, not the drive law: 4x the position
# iterations cut it 3x, velocity iterations change nothing, and the joint
# velocity reads ~0 while the position drifts. Only a stiffness term, which
# feeds back on position, stops it. The real robot does not creep either.
#
# Stiffness: stall torque at 0.5 deg of error, ~6 counts of the servo's
# 4096-count encoder - a tight position loop, as the real one is. The torque
# cap (2.45 N.m) is unchanged, so this makes it precise, not stronger.
# Measured on a 10 deg slope, zero command, 2 s (32 solver iterations):
#   stall at 2.5 deg: wheels turn 4.8 deg, robot moves 2.3 mm   (leaks)
#   stall at 0.5 deg: wheels turn 0.04 deg, robot moves 0.02 mm (holds)
# 64 iterations bought nothing further. Driving tracks 99% either way.
# USD angular units: N.m per deg, N.m per deg/s.
SERVO_STIFFNESS = SERVO_STALL_TORQUE_NM / 0.5
SERVO_DAMPING = 1.0

# Gemini 335L. Depth range: Orbbec datasheet. RGB: read off the real robot's
# /camera/color/camera_info, 2026-09-27 (width, height, fx, fy in pixels).
DEPTH_RANGE_M = (0.17, 20.0)
RGB_INTRINSICS = (848, 480, 404.85430908203125, 405.07916259765625)
# IR pair + depth at the navigation stack's 848x480 (rectified, so no
# distortion), and the camera's own extrinsics - read off the robot's driver
# 2026-09-27 (camera_info + /tf_static with publish_tf on). Principal points
# are within 2 px of centre and not modelled.
IR_INTRINSICS = (848, 480, 408.1622314453125, 408.1622314453125)
STEREO_BASELINE_M = 0.09486                          # left IR -> right IR
COLOR_IN_DEPTH_Q = (0.0023, 0.0007, -0.0018, 1.0)    # ROS (x,y,z,w), ~0.3 deg

# The Gemini's IMU, from the Orbbec driver's own /tf_static (recorded in
# ~/bags/phase1_imu on the robot, 2026-08-28): camera_depth_frame ->
# camera_gyro_frame, in body axes (x fwd, y left, z up). Its data is published
# in camera_accel_gyro_optical_frame (z forward, x right, y down).
IMU_IN_DEPTH_FRAME = (-0.01425, -0.00789, -0.00109)
OPTICAL = Gf.Quatd(0.5, -0.5, 0.5, -0.5)        # ROS (x,y,z,w) = (-.5,.5,-.5,.5)

# Roller mass, ESTIMATED (only the whole wheel was weighed: 112 g). Each roller
# = its CAD barrel in rubber + a steel axle pin (assumed 2 mm diameter, roller
# length). ~3.3 g each, ~39 g for twelve; the aluminium frame/hub gets the rest.
# Scaling the whole wheel by CAD volume instead gave 4.6 g rollers - too heavy
# for what are "very light" rollers on a heavy aluminium frame.
RUBBER_DENSITY = 1100.0
STEEL_DENSITY = 7850.0
ROLLER_PIN_DIAMETER = 0.002

# Roller joints. Measured in the sim 2026-09-27, forward 0.1 m/s at 240 Hz:
#   hull colliders, bare joints     wheel jitter +-2.1 rad/s, rollers 110 rad/s, 0.087 m/s
#   sphere + damping + armature     wheel jitter +-0.2 rad/s, rollers  16 rad/s, 0.097 m/s
# A 2.3 g roller is ~1e5 times lighter than the chassis, and a cooked (faceted)
# hull rolling over a plane is a stream of small impacts, so bare rollers spin
# up chaotically. Armature adds rotor inertia to the joint (1e-6 kg.m^2, ~25x
# the roller's own); the damping is bearing drag (1e-5 N.m.s/rad, ~0.03 N of
# equivalent drag at 0.1 m/s - negligible against 20 N of weight).
ROLLER_ARMATURE = 1e-6                      # kg.m^2
ROLLER_DAMPING = 1e-5 * math.pi / 180       # N.m.s/deg (USD angular units)

# Roller friction, calibrated 2026-09-27 from the real robot's tilt test: on a
# table, CAMERA FACING DOWNHILL, it slides at 15 deg. The same test in the sim
# (tilt_test.py --yaw 0): mu 0.30 -> 11.3 deg, 0.39 -> 15.07, 0.40 -> 15.5;
# slide onset is noisy by ~0.5 deg, i.e. mu +-0.015. NOT tan(15) = 0.27: the
# rollers turn freely sideways, so the robot slides at a shallower angle than
# a block with the same coefficient would. Dynamic = 0.8 x static is an
# assumption (only the onset of sliding was measured).
ROLLER_MU_STATIC = 0.39
ROLLER_FRICTION = (ROLLER_MU_STATIC, 0.8 * ROLLER_MU_STATIC)
# "min": the measured pair (roller on a hard table) governs on any floor at
# least as grippy; a slipperier floor still wins. PhysX's default "average"
# would silently blend this with whatever the scene's floor claims.
ROLLER_COMBINE = "min"
CHASSIS_FRICTION = (0.4, 0.3)

# Density by CAD appearance, kg/m^3. Used ONLY to split a measured link mass
# between its parts; every link total is then scaled to the URDF value.
DENSITY = [("aluminum", 2700), ("rubber", 1100), ("steel", 7850),
           ("brass", 8500), ("plastic", 1050), ("abs", 1050)]


# ----------------------------------------------------------------- URDF numbers
def read_xacro_properties(path):
    """Every <xacro:property name=... value=...>, evaluated in file order."""
    env = {"pi": math.pi}
    for name, value in re.findall(
            r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"', path.read_text()):
        expr = value[2:-1] if value.startswith("${") else value
        env[name] = float(eval(expr, {"__builtins__": {}}, env))
    return env


# ---------------------------------------------------------------- small math
def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rot_z(yaw) @ ry @ rx


def quat(r):
    """numpy 3x3 (column vectors = child axes in parent) -> Gf.Quatd.
    Gf matrices are row-vector, so they hold the transpose."""
    return Gf.Matrix3d(*r.T.flatten().tolist()).ExtractRotation().GetQuat()


def set_pose(prim, t, r):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, t)))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat(r))


def mesh_mass_properties(points, tris):
    """Volume, centroid and inertia-per-unit-density of a closed triangle mesh
    (signed tetrahedra from the origin). Inertia is about the centroid."""
    a, b, c = points[tris[:, 0]], points[tris[:, 1]], points[tris[:, 2]]
    det = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = det.sum() / 6.0
    com = (det[:, None] * (a + b + c)).sum(0) / (24.0 * vol)
    # second moment: for each tet, det/120 * (sum_ij v_i v_j^T + sum_i v_i v_i^T)
    s = a + b + c
    cov = (det[:, None, None] * (np.einsum("ni,nj->nij", s, s)
                                 + np.einsum("ni,nj->nij", a, a)
                                 + np.einsum("ni,nj->nij", b, b)
                                 + np.einsum("ni,nj->nij", c, c))).sum(0) / 120.0
    cov -= vol * np.outer(com, com)
    inertia = np.trace(cov) * np.eye(3) - cov
    return vol, com, inertia


def combine(parts):
    """[(mass, com, inertia_about_com)] -> one rigid body, same frame."""
    m = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / m
    inertia = np.zeros((3, 3))
    for pm, pc, pi in parts:
        d = pc - com
        inertia += pi + pm * (d @ d * np.eye(3) - np.outer(d, d))
    return m, com, inertia


def principal(inertia):
    """Diagonal inertia + the rotation from principal axes to the body frame,
    which is what UsdPhysics.MassAPI stores."""
    vals, vecs = np.linalg.eigh(inertia)
    if np.linalg.det(vecs) < 0:
        vecs[:, 2] *= -1
    return vals, vecs


# ----------------------------------------------------------------- CAD reading
class Part:
    def __init__(self, prim, xform):
        mesh = UsdGeom.Mesh(prim)
        self.name = prim.GetName()
        self.index = prim.GetParent().GetName()            # MeshInstance_N
        pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
        m = np.array(xform, dtype=np.float64)              # row-vector 4x4
        self.points = (pts @ m[:3, :3] + m[3, :3]) * 0.01  # CAD is in cm
        self.counts = np.array(mesh.GetFaceVertexCountsAttr().Get())
        self.indices = np.array(mesh.GetFaceVertexIndicesAttr().Get())
        n = mesh.GetNormalsAttr().Get()
        self.normals = None if n is None else np.array(n, dtype=np.float64) @ m[:3, :3]
        self.normals_interp = mesh.GetNormalsInterpolation()
        st = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
        self.st = st if st and st.HasValue() else None
        self.double_sided = bool(mesh.GetDoubleSidedAttr().Get())
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        self.material = mat.GetPrim().GetName().split("_")[1] if mat else "Default"
        self.source_material = mat
        self.centre = (self.points.min(0) + self.points.max(0)) / 2
        self.size = self.points.max(0) - self.points.min(0)

    def triangles(self):
        tris, k = [], 0
        for n in self.counts:
            f = self.indices[k:k + n]
            tris += [(f[0], f[i], f[i + 1]) for i in range(1, n - 1)]
            k += n
        return np.array(tris)

    def density(self):
        name = self.material.lower()
        return next((d for key, d in DENSITY if key in name), 1200)


def load_cad():
    """Returns the stage too: Parts keep handles into it (materials, st)."""
    stage = Usd.Stage.Open(str(CAD))
    cache = UsdGeom.XformCache()
    return stage, [Part(p, cache.GetLocalToWorldTransform(p))
                   for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]


def classify(parts, wheel_radius):
    """Sort the flat CAD soup into links. Returns
    {'base_link': [...], 'camera': [...], wheel: {'hub': [...], 'rollers': [...]}}"""
    groups = {"base_link": [], "camera": []}
    for w in WHEEL_ANGLES_DEG:
        groups[w] = {"hub": [], "rollers": []}

    def nearest_wheel(p):
        ang = math.degrees(math.atan2(p.centre[1], p.centre[0])) % 360
        w = min(WHEEL_ANGLES_DEG,
                key=lambda k: abs((WHEEL_ANGLES_DEG[k] - ang + 180) % 360 - 180))
        err = abs((WHEEL_ANGLES_DEG[w] - ang + 180) % 360 - 180)
        assert err < 5, f"{p.name} at {ang:.1f} deg matches no wheel"
        return w

    # hub plates and couplers sit ON the wheel's radial line, so the angle
    # from the chassis centre identifies them. Rollers do not: they ride up
    # to 3 cm off that line (168 deg for the 180 deg wheel), so each roller
    # goes to the nearest hub centre instead.
    hubs = {}
    for p in parts:
        if p.name.startswith("hub_plate"):
            hubs.setdefault(nearest_wheel(p), []).append(p.centre)
    hub_centre = {w: np.mean(c, axis=0) for w, c in hubs.items()}

    for p in parts:
        if p.name.startswith("roller_"):
            w = min(hub_centre, key=lambda k: np.linalg.norm(p.centre - hub_centre[k]))
            assert np.linalg.norm(p.centre - hub_centre[w]) < 0.037, p.name  # inside the wheel
            groups[w]["rollers"].append(p)
        elif p.name.startswith("hub_plate") or p.name.endswith("_coupler"):
            # the coupler rides on the servo shaft, so it turns with the wheel
            groups[nearest_wheel(p)]["hub"].append(p)
        elif re.fullmatch(r"Body\d+", p.name) and p.centre[2] > 0.115:
            groups["camera"].append(p)      # Orbbec STEP; replaced by NVIDIA's
        else:
            groups["base_link"].append(p)

    for w in WHEEL_ANGLES_DEG:
        n_hub, n_rol = len(groups[w]["hub"]), len(groups[w]["rollers"])
        assert (n_hub, n_rol) == (4, 12), f"{w}: {n_hub} hub parts, {n_rol} rollers"
        lowest = min(r.points[:, 2].min() for r in groups[w]["rollers"])
        assert abs(lowest + wheel_radius) < 5e-4, \
            f"{w} rollers bottom at {lowest:.4f}, expected {-wheel_radius}"
    return groups


# ----------------------------------------------------------------- writing
def make_looks(stage, parts):
    looks = {}
    for p in parts:
        if p.material in looks:
            continue
        mat = UsdShade.Material.Define(stage, f"{ROOT}/Looks/{p.material}")
        sh = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild("Shader"))
        sh.CreateIdAttr("UsdPreviewSurface")
        src = p.source_material
        src_shader = src.ComputeSurfaceSource()[0] if src else None
        for name, tp in [("diffuseColor", Sdf.ValueTypeNames.Color3f),
                         ("roughness", Sdf.ValueTypeNames.Float),
                         ("metallic", Sdf.ValueTypeNames.Float),
                         ("opacity", Sdf.ValueTypeNames.Float)]:
            inp = src_shader.GetInput(name) if src_shader else None
            if inp and inp.Get() is not None:
                sh.CreateInput(name, tp).Set(inp.Get())
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        looks[p.material] = mat
    return looks


def write_mesh(stage, path, part, to_link_r, to_link_t, looks):
    """Author a CAD part as a mesh expressed in its link's frame."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = (part.points - to_link_t) @ to_link_r
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(part.counts.astype(np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(part.indices.astype(np.int32)))
    if part.normals is not None:
        n = part.normals @ to_link_r
        n /= np.linalg.norm(n, axis=1, keepdims=True).clip(1e-12)
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(n.astype(np.float32)))
        mesh.SetNormalsInterpolation(part.normals_interp)
    if part.st is not None:
        pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, part.st.GetInterpolation())
        pv.Set(part.st.Get())
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(part.double_sided)
    mesh.CreateExtentAttr([Gf.Vec3f(*pts.min(0).tolist()), Gf.Vec3f(*pts.max(0).tolist())])
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(looks[part.material])
    return mesh


def unique_name(part, used):
    base = part.name if not re.fullmatch(r"Body\d+", part.name) \
        else "part_" + part.index.split("_")[-1]
    name, k = base, 1
    while name in used:
        name, k = f"{base}_{k}", k + 1
    used.add(name)
    return name


def physx(prim, schema, **attrs):
    """PhysX-specific attributes. PhysxSchema is not importable from plain
    python.sh, so apply the API by name and author its attributes directly -
    this is exactly what PhysxSchema would write."""
    prim.AddAppliedSchema(schema)
    types = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int,
             float: Sdf.ValueTypeNames.Float}
    for name, value in attrs.items():
        prim.CreateAttribute(name, types[type(value)]).Set(value)


def physics_material(stage, path, friction):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction[0])
    api.CreateDynamicFrictionAttr(friction[1])
    api.CreateRestitutionAttr(0.0)
    return mat


def bind_physics_material(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")


def set_mass(prim, mass, com, inertia):
    vals, vecs = principal(inertia)
    api = UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr(float(mass))
    api.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, vals)))
    api.CreatePrincipalAxesAttr(Gf.Quatf(quat(vecs)))


def revolute(stage, path, body0, body1, pos0, rot0, pos1, rot1):
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([body0])
    j.CreateBody1Rel().SetTargets([body1])
    j.CreateAxisAttr("X")
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalRot0Attr(Gf.Quatf(quat(rot0)))
    j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, pos1)))
    j.CreateLocalRot1Attr(Gf.Quatf(quat(rot1)))
    return j


def gemini_placement():
    """Where to put NVIDIA's asset inside camera_link. Its local axes are
    +Z optical/forward, +Y up, +X left (read off the asset: cameras rotated
    180 deg about Y, left IR at +X). camera_link is REP-103: +X fwd, +Y left,
    +Z up. camera_link's origin is the camera body's CENTRE - the URDF derives
    it as 'rear face on the ledge bend, half a depth forward' - so the asset's
    body centre is put there, not its own origin (which is on the front glass)."""
    stage = Usd.Stage.Open(str(HERE / GEMINI))
    root = stage.GetDefaultPrim()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    rng = Gf.Range3d()
    for child in root.GetChild("body").GetChildren():
        if child.GetName() == "BRACKET":
            continue            # tripod bracket; Kaya sits on its own shell ledge
        rng.UnionWith(cache.ComputeRelativeBound(child, root).ComputeAlignedRange())
    centre = np.array(rng.GetMidpoint())
    size = np.array(rng.GetSize())
    r = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)  # asset -> link
    t = -r @ centre
    # the driver's depth origin is the LEFT IR camera
    ir = UsdGeom.Xformable(root.GetChild("camera_ir_left")).GetLocalTransformation()
    left_ir = r @ np.array(ir.ExtractTranslation()) + t
    return r, t, r @ size, left_ir


# ----------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--roller-collider", choices=["sphere", "hull"], default="sphere",
                    help="sphere (default, NVIDIA Kaya's choice): smooth rolling, "
                         "tracks the commanded speed to 3%%. hull: convex hull of "
                         "the CAD barrel - looks more exact, but the cooked hull is "
                         "faceted and makes the rollers chatter (13%% speed loss).")
    args = ap.parse_args()

    x = read_xacro_properties(URDF)
    wheel_r = x["wheel_radius"]
    camera_mass = x["total_mass"] - x["base_mass"] - 3 * x["wheel_mass"]
    # The URDF gives the camera no inertial (its comment says it is carried by
    # base_link) but base_mass was computed as total - 3 wheels - camera, so the
    # URDF actually sums to 1.924 kg. The camera is put back here.
    assert abs(camera_mass - 0.164) < 1e-3, camera_mass

    cad_stage, parts = load_cad()
    groups = classify(parts, wheel_r)

    stage = Usd.Stage.CreateNew(str(HERE / OUT_ROBOT))
    geom_layer = Sdf.Layer.CreateNew(str(HERE / OUT_GEOM))
    stage.GetRootLayer().subLayerPaths.append("./" + OUT_GEOM)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    for layer in (stage.GetRootLayer(), geom_layer):
        layer.defaultPrim = ROOT[1:]
    stage.GetRootLayer().documentation = (
        "Kaya robot for Isaac Sim. GENERATED by build_kaya_usd.py - edit that, "
        "not this file.")

    # ---------------- geometry layer: visual meshes, in link frames
    stage.SetEditTarget(Usd.EditTarget(geom_layer))
    UsdGeom.Xform.Define(stage, ROOT)
    UsdGeom.Scope.Define(stage, f"{ROOT}/Looks")
    looks = make_looks(stage, [p for p in parts if p not in groups["camera"]])

    base_path = f"{ROOT}/base_link"
    used = set()
    UsdGeom.Xform.Define(stage, base_path)
    UsdGeom.Xform.Define(stage, f"{base_path}/visuals")
    for p in groups["base_link"]:
        write_mesh(stage, f"{base_path}/visuals/{unique_name(p, used)}",
                   p, np.eye(3), np.zeros(3), looks)

    links = {}      # name -> (translation in base_link, rotation in base_link)
    rollers = {}    # wheel -> [(roller link name, t, r, part)]
    for w, theta_deg in WHEEL_ANGLES_DEG.items():
        hub = groups[w]["hub"]
        plates = [p for p in hub if p.name.startswith("hub_plate")]
        centre = np.mean([p.centre for p in plates], axis=0)
        theta = math.atan2(centre[1], centre[0])
        assert abs((math.degrees(theta) - theta_deg + 180) % 360 - 180) < 0.5
        wr = rot_z(theta)                          # X radial OUT, like the URDF
        links[w] = (centre, wr)
        wl = f"{ROOT}/{w}_wheel_link"
        UsdGeom.Xform.Define(stage, wl)
        UsdGeom.Xform.Define(stage, f"{wl}/visuals")
        used = set()
        for p in hub:
            write_mesh(stage, f"{wl}/visuals/{unique_name(p, used)}", p, wr, centre, looks)

        axis = wr[:, 0]
        rollers[w] = []
        for k, p in enumerate(sorted(groups[w]["rollers"], key=lambda q: q.name), 1):
            radial = p.centre - centre
            radial -= axis * (radial @ axis)
            tangent = np.cross(axis, radial)
            tangent /= np.linalg.norm(tangent)
            rr = np.column_stack([tangent, axis, np.cross(tangent, axis)])
            name = f"{w}_roller_{k:02d}"
            rollers[w].append((name, p.centre, rr, p))
            UsdGeom.Xform.Define(stage, f"{ROOT}/{name}")
            UsdGeom.Xform.Define(stage, f"{ROOT}/{name}/visuals")
            write_mesh(stage, f"{ROOT}/{name}/visuals/roller", p, rr, p.centre, looks)

    # ---------------- robot layer: structure, physics, sensors
    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    root = UsdGeom.Xform.Define(stage, ROOT).GetPrim()
    stage.SetDefaultPrim(root)
    root.SetMetadata("kind", "component")

    # Floating-base articulation. Self-collision off: rollers of one wheel
    # overlap their neighbours by design, and nothing on this robot can
    # physically hit another part of itself.
    UsdPhysics.ArticulationRootAPI.Apply(root)
    physx(root, "PhysxArticulationAPI",
          **{"physxArticulation:enabledSelfCollisions": False,
             "physxArticulation:solverPositionIterationCount": 32,
             "physxArticulation:solverVelocityIterationCount": 4,
             # never let a parked robot fall asleep - it would ignore commands
             "physxArticulation:sleepThreshold": 0.0,
             "physxArticulation:stabilizationThreshold": 0.0})

    UsdGeom.Scope.Define(stage, f"{ROOT}/PhysicsMaterials")
    UsdGeom.Scope.Define(stage, f"{ROOT}/joints")
    rubber = physics_material(stage, f"{ROOT}/PhysicsMaterials/roller_rubber", ROLLER_FRICTION)
    physx(rubber.GetPrim(), "PhysxMaterialAPI")
    rubber.GetPrim().CreateAttribute("physxMaterial:frictionCombineMode",
                                     Sdf.ValueTypeNames.Token).Set(ROLLER_COMBINE)
    chassis_mat = physics_material(stage, f"{ROOT}/PhysicsMaterials/chassis", CHASSIS_FRICTION)

    # base_link: base_footprint is the root's origin (the floor), so a robot
    # placed at z=0 stands on its wheels.
    base = stage.GetPrimAtPath(base_path)
    set_pose(base, [0, 0, wheel_r], np.eye(3))
    UsdPhysics.RigidBodyAPI.Apply(base)
    fp = UsdGeom.Xform.Define(stage, f"{base_path}/base_footprint")
    set_pose(fp.GetPrim(), [0, 0, -wheel_r], np.eye(3))

    # camera_link, the ONE transform the URDF owns (the Orbbec driver owns the
    # rest of the camera chain at runtime).
    # camera_link is the LEFT IR lens, as the Orbbec driver defines it (see the
    # URDF). The camera BODY centre (camera_x/y/z) is what was measured; the
    # asset is placed by that, and camera_link is checked against the lens.
    body_t = np.array([x["camera_x"], x["camera_y"], x["camera_z"]])
    cam_r = rpy_matrix(x["camera_roll"], x["camera_pitch"], 0.0)
    g_r, g_body_t, g_size, left_ir = gemini_placement()   # in body-centre frame
    cam_t = np.array([x["camera_link_x"], x["camera_link_y"], x["camera_link_z"]])
    lens = body_t + cam_r @ left_ir
    assert np.abs(lens - cam_t).max() < 2e-4, f"URDF camera_link {cam_t} is not the lens {lens}"
    cam = UsdGeom.Xform.Define(stage, f"{base_path}/camera_link").GetPrim()
    set_pose(cam, cam_t, cam_r)
    g_t = g_body_t - left_ir                  # asset, relative to the lens
    gem = stage.DefinePrim(f"{base_path}/camera_link/gemini_335L", "Xform")
    gem.GetReferences().AddReference("./" + GEMINI)
    set_pose(gem, g_t, g_r)
    stage.OverridePrim(f"{gem.GetPath()}/body/BRACKET").SetActive(False)

    # Two corrections to NVIDIA's asset, both from measurements:
    # 1. Its depth camera clips at 1 m (clippingRange 1..1e6), so the floor
    #    and anything close read as inf. Orbbec's spec is 0.17-20 m.
    depth = stage.OverridePrim(f"{gem.GetPath()}/camera_ir_left/Camera_Depth_Left")
    depth.CreateAttribute("clippingRange", Sdf.ValueTypeNames.Float2).Set(
        Gf.Vec2f(DEPTH_RANGE_M))
    # 2. RGB: the robot streams 848x480 (a 16:9 crop of the 1280x800 sensor).
    #    Match the apertures to the robot's own /camera/color/camera_info so a
    #    sim image and a real one line up pixel for pixel. Lens distortion
    #    (plumb_bob, k1 -0.034) is not modelled - the sim is a pinhole.
    focal = 1.81            # mm, the asset's own focal length (unchanged)
    for cam_path, (w, h, fx, fy) in [("camera_rgb/Camera_rgb", RGB_INTRINSICS),
                                     ("camera_ir_left/Camera_ir_left", IR_INTRINSICS),
                                     ("camera_ir_left/Camera_Depth_Left", IR_INTRINSICS),
                                     ("camera_ir_right/Camera_ir_right", IR_INTRINSICS)]:
        cam_prim = stage.OverridePrim(f"{gem.GetPath()}/{cam_path}")
        cam_prim.CreateAttribute("horizontalAperture", Sdf.ValueTypeNames.Float).Set(w * focal / fx)
        cam_prim.CreateAttribute("verticalAperture", Sdf.ValueTypeNames.Float).Set(h * focal / fy)
    # the measured extrinsics, in the asset's axes (asset X = link Y = left)
    left = UsdGeom.Xformable(stage.GetPrimAtPath(f"{gem.GetPath()}/camera_ir_left"))
    left_t = np.array(left.GetLocalTransformation().ExtractTranslation())
    right = UsdGeom.Xformable(stage.GetPrimAtPath(f"{gem.GetPath()}/camera_ir_right"))
    right.ClearXformOpOrder()
    right.AddTranslateOp().Set(Gf.Vec3d(*(left_t - [STEREO_BASELINE_M, 0, 0])))
    rgb_x = UsdGeom.Xformable(stage.GetPrimAtPath(f"{gem.GetPath()}/camera_rgb"))
    rgb_t = rgb_x.GetLocalTransformation().ExtractTranslation()
    qx, qy, qz, qw = COLOR_IN_DEPTH_Q
    r_body = np.array(Gf.Matrix3d(Gf.Rotation(Gf.Quatd(qw, qx, qy, qz))).GetTranspose())
    rgb_x.ClearXformOpOrder()
    rgb_x.AddTranslateOp().Set(rgb_t)
    rgb_x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat(g_r.T @ r_body @ g_r))
    # 3. The image cameras sit 11.2 mm BEHIND the cover glass with a 5 mm
    #    near clip, so they rendered the inside of their own lens barrel.
    #    Clip just past the glass instead.
    for cam_path in ("camera_rgb/Camera_rgb", "camera_ir_left/Camera_ir_left",
                     "camera_ir_right/Camera_ir_right"):
        stage.OverridePrim(f"{gem.GetPath()}/{cam_path}").CreateAttribute(
            "clippingRange", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(0.02, 100.0))
    box = UsdGeom.Cube.Define(stage, f"{base_path}/camera_link/collision")
    box.CreateSizeAttr(1.0)
    box.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdGeom.Xformable(box).AddTranslateOp().Set(Gf.Vec3d(*(-left_ir)))
    UsdGeom.Xformable(box).AddScaleOp().Set(
        Gf.Vec3f(x["camera_box_x"], x["camera_box_y"], x["camera_box_z"]))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())

    # The Orbbec driver's frames: camera_depth_frame is the identity on
    # camera_link (both are the left IR lens). Kept under this name so a ROS
    # bridge can publish the same tree the real driver does.
    depth = UsdGeom.Xform.Define(stage, f"{base_path}/camera_link/camera_depth_frame")
    set_pose(depth.GetPrim(), [0, 0, 0], np.eye(3))     # identity, as in the driver
    # IMU where the driver says it is, oriented like its data frame, so sim
    # readings compare 1:1 with /camera/gyro_accel/sample.
    imu = stage.DefinePrim(f"{depth.GetPath()}/imu", "IsaacImuSensor")
    xf = UsdGeom.Xformable(imu)
    xf.AddTranslateOp().Set(Gf.Vec3d(*IMU_IN_DEPTH_FRAME))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(OPTICAL)
    imu.CreateAttribute("enabled", Sdf.ValueTypeNames.Bool).Set(True)
    for f in ("linearAcceleration", "angularVelocity", "orientation"):
        imu.CreateAttribute(f"{f}FilterWidth", Sdf.ValueTypeNames.Int).Set(1)

    # base mass = URDF base (CAD distribution, scaled to the scale reading)
    # + the camera the URDF left out, as a uniform box at camera_link.
    ib = np.array([[x["base_ixx"], 0, x["base_ixz"]],
                   [0, x["base_iyy"], 0],
                   [x["base_ixz"], 0, x["base_izz"]]])
    bx, by, bz = x["camera_box_x"], x["camera_box_y"], x["camera_box_z"]
    ic = camera_mass / 12 * np.diag([by**2 + bz**2, bx**2 + bz**2, bx**2 + by**2])
    base_parts = [(x["base_mass"], np.array([0, 0, x["base_com_z"]]), ib),
                  (camera_mass, body_t, cam_r @ ic @ cam_r.T)]
    couplers = []       # filled by the wheel loop, subtracted from base below

    # chassis colliders: convex hulls of the big parts only. Screws and PCBs
    # add cooking time and nothing a collision could ever depend on.
    for mesh in stage.GetPrimAtPath(f"{base_path}/visuals").GetChildren():
        ext = UsdGeom.Mesh(mesh).GetExtentAttr().Get()
        if np.prod(np.array(ext[1]) - np.array(ext[0])) > 12e-6:
            UsdPhysics.CollisionAPI.Apply(mesh)
            UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr("convexHull")
            bind_physics_material(mesh, chassis_mat)

    # wheels + rollers
    report = []
    for w in WHEEL_ANGLES_DEG:
        wt, wr = links[w]
        wl = stage.GetPrimAtPath(f"{ROOT}/{w}_wheel_link")
        set_pose(wl, np.array([0, 0, wheel_r]) + wt, wr)
        UsdPhysics.RigidBodyAPI.Apply(wl)

        # The URDF's wheel_mass is the WHEEL on a scale (hub plates + rollers,
        # no coupler). Split it by CAD volume x density, keeping the total
        # exact. The coupler is not in that weighing: it keeps its CAD mass,
        # and moves from base_link (where the URDF counts it) to this link,
        # because it turns with the wheel.
        raw = []
        for p in groups[w]["hub"] + [r[3] for r in rollers[w]]:
            vol, com, inertia = mesh_mass_properties(p.points, p.triangles())
            assert vol > 0, f"{p.name}: mesh not closed/outward ({vol})"
            raw.append((p, vol * p.density(), com, inertia * p.density()))
        is_coupler = {id(p): p.name.endswith("_coupler") for p, *_ in raw}
        is_roller = {id(p): p.name.startswith("roller_") for p, *_ in raw}
        by_part = {}
        for p, m, c, i in raw:
            if is_roller[id(p)]:
                vol = m / p.density()
                length = p.size.max()
                pin = math.pi * (ROLLER_PIN_DIAMETER / 2) ** 2 * length
                rm = (vol - pin) * RUBBER_DENSITY + pin * STEEL_DENSITY
                by_part[id(p)] = (rm, c, i * rm / m)
        frame_mass = x["wheel_mass"] - sum(v[0] for v in by_part.values())
        frame_cad = sum(m for p, m, *_ in raw if not is_coupler[id(p)] and not is_roller[id(p)])
        for p, m, c, i in raw:
            if is_coupler[id(p)]:
                by_part[id(p)] = (m, c, i)
            elif not is_roller[id(p)]:
                by_part[id(p)] = (m * frame_mass / frame_cad, c, i * frame_mass / frame_cad)
        couplers += [by_part[id(p)] for p, *_ in raw if is_coupler[id(p)]]

        hub = [by_part[id(p)] for p in groups[w]["hub"]]
        hm, hc, hi = combine(hub)
        set_mass(wl, hm, wr.T @ (hc - wt), wr.T @ hi @ wr)

        j = revolute(stage, f"{ROOT}/joints/{w}_wheel_joint", base_path, wl.GetPath(),
                     wt, wr @ rot_z(math.pi), [0, 0, 0], rot_z(math.pi))
        # positive = servo positive: joint X is the INWARD radial (decision 3)
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(SERVO_STIFFNESS)
        drive.CreateDampingAttr(SERVO_DAMPING)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateMaxForceAttr(SERVO_STALL_TORQUE_NM)
        drive.CreateTargetVelocityAttr(0.0)
        physx(j.GetPrim(), "PhysxJointAPI",
              **{"physxJoint:maxJointVelocity": SERVO_MAX_SPEED_DEG_S,
                 "physxJoint:jointFriction": 0.0})

        roller_m = 0.0
        for name, rt, rr, p in rollers[w]:
            rl = stage.GetPrimAtPath(f"{ROOT}/{name}")
            set_pose(rl, np.array([0, 0, wheel_r]) + rt, rr)
            UsdPhysics.RigidBodyAPI.Apply(rl)
            m, c, i = by_part[id(p)]
            roller_m += m
            set_mass(rl, m, rr.T @ (c - rt), rr.T @ i @ rr)

            if args.roller_collider == "hull":
                col = stage.GetPrimAtPath(f"{ROOT}/{name}/visuals/roller")
                UsdPhysics.MeshCollisionAPI.Apply(col).CreateApproximationAttr("convexHull")
            else:
                sph = UsdGeom.Sphere.Define(stage, f"{ROOT}/{name}/collision")
                sph.CreateRadiusAttr(float(wheel_r - np.linalg.norm(
                    (rt - wt) - wr[:, 0] * ((rt - wt) @ wr[:, 0]))))
                sph.CreatePurposeAttr(UsdGeom.Tokens.guide)
                col = sph.GetPrim()
            UsdPhysics.CollisionAPI.Apply(col)
            # millimetre-scale parts: the default contact offset (~2 cm) would
            # make contacts fire long before the roller reaches the floor
            physx(col, "PhysxCollisionAPI",
                  **{"physxCollision:contactOffset": 0.002,
                     "physxCollision:restOffset": 0.0})
            bind_physics_material(col, rubber)

            jr = revolute(stage, f"{ROOT}/joints/{name}_joint", wl.GetPath(), rl.GetPath(),
                          wr.T @ (rt - wt), wr.T @ rr, [0, 0, 0], np.eye(3))
            physx(jr.GetPrim(), "PhysxJointAPI",
                  **{"physxJoint:jointFriction": 0.0,
                     "physxJoint:armature": ROLLER_ARMATURE})
            bearing = UsdPhysics.DriveAPI.Apply(jr.GetPrim(), "angular")
            bearing.CreateTypeAttr("force")
            bearing.CreateStiffnessAttr(0.0)
            bearing.CreateDampingAttr(ROLLER_DAMPING)
        report.append((w, math.degrees(math.atan2(wt[1], wt[0])) % 360,
                       np.hypot(wt[0], wt[1]), hm, roller_m))

    # base_link = URDF base + camera - the three couplers now on the wheels
    base_m, base_com, base_i = combine(
        base_parts + [(-m, c, -i) for m, c, i in couplers])
    set_mass(base, base_m, base_com, base_i)

    geom_layer.Save()
    stage.GetRootLayer().Save()

    # ---------------- summary: the numbers worth eyeballing
    total = base_m + sum(r[3] + r[4] for r in report)
    print(f"wrote {HERE / OUT_ROBOT}\n      {HERE / OUT_GEOM}")
    print(f"roller collider: {args.roller_collider}")
    print(f"base_link  {base_m:.4f} kg (URDF base {x['base_mass']:.4f} + camera "
          f"{camera_mass:.4f} - 3 couplers {sum(c[0] for c in couplers):.4f}), "
          f"COM {np.round(base_com, 4)} m")
    for w, ang, rad, hm, rm in report:
        print(f"{w:8s}   at {ang:5.1f} deg, r = {rad * 1000:.1f} mm, "
              f"hub+coupler {hm * 1000:.1f} g + 12 rollers {rm * 1000:.1f} g")
    print(f"TOTAL      {total:.4f} kg   (scale: {x['total_mass']:.3f} kg)")
    print(f"camera     camera_link = left IR lens {np.round(cam_t, 5)}, body centre {np.round(body_t, 5)}, asset box {np.round(g_size, 4)} m "
          f"(URDF box {bx}, {by}, {bz})")
    assert abs(total - x["total_mass"]) < 1e-4


if __name__ == "__main__":
    main()
