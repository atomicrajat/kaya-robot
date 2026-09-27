# Kaya for Isaac Sim

`kaya.usda` is Kaya as an Isaac Sim robot component: articulated, with mass,
colliders, servo-like wheel drives, simulated omni rollers, and the Orbbec
Gemini 335L (RGB, stereo IR, depth, IMU) mounted where the real one is.
Every number in it was measured on the real robot - see the tables below.

## After cloning

Needs **Isaac Sim 6.0.1** (tested on 6.0.1, Ubuntu 24.04, RTX 5090). Nothing
else: no pip installs, no network - the camera asset is vendored and the
generated USD is committed. Run everything with Isaac Sim's own Python;
`$ISAAC_SIM` below is wherever it is installed (here `~/isaac-sim`).

    cd kaya/kaya_description/usd
    $ISAAC_SIM/python.sh kaya_sim.py --test      # 13 checks, headless, ~1 min - run this first
    $ISAAC_SIM/python.sh kaya_sim.py             # GUI: floor, lights, props, teleop

In the GUI, click the viewport, then drive: **W/S** forward/back, **A/D**
strafe, **Q/E** rotate, **SPACE** stop. The window "Kaya - Gemini 335L RGB"
is the robot's own camera. Stop -> Play, or accepting "Fetch" when the USD
changes on disk, is handled: the driver re-attaches (and restarts the
timeline itself if physics was torn down while playing).

Other tools, all taking `--device cuda` for GPU physics (what training uses):

    $ISAAC_SIM/python.sh step_compare.py         # replay the real robot's step test, sim vs real
    $ISAAC_SIM/python.sh tilt_test.py            # re-run the real tilt test (friction calibration)

## Using Kaya in your own scene - read this

The USD carries the robot. **Four things it cannot carry** live in the scene
or in Python, and a scene that skips them gets a different robot.
`kaya_isaac.py` does all four; use it rather than re-doing them:

| needed | why | in `kaya_isaac.py` |
|---|---|---|
| stage in metres, Z up | a reference does not rescale or rotate | `new_stage()` / `check_stage()` |
| **240 Hz physics** | the physics timestep is the scene's; at Isaac's default 60 Hz the ~3 g rollers chatter | `setup_physics()` |
| a floor with a friction material | the robot's grip is min(roller 0.40, floor) | `add_floor()` |
| **drive through `KayaDriver`** | plain velocity targets let the wheels creep on slopes and drop the real 0.1 s servo lag | `KayaDriver` |

    import kaya_isaac as kaya                  # after SimulationApp starts
    import isaacsim.core.experimental.utils.app as app_utils

    stage = kaya.new_stage()                   # or kaya.check_stage(your_stage)
    kaya.add_floor(stage)
    kaya.add_kaya(stage, "/World/kaya")        # z = 0 stands it on the floor
    kaya.setup_physics(device="cpu")           # or "cuda"
    app_utils.play()
    robot = kaya.KayaDriver("/World/kaya")
    while simulation_app.is_running():
        robot.drive(0.1, 0.0, 0.0)             # vx, vy m/s, w rad/s - what /cmd_vel means
        robot.update()                         # once per frame
        simulation_app.update()
    # sensors: kaya.cameras("/World/kaya") -> prim paths; robot.read_imu(noisy=True)

Keep the `usd/` folder together: `kaya.usda` loads its geometry and the
camera by relative path. Dragging it into the GUI by hand works too
(`File > Add Reference`), but then you own the four rows above yourself.

## Rebuild after changing the URDF or the CAD

    $ISAAC_SIM/python.sh build_kaya_usd.py && $ISAAC_SIM/python.sh kaya_sim.py --test

Never edit `kaya.usda` or `kaya_geometry.usdc` by hand; they are generated.
The numbers come from `../urdf/kaya.urdf.xacro`, the shapes from
`source/kaya_robot_ttb.usdz` (the Fusion export). Measurements are recorded
in `../cad/kaya_dimensions.yaml`, go into the URDF, then rebuild here. After
any change to masses or rollers, re-run `tilt_test.py` too - the friction
calibration drifted once when roller masses changed after it.

## Files

| file | what |
|---|---|
| `kaya.usda` | **the asset.** Text, so every physics value can be read and diffed |
| `kaya_geometry.usdc` | visual meshes, sublayered by `kaya.usda` |
| `kaya_isaac.py` | **scene setup + `KayaDriver`** - what a scene needs beyond the USD |
| `kaya_servos.py` | `WheelServos`: drive kinematics + the measured servo chain |
| `kaya_imu_noise.py` | the real Gemini IMU's noise, bias and scale |
| `kaya_sim.py` | demo scene + teleop; `--test` runs the 13 checks |
| `step_compare.py`, `tilt_test.py` | re-run the real robot's step and tilt tests in sim |
| `build_kaya_usd.py` | the generator; its docstring lists every non-obvious decision |
| `data/` | the real step-test recording `step_compare.py` replays |
| `source/kaya_robot_ttb.usdz` | Fusion 360 export, 2026-09-08 |
| `sensors/orbbec_gemini_335L/` | NVIDIA's Gemini 335L asset (Isaac 6.0), vendored so it loads offline |

## Prim tree

    /kaya                          ArticulationRoot, origin = base_footprint (floor)
      base_link                    rigid body, 1.720 kg (chassis + camera)
        visuals/                   upper_deck, shell, servos, battery, jetson...
        base_footprint             frame only, on the floor
        camera_link                the LEFT IR lens (as the Orbbec driver defines it), 4.89 deg nose UP
          gemini_335L              NVIDIA asset
            camera_rgb/Camera_rgb            848x480, the robot's real intrinsics
            camera_ir_left/Camera_ir_left    left IR, 95 mm baseline
            camera_ir_left/Camera_Depth_Left depth, 0.17-20 m
            camera_ir_right/Camera_ir_right
          camera_depth_frame       identity on camera_link, as in the driver's /tf_static
            imu                    IsaacImuSensor, at the driver's offset, optical axes
          collision                box, invisible
      wheel_{1,2,3}_wheel_link     frame/hub 73 g + coupler 11 g
      wheel_{1,2,3}_roller_{01..12} one link per roller, ~3.2 g   (wheel = 112 g, weighed)
      joints/
        wheel_N_wheel_joint        position servo: stall at 0.5 deg error, 2.45 N.m cap, 6.28 rad/s cap
        wheel_N_roller_MM_joint    free, bearing damping + armature
      PhysicsMaterials/            roller rubber, chassis
      Looks/                       visual materials from the CAD

Wheel names follow the servo ids, not the CAD's names:
`wheel_1` = servo 1 = front-right, 300 deg; `wheel_2` = front-left, 60;
`wheel_3` = rear, 180.

## Things that behave differently from what you might assume

- **Positive wheel joint velocity = positive servo Goal Speed.** The joint axis
  points *inward*, so the drive node's kinematics
  `v_i = -sin(th) vx + cos(th) vy + L w` applies unchanged. The URDF's
  `axis="1 0 0"` points outward, which is the opposite sign; it has only ever
  animated RViz. `--test` proves the sign in sim: forward, strafe and rotate
  each go the right way.
- **Wheels at 143 mm (CAD geometry), not 138 mm (URDF).** 138 is the calibrated
  kinematic L, which absorbs real slip. The sim gets real geometry. Its
  rotation then tracks at 99% using L = 143.
- **The URDF weighs 1.924 kg; the robot weighs 2.088.** The URDF leaves out the
  164 g camera. The USD puts it back.
- **Wheels are position servos, fed a moving target.** The real ST3020 closes
  its loop on its encoder and holds when stopped. A velocity-only drive in
  PhysX crept 0.08 rad/s on a slope whatever its damping; that is solver
  residual, and only position feedback cancels it. PhysX also reports these
  joint angles wrapped into (-2pi, 2pi], so `WheelServos` counts the wraps
  itself.
- **Rollers use sphere colliders, not the CAD barrel hull.** The hull looks more
  exact, but measured worse: 13% speed loss and rollers spinning at 110 rad/s,
  against 2% loss and a physically correct 16 rad/s with spheres.
- **NVIDIA's camera asset needed three fixes**, all applied as overrides:
  - depth near clip was 1 m (now 0.17 m, Orbbec's spec);
  - RGB aperture now matches the robot's `camera_info`;
  - the image cameras rendered their own lens barrel (near clip was 5 mm, behind the glass).

## Last `--test` (2026-09-27, CPU; GPU gives the same 13/13)

    PASS  mass (PhysX, all links)            2.0880 kg vs 2.088 scale, 40 links, 39 dofs
    PASS  rest height of base_link           z = 37.0 mm (wheel radius 37.0)
    PASS  rest tilt                          0.00 deg
    PASS  IMU at rest vs the real Gemini IMU sim [-0.16, -9.769, 0.836] real [-0.1611, -9.7531, 0.8337] m/s^2
    PASS  forward 0.10 m/s                   vx +0.099 vy -0.000 m/s  w -0.000 rad/s
    PASS  strafe left 0.10 m/s               vx +0.000 vy +0.099 m/s  w +0.000 rad/s
    PASS  rotate CCW 0.5 rad/s               vx +0.000 vy -0.000 m/s  w +0.495 rad/s
    PASS  holds on a 10 deg slope            moved 0.03 mm in 2 s, wheels turned 0.06 deg
    PASS  stereo baseline                    94.86 mm (the camera's own calibration: 94.86)
    PASS  depth fx at 848x480                408.16 px (camera_info: 408.16)
    PASS  depth to wall (centre pixel)       1.4806 m, expected 1.4806 m
    PASS  depth sees the floor (< 1 m)       bottom-centre pixel 0.354 m
    PASS  camera pitched UP 4.89 deg         optical axis +4.89 deg on the chassis (chassis stance -0.19 deg), depth top 1.517 > bottom 0.823

## Calibrated from the real robot

| value | how |
|---|---|
| wheel mass 112 g | weighed, 2026-09-27 (without the shaft coupler) |
| roller friction mu 0.40 | real robot slides at 15 deg on a table, camera facing downhill; `tilt_test.py --yaw 0` gives 14.6 deg (CPU) / 15.6 deg (GPU) |
| IMU position + axes | the Orbbec driver's /tf_static, from `~/bags/phase1_imu` on the robot |
| camera pitch AND roll | sim IMU at rest (-0.160, -9.769, +0.836) vs real (-0.161, -9.753, +0.834) m/s^2 |
| IR/depth intrinsics, stereo baseline, colour extrinsics | robot driver's camera_info + /tf_static at 848x480, 2026-09-27: fx 408.16, baseline 94.86 mm, colour rotated ~0.3 deg |
| IMU noise, bias, scale | 30 s still on the robot: see `kaya_imu_noise.py` (the sim IMU itself is perfect; apply `corrupt()` to its readings) |
| servo response (30 Hz loop, 27.5 ms delay, wn 31, zeta 0.60) | real step test 2026-09-27, `data/real_step_2026-09-27.json`; `step_compare.py` replays it: start 0.101 vs 0.101 s, stop 0.134 vs 0.134 s, overshoot 16 vs 20%, 201.8 vs 202.1 mm |
| floor (shiny tile) mu 0.40 | owner: no grippier than the table. An upper bound - lower it for a slipperier floor |

## Not measured yet (the sim uses a stand-in)

| value | stand-in | how to measure |
|---|---|---|
| roller vs frame split of the 112 g wheel | rubber barrel + 2 mm steel pin = ~3.2 g per roller | weigh one roller |
| servo speed at the robot's supply voltage | 60 RPM (12 V spec) | command max speed, read Present Speed |

## Found on the REAL robot while building this (not yet changed there)

1. **Camera roll sign was wrong in the URDF** (-0.94 deg; the camera is really
   rolled +0.94 deg, right side lower). Fixed in `kaya.urdf.xacro` on master,
   proven against the real IMU above. Not deployed: the saved maps were built
   with the old TF, so re-check relocalisation when it goes out.
2. **The URDF's camera_link was not the driver's camera_link** - CONFIRMED and
   fixed in `kaya.urdf.xacro` on master. The driver treats camera_link as the
   left IR lens; the URDF had it at the body centre, 47.5 mm right of the lens.
   Measured with a bottle on the centre line at 0.5 m and 1.0 m: the old TF put
   it at -44.5 / -38.9 mm (fit: -51 mm offset, +0.65 deg yaw - an offset, not a
   yaw); the fixed TF at +3.0 / +8.6 mm. camera_link now IS the lens, in both
   the URDF and the USD. Not deployed: every saved map was built with the old
   TF, so plan a remap when it goes out.

## CPU vs GPU physics (2026-09-27)

Everything above passes on both. Differences worth knowing:

| | CPU | GPU (cuda:0) |
|---|---|---|
| `--test` suite | 13/13 | 13/13 |
| step test RMSE vs real | 0.155 rad/s | 0.150 rad/s |
| step overshoot (real +16%) | +17% | +12% |
| tilt slide angle at mu 0.40 | 14.6 deg | 15.6 deg |

The GPU solver holds static friction ~0.9 deg longer at every mu; 0.40 splits
the difference. On GPU, USD transforms are not written back from physics
while running (results stay on the GPU), so read poses from the Articulation
API, not from USD prims. `WheelServos` copies joint states to numpy every
step: fine for one robot, needs vectorising on the GPU for many.
