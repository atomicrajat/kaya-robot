# Kaya for Isaac Sim

`kaya.usda` is Kaya as an Isaac Sim robot component: articulated, with mass,
colliders, servo-like wheel drives, simulated omni rollers, and the Orbbec
Gemini 335L (RGB, stereo IR, depth, IMU) mounted where the real one is.
Isaac Sim 6.0.1, on the workstation (`~/isaac-sim`).

## Run it

    cd ~/work_projects/kaya_robot/kaya/kaya_description/usd
    ~/isaac-sim/python.sh kaya_sim.py            # GUI: floor, lights, props, teleop
    ~/isaac-sim/python.sh kaya_sim.py --test     # headless checks, ~30 s
    ~/isaac-sim/python.sh tilt_test.py --mu 0.4 0.5   # re-run the real tilt test in sim
    ~/isaac-sim/python.sh step_compare.py             # replay the real step test, sim vs real

In the GUI, click the viewport, then drive: **W/S** forward/back, **A/D**
strafe, **Q/E** rotate, **SPACE** stop. The second window, "Kaya - Gemini
335L RGB", is the robot's own camera.

**Loading it by hand** in any Isaac Sim scene:
1. `File > Open` an empty stage, or your own scene.
2. `Create > Physics > Physics Scene` if the scene has none, and
   `Create > Physics > Ground Plane`.
3. Drag `kaya.usda` from the Content browser into the viewport, or use
   `File > Add Reference`. It lands with base_footprint at the drop point, so
   z = 0 is standing on the floor.
4. Press Play. To see through the camera, set a viewport's camera (the camera
   icon at the top of the viewport) to
   `.../kaya/base_link/camera_link/gemini_335L/camera_rgb/Camera_rgb`.
5. To drive it from your own script, use `kaya_servos.WheelServos`: it turns
   speed commands into servo position targets on every physics step. Plain
   velocity targets let the wheels creep under load (see below).

## Rebuild after changing the URDF or the CAD

    ~/isaac-sim/python.sh build_kaya_usd.py && ~/isaac-sim/python.sh kaya_sim.py --test

Never edit `kaya.usda` or `kaya_geometry.usdc` by hand; they are generated.
The numbers come from `../urdf/kaya.urdf.xacro`, the shapes from
`source/kaya_robot_ttb.usdz` (the Fusion export). Measurement changes go
into the URDF (via `cad/kaya_dimensions.yaml`), then rebuild here.

## Files

| file | what |
|---|---|
| `build_kaya_usd.py` | the generator; its docstring lists every non-obvious decision |
| `kaya.usda` | **the asset.** Text, so every physics value can be read and diffed |
| `kaya_geometry.usdc` | visual meshes, sublayered by `kaya.usda` |
| `kaya_sim.py` | spawn scene + teleop + `--test` verification |
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
  rotation then tracks at 98% using L = 143.
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

## Last `--test` (2026-09-27)

    PASS  mass (PhysX, all links)      2.0880 kg vs 2.088 scale, 40 links, 39 dofs
    PASS  rest height of base_link     z = 37.0 mm (wheel radius 37.0)
    PASS  forward 0.10 m/s             vx +0.098 vy +0.000 m/s  w +0.001 rad/s
    PASS  strafe left 0.10 m/s         vx -0.000 vy +0.098 m/s  w +0.000 rad/s
    PASS  rotate CCW 0.5 rad/s         w +0.491 rad/s
    PASS  depth to wall (centre pixel) 1.4798 m, expected 1.4798 m
    PASS  depth sees the floor         0.344 m
    PASS  camera pitched UP 4.89 deg   optical axis +4.89 deg

## Calibrated from the real robot

| value | how |
|---|---|
| wheel mass 112 g | weighed, 2026-09-27 (without the shaft coupler) |
| roller friction mu 0.39 | real robot slides at 15 deg on a table, camera facing downhill; `tilt_test.py --yaw 0` reproduces 15.07 deg |
| IMU position + axes | the Orbbec driver's /tf_static, from `~/bags/phase1_imu` on the robot |
| camera pitch AND roll | sim IMU at rest (-0.160, -9.769, +0.836) vs real (-0.161, -9.753, +0.834) m/s^2 |
| IR/depth intrinsics, stereo baseline, colour extrinsics | robot driver's camera_info + /tf_static at 848x480, 2026-09-27: fx 408.16, baseline 94.86 mm, colour rotated ~0.3 deg |
| IMU noise, bias, scale | 30 s still on the robot: see `kaya_imu_noise.py` (the sim IMU itself is perfect; apply `corrupt()` to its readings) |
| servo response (30 Hz loop, 27.5 ms delay, wn 31, zeta 0.60) | real step test 2026-09-27, `data/real_step_2026-09-27.json`; `step_compare.py` replays it: start 0.101 vs 0.101 s, stop 0.134 vs 0.134 s, overshoot 16 vs 20%, 201.8 vs 202.1 mm |
| floor (shiny tile) mu 0.39 | owner: no grippier than the table. An upper bound - lower it for a slipperier floor |

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
