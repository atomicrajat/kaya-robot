# Getting started: from a built Kaya to a working robot

This takes you from **a built Kaya with a freshly flashed Jetson** to where this
project is now:

- the robot drives omnidirectionally from `/cmd_vel`;
- it maps a space in 3D, localises itself in it with cuVSLAM, and navigates
  with Nav2;
- a web dashboard shows the live map and position;
- you can ask it in words ("take me where I can cook");
- a calibrated copy of it runs in Isaac Sim, and a policy trained there plays
  football on the real floor.

Steps are in order. Stop at the first one that fails, and read the file it
points to: most failures on this hardware are already written up.

> **Status of this guide (2026-10-12).** Every command here is one that has run
> on the project's robot, and every script it names is in the repo. The
> sequence as a whole has **not** yet been followed end-to-end by someone
> building a second Kaya. Three scripts were pulled off the robot or written
> from the documented configuration on 2026-10-12 and are marked **CHECK** at
> the step that first runs them: read them before trusting them.

---

## 0. What you need

**The robot.** Measurements are in `kaya_description/cad/kaya_dimensions.yaml`,
build notes in `kaya_description/cad/CAD_CONTEXT.md`.

| part | this build |
|---|---|
| compute | Jetson Orin Nano **8 GB** module on the p3768 carrier (JetPack 7.2.1) |
| camera | Orbbec **Gemini 335L**, on the front ledge, pitched ~5 deg up |
| drive | 3x Waveshare **ST3020** serial-bus servos, IDs 1/2/3, 1 Mbaud on `/dev/ttyACM0` |
| wheels | 3x 74 mm double-row omni wheels, radial axles, at 60 / 180 / 300 deg |
| power | 20 V 2.0 Ah power-tool pack, with buck converters |
| wifi | Intel PCIe card (JetPack 7 ships no driver - provisioning builds one) |

**A workstation**, for the dashboard, perception and simulation. This one is
Ubuntu 24.04 with ROS 2 Jazzy and an RTX 5090 (24 GB). Perception wants about
16 GB of VRAM; the simulator runs on less; RL training used all 24. It must be
on the **same subnet** as the robot, because ROS 2 topics cross over DDS on
`ROS_DOMAIN_ID` 0. ROS is not on the path by default: `source
/opt/ros/jazzy/setup.bash` in every shell, on both machines. Reach the robot by
IP; mDNS (`kaya.local`) failed repeatedly here.

**Battery.** About 2.5 h with the full stack, 1.8 h after a partial charge, and
the servo voltage gives no warning before it dies. Charge fully, and commit
work as you verify it: a dead battery has stranded verified work once.

---

## 1. Flash the Jetson (host PC)

Flash JetPack 7.2.x with the **`jetson-orin-nano-devkit-super`** board config.
Without it the GPU stays capped at 624.75 MHz, and a symlinked nvpmodel only
looks right. See `robot_setup/SETUP_LOG.md` sections 3-4. This is the one step
no script can do for you.

## 2. Provision the Jetson

On the robot:

    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    git clone <this repo> kaya
    cd kaya/robot_setup
    ./kaya_provision.sh --dry-run     # see what it will do
    ./kaya_provision.sh               # safe to re-run; skips finished stages

It works through these stages in order: `precheck`, `swap`, `groups`,
`usbgadget`, `docker`, `jetpack`, `ros2`, `isaacros` (cuVSLAM + nvblox),
`wifi` (builds iwlwifi), `orbbec` (the camera driver plus its firmware patch),
and `verify`. Log out and back in afterwards, then reboot, then:

    ./kaya_check.sh                   # health check: power mode, camera, servos, wifi

Every script in this guide runs from this checkout, `~/ros2_ws/src/kaya`.
Nothing is copied to the home directory.

## 3. Build Kaya

    sudo apt install ros-jazzy-foxglove-bridge python3-serial
    cd ~/ros2_ws
    MAKEFLAGS="-j2" colcon build --parallel-workers 1
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

Always build with those caps. An unconstrained build on 8 GB runs out of
memory hard enough to lock out ssh (`robot_setup/SETUP_LOG.md` section 8.1).
Provisioning does not install `foxglove_bridge` or `python3-serial`, but
`nav.launch.py` and the servo driver both need them.

## 4. Servos: IDs and wheel mapping

Each servo must carry the right ID for its wheel, because the drive's
kinematics depend on it:

| servo ID | wheel | angle (from forward, counter-clockwise) |
|---|---|---|
| 1 | front-right | 300 deg |
| 2 | front-left | 60 deg |
| 3 | rear | 180 deg |

"Forward" is where the camera looks. New servos ship as ID 1, so set the IDs
with one servo connected at a time, then pulse each one with the robot on
blocks and watch which wheel turns:

    source ~/ros2_ws/install/setup.bash
    python3 kaya_drive/tools/servo_set_id.py --from 1 --to 2 --yes   # CHECK: untested on hardware
    python3 kaya_drive/tools/wheel_probe.py 1 300 1.0                # the script that resolved this mapping

A positive Goal Speed on every servo must push the robot counter-clockwise,
seen from above; that was verified on this robot with no per-wheel sign flip
(`kaya_description/cad/kaya_dimensions.yaml`, section `servo_wheel_map`). Sign
errors show up in the next step as a robot that strafes when told to rotate.

## 5. First drive, on blocks

Lift the wheels off the floor, then:

    ros2 launch kaya_drive drive.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard     # second terminal

Check that forward drives forward, sideways strafes and rotation rotates, then
put it on the floor. `/odom` and `/joint_states` should follow. The drive stops
on its own if commands stop for 0.5 s.

## 6. Calibrate YOUR robot

The numbers in this repo belong to **this** robot. For yours, re-measure the
following, record them in `kaya_description/cad/kaya_dimensions.yaml`, carry
them into `kaya_description/urdf/kaya.urdf.xacro`, and rebuild.

| what | how | why it matters |
|---|---|---|
| total mass | kitchen scale | the URDF and the sim scale every inertia to it |
| camera pitch and roll | robot on a level floor; average `/camera/gyro_accel/sample` for ~30 s; pitch and roll come from the gravity vector (method in the URDF comments). **Mind the roll sign:** it was once flipped converting to REP-103 | a wrong pitch tilts the whole map; a wrong roll tilts depth sideways |
| rotation constant L | spin ~10 turns at a constant rate; compare the odometry's turns with a counted true rotation (`kaya_dimensions.yaml`, `wheels.kinematic_L_mm`) | scales every rotation in odometry |
| wheel rolling radius | roll 10 revolutions under load and measure the distance | scales every straight-line distance |
| camera_link | the **left IR lens**, where the Orbbec driver puts it - not the camera body centre. Derive it from the body position as the URDF does; check with an object on the centre line | an error here shifts every depth point sideways (it was 5 cm) |

Then run `python3 kaya_bringup/tools/spin_translation_test.py` with the SLAM
stack up (step 7): one full turn in place should end within a couple of
centimetres of where it started (this robot: 1.7 cm).

## 7. Map a space

Clear the space of people, since mapping fuses everything in permanently:

    kaya_bringup/tools/kaya_run.sh start mapping:=true nav2:=false

The stack starts the wheel driver itself (`enable_drive:=true` is the
default), so just run `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
and drive slowly around the whole space. Then save all three parts of the map
into one folder (the cuVSLAM part has been saved this way many times; the 2D
and nvblox parts were separate commands until 2026-10-12, **CHECK**):

    ros2 run kaya_bringup kaya_save_map ~/kaya_maps/home
    #   ~/kaya_maps/home/cuvslam/       where the robot IS    (cuVSLAM relocalisation)
    #   ~/kaya_maps/home/home_2d.*      what Nav2 plans on    (2D grid)
    #   ~/kaya_maps/home/home.nvblx     what the SPACE is     (nvblox volume, for looking at)
    kaya_bringup/tools/kaya_run.sh stop

Always stop with `kaya_run.sh stop`. Stopping `ros2 launch` directly orphans
the containers and fills the board's memory (`kaya_bringup/tools/README.md`).
Put a piece of tape on the floor where the robot stood when mapping started:
that is **the mark**, and everything below depends on it.

## 8. Navigate on the map

Put the robot back on the mark, then:

    kaya_bringup/tools/kaya_nav.sh home                 # CHECK: the robot's own script, pulled 2026-10-12
    kaya_bringup/tools/kaya_nav.sh home --relocalise    # localise into the saved map instead of the mark

`kaya_nav.sh` is the configuration that worked, with the reason for every flag
in its header: the saved 2D grid as the global costmap, nvblox and colour off,
the board in MAXN_SUPER. Wait for `map frame is now anchored` in
`~/kaya_run.log`, then send goals from RViz or the dashboard. Stop with
`kaya_run.sh stop`.

Prefer the mark. Relocalisation worked to 5.9 cm from a 5 m wrong hint the
afternoon the map was made, and stopped working once the furniture moved
(`ROADMAP.md`, "the relocalisation recovery radius is not fixed").

To measure where the robot actually ends up against each goal, run
`python3 kaya_bringup/tools/goal_watch.py` alongside.

## 9. Workstation: dashboard

    source /opt/ros/jazzy/setup.bash
    ssh-copy-id <you>@<robot-ip>                       # kaya.sh logs in without a password
    export KAYA_ROBOT=<you>@<robot-ip>
    export KAYA_BASE_MAP=/path/to/home_2d              # the .pgm/.yaml stem from step 7, copied here
    kaya_bringup/tools/kaya.sh                         # stack on the robot, wait for anchor, dashboard
    # dashboard: http://localhost:8080                 stop: kaya_bringup/tools/kaya.sh stop

`kaya.sh` runs `kaya_nav.sh` on the robot (`KAYA_NAV_ARGS="home --relocalise"`
to change what it launches), waits for the anchor, then prints whether the
robot is **ON THE MARK**. The dashboard keeps keyframes in the folder containing
this repo; set `KAYA_WORKSPACE` to put them elsewhere. Details in
`dashboard/README.md`.

## 10. Workstation: perception ("take me where I can cook")

    python3 -m venv --system-site-packages ../perception_venv
    ../perception_venv/bin/pip install -r perception/requirements.txt \
        --extra-index-url https://download.pytorch.org/whl/cu128
    # scene questions also need Ollama with a vision model:
    ollama pull qwen2.5vl:7b

Record keyframes on a drive with `perception/keyframe_store.py`, which saves a
pose-tagged image every ~1 m or 45 deg, then caption them once with
`perception/scene_memory.py`. The dashboard's Ask box then answers from the
captions in about a second. Keyframe poses belong to the frame they were
recorded in, so start every session from the mark.

## 11. Workstation: simulation

Install Isaac Sim 6.0.1, then follow `kaya_description/usd/README.md`:

    cd kaya_description/usd
    $ISAAC_SIM/python.sh kaya_sim.py --test       # 13 checks
    $ISAAC_SIM/python.sh kaya_sim.py              # drive it with W/A/S/D/Q/E

If you changed any numbers in step 6, rebuild the sim robot with
`build_kaya_usd.py` and re-run `tilt_test.py` and `step_compare.py` against your
own robot's tilt and step tests.

## 12. Workstation: train a policy in Isaac Lab

`kaya_lab/README.md`, sections 1-3: check throughput, train "drive to a point"
(three minutes to 100%), then the ball task. Needs Isaac Lab 3.0.0 beside the
Isaac Sim above.

## 13. The robot plays football

`kaya_lab/README.md`, section 4: build the YOLO engine once, print the three
markers, set up the box and the ball, and run `ball_policy_node.py` with the
committed `policies/policy_v24.npz`. The node runs from this checkout; a
`git pull` on the robot is the whole deploy.

---

## Where things are written down

| question | file |
|---|---|
| why a setting is what it is, and what is still open | `ROADMAP.md` |
| what was learned, concept by concept | `LEARNINGS.md` |
| JetPack / driver / wifi problems | `robot_setup/SETUP_LOG.md` |
| a diagnostic for a symptom | `kaya_bringup/tools/README.md` |
| the robot's dimensions and how each was measured | `kaya_description/cad/` |
| the sim, and what a scene must supply | `kaya_description/usd/README.md` |
| RL: the plan and every result | `SIM_RL_PLAN.md`, `kaya_lab/LEARNING_LOG.md` |
