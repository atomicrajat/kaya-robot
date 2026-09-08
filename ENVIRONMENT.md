# Development environment

Kaya development moved from the MacBook to the Ubuntu workstation on
2026-09-08. **The Mac copy at `~/work/kaya_robot` is a stale mirror — do not
edit it.** Everything below describes the machine you are on now.

`ROADMAP.md` is unchanged and still the plan. This file only covers the
machine.

---

## The machines

| | dev workstation | the robot |
|---|---|---|
| host | `rajatmr-rog-strix-ttb` | `kaya` |
| user | `rajatmr` | `orin-nano` |
| LAN | 192.168.1.2 | **192.168.1.28** |
| tailscale | 100.69.143.0 | 100.110.174.88 |
| project | `~/work_projects/kaya_robot` | `~/ros2_ws/src/kaya` |
| OS | Ubuntu 24.04.4, kernel 7.0.0-30 | JetPack 7.2.1 (L4T R39) |
| GPU | RTX 5090 Laptop, **24 GB** | Orin Nano 8 GB, no DLA |
| ROS | Jazzy at `/opt/ros/jazzy` | Humble |

The path is `work_projects` — plural, with an underscore. Not `~/work/`, which
is where it lived on the Mac.

Both machines are on the same 192.168.1.0/24 subnet, so the robot is reachable
directly — verified from here. Reach it **by IP**; `kaya.local` mDNS resolution
failed repeatedly across the 2026-09-07/08 sessions.

    ssh orin-nano@192.168.1.28

---

## Machine setup — done

Both manual steps this file used to list are complete as of 2026-09-08:

- **This machine is authorised on the robot.** `ssh-copy-id` has been run;
  `ssh orin-nano@192.168.1.28` and `git fetch`/`push` to `origin` no longer
  prompt for a password.
- **Git identity is set** to `Rajat <atomicrajat619@gmail.com>`, matching the
  repo history.

Commits in this project carry **no AI author or co-author trailer** — no
`Co-Authored-By: Claude`, no "Generated with Claude Code" footer, no session
link.

---

## What is different from the Mac

- **ROS 2 Jazzy is installed but not on `PATH`.** Source it in every shell:

      source /opt/ros/jazzy/setup.bash

  This is genuinely new capability — the Mac had no ROS at all. Anything that
  used to mean "ssh in and run it on the robot" can now run here: `rviz2`,
  `ros2 topic echo` against the robot's Humble stack over the LAN, and the
  Phase 8 dashboard work. Note the version gap: **Jazzy here, Humble on the
  robot.** They interoperate over DDS for standard message types, but do not
  expect to build the robot's packages here unmodified.

- **`rsync` from the Mac to here does not work.** macOS ships openrsync, which
  fails against Ubuntu's rsync 3.2.7 while parsing exclude rules
  (`buffer overflow: recv_rules`). The transfer that worked was tar over ssh.
  If you ever pull from the Mac again, delete the `._*` AppleDouble files
  afterwards — bsdtar recreates them on extraction and they corrupt
  `git status` when they land in `.git/objects/pack/`.

- **`meshlab` is not installed** for viewing `maps/*/home.ply`.

- **The GPU changes a roadmap decision.** `ROADMAP.md` Phase 10 ranks the cloud
  vision model first and lists "Cosmos 3 Nano on a workstation" second, hedged
  with *"if a machine with enough VRAM is available"*. That machine is this
  one: 24 GB VRAM, with CUDA, Docker + NVIDIA runtime, ollama, Isaac Sim and
  IsaacLab already installed. A 16B model at INT4/FP8 fits comfortably — it was
  ruled out for the Jetson's 8 GB shared with a 3.7 GB nav stack, which does
  not apply here. Both options are open now. The robot side is identical either
  way, so this is a configuration choice, not an architecture one.

  The same GPU makes the CLIP-class keyframe pre-filter effectively free, and
  makes NanoOWL (Phase 11) worth prototyping off-board before deciding whether
  it needs to run on the Jetson at all.

---

## What was transferred, and its state

    kaya/           the ROS packages + ROADMAP.md, full git history,
                    master at ab1e5cb — identical to the robot, verified
    maps/           home_final (41 MB: mesh + 2D grid), home, place
    cad/            CAD_CONTEXT.md, kaya_dimensions.yaml, regenerated xacro
    SETUP_LOG.md    the JetPack/ROS provisioning log
    kaya_check.sh   health check and provisioning scripts
    kaya_provision.sh
    curriculum.html
    backup_before_flash.sh

`git fsck` is clean and the working tree has no modifications. `origin` points
at the robot over ssh, and the remote-tracking ref matches — nothing is
unpushed.

The robot's full map store was pulled across on 2026-09-08 and now lives
outside the repo at `~/work_projects/kaya_robot/robot_maps/` — 719 MB, all
seven map directories, checksum-verified against the robot. Before that, the
`cuvslam/data.mdb` relocalisation maps and the `.nvblx` volumes existed **only**
on the robot's disk; just the meshes and 2D grids had ever been copied off, so
losing that disk would have meant re-mapping the space. See
`robot_maps/README.md` for what each directory holds and how to restore one.
Re-run the pull after any mapping session worth keeping.

---

## Picking the work back up

`ROADMAP.md` → *"Next session: test after charging"* is still the plan, in
order, stopping at the first failure:

1. Health check after the unclean shutdown — `sudo dmesg | grep -i nvgpu`,
   expect zero ACR errors. After a hard power loss the Jetson can boot with a
   dead GPU while the camera looks perfectly fine.
2. Re-verify the saved map still loads and relocalises from a deliberately
   wrong hint.
3. Navigate to two or three points, including one in another room — crossing
   the map is untested.
4. Drive a longer loop and re-check raw VIO drift (`odom` → `base_footprint`,
   not the corrected pose). Confirm 0.63% was not a lucky run.
5. Then start **Phase 8** — Foxglove dashboard. Needs no driving and little
   battery, and is the natural first thing to do from this machine.

Two practical constraints that have already cost real time:

- **Charge fully first.** Runtime tracks charge in — 2h30m, 1h50m, and under an
  hour after a quick top-up. Budget under two hours with the full stack, and
  note that servo voltage gives no warning before the battery dies.
- **Commit as you verify.** A battery death has already stranded verified,
  uncommitted work on the robot once.

And the settings table at the bottom of `ROADMAP.md` is not decoration — each
row cost real debugging. `time_domain: global` in particular: `system` corrupts
IMU timestamps and takes VIO drift from 0.63% to 15-32%, while every health
counter keeps reporting perfect health.
