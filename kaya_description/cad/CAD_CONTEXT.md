# Kaya CAD — context for the Fusion 360 session

> Lives in the repo at `kaya_description/cad/` since 2026-09-27, beside
> `kaya_dimensions.yaml`. The Isaac Sim model built from all of this is in
> `kaya_description/usd/` (see its README).

Hand this file to whoever drives the Fusion 360 model, together
with `kaya_dimensions.yaml` and the robot photos. It carries everything decided
so far so that session does not have to re-derive it.

---

## 1. What we are building and why

A parametric Fusion 360 model of the **Kaya** robot: a three-wheel holonomic
(omni) mobile robot built on a Jetson Orin Nano 8GB.

Two consumers, and they want different things:

| Consumer | Needs | Tolerance for approximation |
|---|---|---|
| **ROS 2 / Nav2** (now) | Accurate kinematics, simple collision shapes, mass + inertia | Visual detail irrelevant |
| **Isaac Sim / OpenUSD** (later) | The above, plus visual meshes and real material properties | Cosmetic shape can be simplified |

Neither needs a manufacturing-accurate model. **Do not spend effort on fillets,
fastener heads, or cable routing.** Spend it on the numbers in §3.

---

## 2. THE NON-NEGOTIABLE CONSTRAINT: frames must match ROS

This is the single thing most likely to go wrong, and it is invisible until the
robot drives at an angle to every command.

### Origin and axes (ROS REP-103)

- **`base_link` origin**: on the chassis' axis of rotation, at **wheel axle
  height**. Not at the bottom plate, not at the geometric centre of a bounding
  box — the axis the robot spins about.
- **Axes**: `+X` forward, `+Y` **left**, `+Z` up. Right-handed.
- **`base_footprint`**: the same point projected to the floor, `z = 0`. So
  `base_link` sits exactly one wheel rolling radius above it.
- **Forward (`+X`) is defined by the camera** — where it looks, projected onto
  the floor plane.

> Fusion 360 defaults to Y-up in some templates. **Build in Z-up.** If the model
> is authored Y-up, every downstream transform is wrong by a 90° rotation and it
> will not be obvious until the robot is upside-down in RViz.

### Link names — must match exactly

The ROS URDF already exists and uses these names. Fusion components should carry
the same names so the export maps cleanly:

```
base_footprint
└── base_link
    ├── wheel_1_wheel_link      (servo ID 1)
    ├── wheel_2_wheel_link      (servo ID 2)
    ├── wheel_3_wheel_link      (servo ID 3)
    ├── camera_link             ← the LEFT IR LENS. STOP HERE. See below.
    └── mpu6050_link
```

**Do not model anything below `camera_link`.** The Orbbec driver publishes the
whole internal camera chain (`camera_link → camera_depth_frame → … →
camera_accel_gyro_optical_frame`) on `/tf_static` at runtime. Duplicating any of
it gives TF two parents for one frame, which produces `TF_MULTIPLE_AUTHORITY`
and a transform that flickers between two values. Our URDF's job is exactly one
transform: `base_link → camera_link`.

**`camera_link` is the left IR lens, not the camera body centre.** The driver's
`/tf_static` has `camera_link → camera_depth_frame` as the identity, and depth
is the left IR camera. Until 2026-09-27 the URDF put `camera_link` at the body
centre, 47.5 mm to the right, and every depth point landed ~5 cm right of the
truth - measured with a bottle on the centre line (see the URDF comment).

### Joints

- Three wheel joints: **continuous**, axis tangential to the chassis circle.
  For a wheel at body angle θ, the joint origin is rotated θ about Z and the
  axis is the joint's local Y.
- Everything else: **fixed**.

---

## 3. What must be accurate, ranked

1. **Wheel rolling radius** — linearly scales all odometry. Measured under load,
   not with calipers on an unloaded wheel.
2. **Contact-patch radius** (rotational centre → wheel floor contact) — scales
   all rotational odometry.
3. **Wheel angles and spin directions** — build the kinematics matrix. A sign
   error here makes the robot strafe when told to rotate.
4. **Camera pitch** — an unrecorded tilt makes nvblox see a sloped floor and
   fill the costmap with obstacles that are not there. Measured from the IMU's
   gravity vector, not a protractor.
5. **Camera XYZ** — a fixed offset in every map built.
6. **Mass and inertia** — Fusion computes inertia from geometry + material
   density, which is the main reason real CAD beats the current placeholders.

Everything else is cosmetic.

---

## 4. Single source of truth

```
cad/kaya_dimensions.yaml   ← measurements live HERE, nowhere else
        │
        ├──→ Fusion 360 model
        ├──→ kaya_description/urdf/kaya.urdf.xacro
        └──→ Isaac Sim USD
```

Never type a dimension straight into Fusion. Put it in the YAML and drive the
model from it — Fusion **User Parameters** map onto this cleanly, so create one
parameter per YAML field and reference parameters in every sketch.

The reason is not tidiness. If CAD and URDF are authored separately they will
drift, and the resulting sim-to-real mismatch looks like a physics problem when
it is actually a typo.

---

## 5. Isaac Sim requirements — decide these while modelling, not after

**Collision ≠ visual.** Emit two geometries per link. Collision should be convex
primitives (cylinder, box); a detailed mesh simulates orders of magnitude slower
and less stably.

**Omni wheels are the hard part.** Their behaviour comes entirely from passive
rollers. Simulating twelve free-spinning rollers per wheel is expensive and
numerically twitchy. The standard approach:

- Model rollers **visually** so it looks right
- Simulate as a **single cylinder with anisotropic friction** — high along the
  roll direction, near-zero laterally

**Materials.** Set real densities per component in Fusion (aluminium vs acrylic
vs PLA) so the computed inertia tensors mean something. Then check the model's
total mass against the scale reading in `mass_g.total_robot` — if they disagree
by more than ~15%, a density or a wall thickness is wrong.

---

## 6. Known-good values, measured on the real robot

Already verified. Do not re-derive, and do not let a spec sheet override them.

| Fact | Value | Source |
|---|---|---|
| Servos | **3× WAVESHARE ST3020**, IDs 1/2/3, 1 Mbaud, `/dev/ttyACM0` | read off the labels |
| Wheels | 74 mm dia, **25 mm wide, staggered double roller row** | measured + photos |
| Contact radius `L` | **143 mm** (assembly mid-plane) | derived, cross-checked |
| Wheel layout | **two forward at ±60°, one rear at 180°** | photos |
| Wheel axle | **RADIAL** — shafts point straight out from the chassis | photos |
| Camera | Orbbec Gemini 335L, sn `CP3C641000JS`, FW 1.3.25 | driver |
| Camera body centre | **21.81 mm forward, 0 lateral, 173.72 mm above the floor** | CAD session, 2026-08-29 (supersedes the earlier direct 165 mm reading — see `kaya_dimensions.yaml` camera.front_face_z_mm) |
| `camera_link` (left IR lens) | **23.99 mm forward, 47.48 mm LEFT, 175.24 mm above the floor** | derived from the body centre; confirmed with a bottle test 2026-09-27 |
| Camera tilt | **4.89° NOSE UP**, **+0.94° roll (right side lower)** in REP-103 | IMU gravity vector; roll SIGN confirmed 2026-09-27 against the real IMU in Isaac Sim |
| Stereo | fx 408.16 px at 848×480, baseline **94.86 mm**, colour 23.75 mm right of left IR | driver camera_info + /tf_static, 2026-09-27 |
| Camera IMU | 6-axis, **no magnetometer**, 203 Hz, device timestamps | measured |
| IMU noise | gyro sd 0.0005 rad/s, accel sd 0.007–0.010 m/s², gyro bias (+0.0099, +0.0002, +0.0053) rad/s | measured (30 s still, 2026-09-27) |
| One wheel | **112 g** (hub + rollers, no coupler) | kitchen scale, 2026-09-27 |
| Roller friction | robot slides at **15°** on a table, camera downhill → μ 0.40 in the sim | tilt test, 2026-09-27 |
| Servo response | 30 Hz drive loop + 27.5 ms delay + 2nd order (ωn 31 rad/s, ζ 0.60) | step test, 2026-09-27 |
| Total mass | **2.088 kg** | scale |
| Module | Orin Nano 8GB, `p3767-0003` on carrier `p3768-0000` | SKU fuse `0xD5` |

> **Two earlier claims were wrong and are corrected above.** `SETUP_LOG.md` says
> the servos are Feetech ST3215 — they are **Waveshare ST3020**. And an earlier
> note had the camera pitched *down*; it is pitched **up**, confirmed by both the
> IMU (4.89°) and visual inspection (~5°), agreeing to 0.1°.

### Build, as seen in the photos

- **Chassis**: truncated equilateral triangle, built as a **two-plate stack**
  joined by brass standoffs, with a folded sheet-metal shell (ventilation slots,
  antenna cutouts) dropping over the top as a separate cover.
  - **Upper plate** carries the electronics: Jetson carrier, a servo driver PCB
    with pin headers, wiring.
  - **Lower plate** carries the battery and the servo brackets. A separate small
    retainer plate holds the battery down; model it, since it sets the usable
    height between the plates.
  - Standoffs appear in at least two different lengths.
- **Servos**: one at each of the three **truncated corners**, on brackets below
  the upper plate. Bodies sit roughly tangential; output shafts point **radially
  outward** through metal couplers to the wheels. This is the visual confirmation
  that the wheel axle is radial and the wheels roll tangentially - the URDF
  encodes it as `axis xyz="1 0 0"` in a joint frame rotated by theta.
- **Battery**: "SHARE+ MAX 2.0Ah" power-tool pack, mounted centrally between the
  plates. Slides in on rails, so it is removable - model the envelope plus the
  clearance the slide needs, not just the pack.
- **Camera**: on top of the shell at the front apex, on a small bracket, tilted up.
- Two whip antennas at the rear corners.

## 7. Current URDF state — SUPERSEDED, regenerated from CAD

Everything below this line was true of the *hand-measured* URDF (commits
`735fc6d`, `f433640`, `ef20a4d`) and is now stale. The CAD session of
2026-08-29 regenerated the file from Fusion — real vendor STEP envelopes for
the servos and camera, measured chassis/shell geometry, and computed inertia
tensors — and Phase 2 (2026-09-03) fixed the servo-to-wheel mapping and
calibrated the rotational term on top of that. **This is what's actually
live**, committed on the robot's `kaya_description` package at commits
`371f15b` (URDF regen + servo mapping fix + chassis_radius calibration) and
`b73f12f` (the `kaya_drive` package that drives it). A mirror of that exact
file is kept here as `kaya.urdf.xacro.cad-regenerated` for reference; the
robot's committed copy is the real source of truth, not this one.

```
wheel_radius     0.037   MEASURED (74 mm dia)
wheel_width      0.025   MEASURED (double row)
chassis_radius   0.138   CALIBRATED 2026-09-03 (10-turn spin test; CAD's
                          geometric contact_radius stays 143 mm — see
                          kaya_dimensions.yaml wheels.kinematic_L_mm)
footprint_radius 0.160   DERIVED  (encloses the 155.5 mm outer extent)
deck_radius      0.1195  CAD, outline still an ESTIMATE despite the real
                          measured hexagon outline existing (195/50 mm) —
                          worth reconciling next time Fusion is open
chassis_height   0.130   now CAD-derived, not a photo guess
camera body      (0.02181, 0.000, 0.13672), z relative to base_link;
                 0.17372 m above the FLOOR
camera_link      (0.02399, 0.04748, 0.13824) rpy (+0.0164, -0.0853, 0)
                 = the LEFT IR LENS (2026-09-27; was the body centre, and
                 the roll sign was flipped)
total_mass       2.088   MEASURED
wheel_mass       0.112   MEASURED 2026-09-27 (was 0.05845 CAD-derived)
base_mass        1.588   = total - 3 wheels - camera, keeping the CAD's
                          mass DISTRIBUTION (inertia scaled 1.588/1.74864)
```

Wheel joints, **resolved 2026-09-03** by pulsing each servo individually and
watching which wheel moved:

| Servo | Wheel | θ |
|---|---|---|
| 1 | front-right | 300° |
| 2 | front-left | 60° |
| 3 | rear | 180° |

Each with a **radial** axis (`axis xyz="1 0 0"` in a frame rotated by θ), so
the wheels roll tangentially. All three servos' positive Goal Speed matches
the standard kinematic sign convention directly — no per-wheel flip.

Inertia tensors are now real, computed from CAD geometry and scaled by a
single factor (1.0294 since the wheel was weighed; 1.1336 before) so the four
rigid bodies sum to the measured 2.088 kg
while keeping the CAD's mass distribution — see `kaya.urdf.xacro`'s own
comments for the derivation. **What this replaced**: solid-cylinder
placeholders that overstated `izz` (the yaw-response-dominating term) by
3.5×.

**Still not done**, per `kaya_dimensions.yaml`'s open questions:
`chassis.plate_thickness_mm` and `plate_material` (calipers + identify the
material), and per-component masses (`one_wheel`, `one_servo`, `battery`,
`jetson_plus_carrier` — a kitchen scale). Both blocking a fully-real (rather
than total-mass-scaled) inertia tensor.

## 8. Deliverables

Updated 2026-09-03 against the actual regenerated URDF and `kaya_dimensions.yaml`
r2 — not re-verified inside Fusion itself, since no Fusion MCP access exists
from this side. Confirm the unclear ones next time that session is open.

- [x] Fusion 360 model, Z-up, components named per §2, driven by User Parameters
- [ ] `kaya_dimensions.yaml` fully populated — much closer (vendor STEP envelopes,
      measured chassis/shell geometry), but `chassis.plate_thickness_mm`,
      `plate_material`, and most of `mass_g` are still `null`; `derived:` section
      still entirely `null`
- [ ] Visual meshes exported (OBJ or STL) per link — unconfirmed; the URDF's
      mesh line (`package://kaya_description/meshes/${name}.obj`) is present
      but commented out, cylinders are what's actually active
- [x] Collision primitives defined per link — cylinders/box, all present
- [x] Mass + inertia tensor per link, from real densities — done, scaled to
      the measured 2.088 kg total (see §7)
- [x] Regenerated `kaya.urdf.xacro` with no `TODO measure` remaining — done,
      committed on the robot (`kaya_description` commit `371f15b`)
- [ ] STEP export, so the model survives independent of Fusion — unconfirmed

## 9. Answered, and still open

**Answered from the photos** (were open questions in the first draft):

1. Chassis is a **flat structural plate plus a folded sheet-metal shell**, not a
   printed body. Straightforward parametric work.
2. All three contact radii are **equal**, so the symmetric kinematics hold.
3. The camera bracket appears **fixed**, not adjustable — so the 4.89° tilt
   should be stable unless the robot takes a knock.

**Answered 2026-09-03** — servo-to-wheel mapping and spin direction, by pulsing
each servo individually in wheel mode (low speed, ~35°, robot on the floor)
and watching which wheel turned and which way:

| Servo | Wheel position | θ (from +X, CCW+) | Spin sign |
|---|---|---|---|
| 1 | front-right | 300° (−60°) | matches standard convention, no flip |
| 2 | front-left | 60° | matches standard convention, no flip |
| 3 | rear | 180° | matches standard convention, no flip |

All three servos' positive Goal Speed matches `v_i = -sin(θᵢ)vx + cos(θᵢ)vy +
Lω` directly — no per-wheel sign correction needed in the drive node. Applied
to `kaya.urdf.xacro` (previously had a provisional, and wrong, θ assignment
per servo — see its own warning comment, now resolved).

**Still open — none of these block the CAD:**

| Question | Why it matters | Needs |
|---|---|---|
| Loaded rolling radius | 74 mm is a caliper reading; rollers compress under 2.088 kg | 10-rev roll test |
| MPU-6050 position | Only affects the legacy fallback sensor | a ruler |

## 10. Things to measure while modelling

The CAD session will want these and they are not in the YAML yet, because they
only matter once the shell is being drawn:

- Triangle edge length and corner truncation width
- Structural plate thickness, and shell sheet thickness
- Height of the plate's top surface above the floor
- Shell height and where it sits relative to the plate
- Servo bracket dimensions and how far the shaft coupler stands the wheel off
- Battery pack outer dimensions
- Jetson carrier position on the plate

Add each to `kaya_dimensions.yaml` as it is measured — never straight into
Fusion. The YAML is the source of truth; the model is a generated artefact.
