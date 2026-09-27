#!/usr/bin/env python3
"""Replay a real step test in Isaac Sim and compare the wheels, sim vs real.

    ~/isaac-sim/python.sh kaya_description/usd/step_compare.py [--ideal]

Replays the exact /cmd_vel sequence recorded on the robot (data/real_step_*.json,
made by driving it 0.1 m/s forward 2 s, stop, back, stop) through WheelServos,
samples the sim wheels at the real /joint_states times, and reports the error
plus rise / overshoot / stop and distance, side by side. Writes a plot.
"""
import argparse, json, math, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser()
ap.add_argument("--data", default=str(HERE / "data" / "real_step_2026-09-27.json"))
ap.add_argument("--ideal", action="store_true", help="instant servos, for comparison")
ap.add_argument("--plot", default=str(HERE / "test_output" / "step_compare.png"))
ap.add_argument("--device", choices=["cpu", "cuda"], default="cpu",
                    help="physics on CPU (one robot: fastest) or GPU (what training uses)")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
sys.path.insert(0, str(HERE))
import kaya_isaac as kaya  # noqa: E402
from kaya_servos import WheelServos, wheel_speeds  # noqa: E402

D = json.load(open(args.data))
cmd = np.array(D["cmd"]); t0 = cmd[0, 0]; ct, cv = cmd[:, 0] - t0, cmd[:, 1]
jt = np.array([r[0] for r in D["js"]]) - t0; jv = np.array([r[2] for r in D["js"]])
od = np.array(D["odom"]); real_fwd = od[:, 1].max() - od[0, 1]

st = kaya.new_stage()
kaya.add_floor(st, size=20.0)                 # the tile, as in kaya_sim.py
kaya.add_kaya(st, "/World/kaya")
kaya.setup_physics(device=args.device)
app_utils.update_app(steps=2); robot = Articulation("/World/kaya"); app_utils.play(); app_utils.update_app(steps=3)
for _ in range(60): app.update()                                    # settle
servos = WheelServos(robot, ideal=args.ideal)
T0 = SimulationManager.get_simulation_time()
st_t, st_v, st_x = [], [], []
x0 = robot.get_world_poses()[0].numpy()[0, 0]
while (t := SimulationManager.get_simulation_time() - T0) < jt[-1]:
    i = np.searchsorted(ct, t, side="right") - 1
    servos.drive(cv[i] if i >= 0 else 0.0, 0.0, 0.0)
    app.update()
    st_t.append(t); st_v.append(robot.get_dof_velocities().numpy()[0, servos.idx].copy())
    st_x.append(robot.get_world_poses()[0].numpy()[0, 0] - x0)
st_t, st_v, st_x = map(np.array, (st_t, st_v, st_x))

def metrics(t, w, tgt, on, off):
    def first(cond, after):
        m = (t > after) & cond
        return t[m][0] - after if m.any() else float("nan")
    ss = w[(t > on + 1.0) & (t < off)]
    return (first(w >= 0.1 * tgt, on), first(w >= 0.9 * tgt, on),
            # overshoot of the STEP: the first 0.4 s only, not a roller
            # hand-over spike somewhere in the cruise
            w[(t > on) & (t < on + 0.4)].max() / tgt - 1, ss.mean() / tgt, first(w <= 0.1 * tgt, off))

tgt = wheel_speeds(0.1, 0, 0)[0]
on = ct[np.argmax(cv > 0)]; off = ct[np.argmax((cv == 0) & (ct > on))]
# Like for like: the servo reports speed from encoder counts over its sample
# window, not an instantaneous velocity, so average the sim over the 33 ms
# (one drive_node tick) before each real sample. Unaveraged, the sim shows
# roller hand-over spikes the real readback never could.
def servo_readback(t_query, win=1.0 / 30.0):
    return np.array([[st_v[(st_t > tq - win) & (st_t <= tq), k].mean() if ((st_t > tq - win) & (st_t <= tq)).any()
                      else 0.0 for k in range(3)] for tq in t_query])
sim_at_real = servo_readback(jt)
rmse = np.sqrt(np.mean((sim_at_real[:, :2] - jv[:, :2]) ** 2))
r = metrics(jt, jv[:, 0], tgt, on, off); s = metrics(jt, sim_at_real[:, 0], tgt, on, off)
print(f"\n=========== STEP TEST: sim ({'IDEAL' if args.ideal else 'servo-chain'} servos) vs real robot ===========")
print(f"wheel speed RMSE vs real (wheels 1+2):  {rmse:.3f} rad/s   (real servo jitter ~0.055)")
for name, i, fmt in [("10% of speed after command", 0, "{:.3f} s"), ("90% of speed after command", 1, "{:.3f} s"),
                     ("overshoot", 2, "{:+.0%}"), ("steady speed / target", 3, "{:.1%}"), ("below 10% after stop", 4, "{:.3f} s")]:
    print(f"  {name:28s} real {fmt.format(r[i]):>8s}   sim {fmt.format(s[i]):>8s}")
print(f"  {'distance forward':28s} real {real_fwd * 1000:6.1f} mm   sim {st_x.max() * 1000:6.1f} mm")

import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(10, 4.5))
ax.plot(jt, jv[:, 0], "o", ms=3, color="#1f77b4", label="real wheel_1 (/joint_states)")
ax.plot(st_t, st_v[:, 0], "-", lw=0.6, color="#d62728", alpha=0.35, label="sim wheel_1, instantaneous")
ax.plot(jt, sim_at_real[:, 0], "-", lw=1.8, color="#d62728", label="sim wheel_1, as the servo reads it")
ax.step(ct, cv * wheel_speeds(1, 0, 0)[0], where="post", color="gray", lw=1, ls="--", label="command")
ax.set_xlabel("time (s)"); ax.set_ylabel("wheel speed (rad/s)"); ax.grid(alpha=0.3); ax.legend(loc="upper right")
ax.set_title(f"Kaya step test - sim ({'ideal' if args.ideal else 'servo chain'}) vs real, RMSE {rmse:.3f} rad/s")
Path(args.plot).parent.mkdir(parents=True, exist_ok=True); fig.tight_layout(); fig.savefig(args.plot, dpi=110)
print(f"plot: {args.plot}")
servos.close(); app_utils.stop(); app.close()
