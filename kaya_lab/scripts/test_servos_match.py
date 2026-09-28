"""The batched torch servo chain must match the single-robot numpy original
(kaya_description/usd/kaya_servos.py) step for step. No simulator needed:
the wheels are assumed to follow their position target exactly."""
import sys, types, math
from pathlib import Path
import numpy as np, torch
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "kaya_lab"), str(ROOT / "kaya_description" / "usd")]

# stub the Isaac Sim imports the numpy class makes, and a fake robot
sm = types.ModuleType("isaacsim.core.simulation_manager")
sm.IsaacEvents = types.SimpleNamespace(PRE_PHYSICS_STEP=0)
sm.SimulationManager = types.SimpleNamespace(register_callback=lambda *a: 0, deregister_callback=lambda *a: 0)
for name in ["isaacsim", "isaacsim.core"]:
    sys.modules[name] = types.ModuleType(name)
sys.modules["isaacsim.core.simulation_manager"] = sm

class FakeRobot:                       # the wheel sits exactly on its last target, wrapped like PhysX
    def __init__(s): s.q = np.zeros((1, 3))
    def get_dof_indices(s, names): return torch.tensor([0, 1, 2])
    def get_dof_positions(s): return torch.tensor(s.q)
    def set_dof_position_targets(s, t, dof_indices=None): s.q = ((t + 2*np.pi) % (4*np.pi)) - 2*np.pi
    def set_dof_velocity_targets(s, t, dof_indices=None): pass

from kaya_servos import WheelServos, wheel_speeds
from kaya_lab.servos import KayaServos

dt = 1 / 240
ref_np, ref_t = [], []
r = FakeRobot(); a = WheelServos(r)
b = KayaServos(1, dt, "cpu"); b.reset(torch.tensor([0]), torch.zeros(1, 3))
q_t = torch.zeros(1, 3)
cmd = lambda k: (0.15, 0.05, 0.8) if 10 <= k < 400 else (0.0, 0.0, 0.0)
for k in range(700):
    a.drive(*cmd(k)); a._on_step(dt); ref_np.append(a.ref.copy())
    b.command_twist(torch.tensor([cmd(k)])); pos, vel = b.step(q_t)
    q_t = ((pos + 2*math.pi) % (4*math.pi)) - 2*math.pi; ref_t.append(vel[0].numpy())
ref_np, ref_t = np.array(ref_np), np.array(ref_t)
# The numpy original accumulates t += dt in floating point, so its 30 Hz tick
# lands one physics step early or late now and then (it ticked on steps 0, 7,
# 15, 24, 31...). The torch port ticks on an exact grid. A command change can
# therefore be picked up one TICK (8 steps) apart. Compare each response with
# the best whole-tick alignment; within it the chain (delay + servo) must be
# identical.
def seg_err(lo, hi):
    return min(np.abs(ref_np[lo:hi] - np.roll(ref_t, -s_, 0)[lo:hi]).max() for s_ in (-8, 0, 8))
e_start, e_stop = seg_err(0, 390), seg_err(395, 690)
print(f"speed-up response max err {e_start:.2e} rad/s, slow-down response {e_stop:.2e} rad/s "
      f"(peak {np.abs(ref_np).max():.2f} rad/s)")
print("wheel speeds match kinematics:", np.allclose(b.M.numpy() @ np.array([0.15, 0.05, 0.8]), wheel_speeds(0.15, 0.05, 0.8), atol=1e-5))
assert max(e_start, e_stop) < 1e-4, "torch port diverges from the numpy original"
print("PASS")
