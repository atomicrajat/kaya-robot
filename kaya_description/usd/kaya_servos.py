"""Kaya's wheel servos and drive kinematics, for scripts running in Isaac Sim.

The Waveshare ST3020 closes its loop on a 4096-count encoder, so in wheel mode
it holds its position when told speed 0 and tracks speed through position.
The asset models that as a stiffness + damping drive (build_kaya_usd.py), and
this class feeds it the way the servo's own firmware would: a speed command
becomes a position target advancing at that speed, plus the matching velocity
target. Sending only velocity targets lets the wheels creep under load - see
the SERVO_STIFFNESS note in build_kaya_usd.py for the measurement.
"""

import math

import numpy as np

WHEEL_JOINTS = ["wheel_1_wheel_joint", "wheel_2_wheel_joint", "wheel_3_wheel_joint"]

# The real command chain, fitted 2026-09-27 to the robot's own step test
# (data/real_step_2026-09-27.json: 0.1 m/s forward 2 s, stop, back, stop):
#   drive_node latches /cmd_vel at 30 Hz (control_rate_hz) -> 27.5 ms of
#   bus + firmware delay -> the servo's own speed loop, 2nd order:
#   wn 31 rad/s, zeta 0.60 (16% overshoot, 10-90% rise ~70 ms).
# Fit RMSE 0.065 rad/s - the servo's own speed jitter is 0.055. An ideal,
# instant servo is off by 0.524 rad/s on the same data.
CONTROL_RATE_HZ = 30.0
CHAIN_DELAY_S = 0.0275
SERVO_WN = 31.0
SERVO_ZETA = 0.60
WHEEL_ANGLES_DEG = [300.0, 60.0, 180.0]       # servo ids 1, 2, 3
WHEEL_R = 0.037
L_SIM = 0.143       # the sim's geometry; the real robot calibrated 0.138


def wheel_speeds(vx, vy, w, L=L_SIM):
    """Body twist -> wheel rad/s, the same model the drive node runs
    (kaya_drive/kinematics.py): v_i = -sin(th_i) vx + cos(th_i) vy + L w."""
    return np.array([(-math.sin(math.radians(t)) * vx + math.cos(math.radians(t)) * vy
                      + L * w) / WHEEL_R for t in WHEEL_ANGLES_DEG])


class WheelServos:
    """Runs on every PHYSICS step, like servo firmware on its control tick.
    Integrating on the app/timeline clock instead is wrong: with a heavy scene
    one rendered frame can cover more time than its physics substeps do, the
    target runs ahead of the wheel, and the servo chases it at full speed
    (seen 2026-09-27: 9 rad of error after 3.5 s).

    PhysX also reports these joint angles WRAPPED into (-2pi, 2pi] (it jumps
    by 4pi; measured 2026-09-27, 6.09 -> -5.46 rad while spinning forward).
    A target that simply accumulates therefore sits whole turns away from the
    reported angle after a few revolutions and the drive spins the wheel flat
    out to "catch up". So this counts rollovers itself - like counting an
    encoder's wraps - and hands PhysX the target RELATIVE to the angle PhysX
    currently reports: the drive only ever sees the true, small error."""

    def __init__(self, robot, ideal=False):
        """ideal=True: the wheels follow commands instantly (no 30 Hz loop, no
        delay, no servo dynamics) - handy for debugging, wrong for training."""
        from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager

        self.robot = robot
        self.idx = robot.get_dof_indices(WHEEL_JOINTS).numpy()
        self.speed = np.zeros(3)          # what was commanded
        self.ideal = ideal
        self.t = 0.0                      # time since construction (physics)
        self.next_tick = 0.0
        self.latched = []                 # (time, speed) entering the delay line
        self.ref = np.zeros(3)            # the servo's internal speed reference
        self.ref_rate = np.zeros(3)
        self.broken = False
        self.reset()
        self._sm = SimulationManager
        self._uid = SimulationManager.register_callback(self._on_step, IsaacEvents.PRE_PHYSICS_STEP)

    WRAP = 4 * math.pi

    def reset(self):
        """Hold wherever the wheels are now (after a teleport, a reset...)."""
        q = self.robot.get_dof_positions().numpy()[0, self.idx].astype(np.float64)
        self.q_prev = q
        self.q_unwrapped = q.copy()
        self.target = q.copy()

    def command(self, wheel_rad_s):
        self.speed = np.asarray(wheel_rad_s, dtype=np.float64)

    def drive(self, vx, vy, w):
        self.command(wheel_speeds(vx, vy, w))

    def close(self):
        try:
            self._sm.deregister_callback(self._uid)
        except Exception:
            pass

    def _on_step(self, dt, *_):
        if self.broken:
            return
        try:
            self._step(dt)
        except Exception:
            # physics was rebuilt under us (the USD was reloaded, or Stop):
            # this handle is dead. The owner sees .broken and re-attaches.
            self.broken = True

    def _step(self, dt):
        q = self.robot.get_dof_positions().numpy()[0, self.idx].astype(np.float64)
        d = (q - self.q_prev + self.WRAP / 2) % self.WRAP - self.WRAP / 2
        self.q_unwrapped += d
        self.q_prev = q
        self.t += dt
        if self.ideal:
            self.ref = self.speed.copy()
        else:
            while self.t >= self.next_tick:          # drive_node's 30 Hz loop
                self.latched.append((self.next_tick, self.speed.copy()))
                self.next_tick += 1.0 / CONTROL_RATE_HZ
            u = np.zeros(3)
            while self.latched and self.latched[0][0] <= self.t - CHAIN_DELAY_S:
                u = self.latched.pop(0)[1]
                self.u_last = u
            u = getattr(self, "u_last", u)
            acc = SERVO_WN ** 2 * (u - self.ref) - 2 * SERVO_ZETA * SERVO_WN * self.ref_rate
            self.ref_rate += acc * dt
            self.ref += self.ref_rate * dt
        self.target += self.ref * dt
        # all bookkeeping in float64; PhysX only sees a target near its own q
        sent = q + (self.target - self.q_unwrapped)
        self.robot.set_dof_position_targets(sent[None].astype(np.float32),
                                            dof_indices=self.idx)
        self.robot.set_dof_velocity_targets(self.ref[None].astype(np.float32),
                                            dof_indices=self.idx)
