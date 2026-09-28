"""The real Kaya's command chain, vectorised over N environments in torch.

This is `kaya_description/usd/kaya_servos.py` (one robot, numpy) ported to run
on the GPU for every env at once. Same model, same measured numbers:

    body twist (vx, vy, w)
      -> wheel speeds    v_i = -sin(th_i) vx + cos(th_i) vy + L w     (drive node)
      -> latched at 30 Hz                                             (drive_node loop)
      -> 27.5 ms delay                                                (bus + firmware)
      -> 2nd-order servo: wn 31 rad/s, zeta 0.60                      (ST3020 speed loop)
      -> position target advancing at that speed + velocity target    (holds on slopes)

Call `step()` once per PHYSICS step (240 Hz), never per policy step: the
servo integrates on the physics clock, like firmware on its control tick.

Two PhysX facts it handles (see kaya_servos.py for the measurements):
  * wheel angles are reported wrapped into (-2pi, 2pi], so wraps are counted
    here and PhysX is only ever handed a target close to its own angle;
  * a velocity-only drive creeps on slopes, hence the position target.

Every env has its own delay, wn, zeta and 30 Hz phase, so domain
randomisation is just `randomize()`.
"""

import math

import torch

CONTROL_RATE_HZ = 30.0
CHAIN_DELAY_S = 0.0275
SERVO_WN = 31.0
SERVO_ZETA = 0.60
WHEEL_ANGLES_DEG = (300.0, 60.0, 180.0)     # servo ids 1, 2, 3
WHEEL_R = 0.037
L_SIM = 0.143                               # the sim's geometry; the real robot calibrated 0.138
WRAP = 4 * math.pi


def twist_to_wheels_matrix(device, L=L_SIM) -> torch.Tensor:
    """(3, 3) matrix M with wheel_rad_s = twist @ M.T, twist = (vx, vy, w)."""
    rows = [[-math.sin(math.radians(t)), math.cos(math.radians(t)), L] for t in WHEEL_ANGLES_DEG]
    return torch.tensor(rows, device=device, dtype=torch.float32) / WHEEL_R


class KayaServos:
    """Batched servo chain. State is per env; all tensors are (N, 3) unless noted."""

    HIST = 32   # delay-line length in physics steps (32 / 240 Hz = 133 ms, far above any delay)

    def __init__(self, num_envs: int, physics_dt: float, device, ideal: bool = False):
        self.n, self.dt, self.device, self.ideal = num_envs, physics_dt, device, ideal
        f64 = dict(device=device, dtype=torch.float64)
        f32 = dict(device=device, dtype=torch.float32)
        self.M = twist_to_wheels_matrix(device)
        self.speed = torch.zeros(num_envs, 3, **f32)        # commanded wheel speed
        self.latched = torch.zeros(num_envs, 3, **f32)      # what the 30 Hz loop last sampled
        self.hist = torch.zeros(self.HIST, num_envs, 3, **f32)
        self.ptr = 0
        self.ref = torch.zeros(num_envs, 3, **f32)          # servo's internal speed reference
        self.ref_rate = torch.zeros(num_envs, 3, **f32)
        # float64 for the accumulating angles: float32 loses mm after long episodes
        self.q_prev = torch.zeros(num_envs, 3, **f64)
        self.q_unwrapped = torch.zeros(num_envs, 3, **f64)
        self.target = torch.zeros(num_envs, 3, **f64)
        # per-env chain parameters (nominal until randomize())
        self.steps_per_tick = max(1, round(1.0 / (CONTROL_RATE_HZ * physics_dt)))   # 8 at 240 Hz
        self.tick_phase = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.delay_steps = torch.full((num_envs,), self._delay_steps(CHAIN_DELAY_S), dtype=torch.long, device=device)
        self.wn = torch.full((num_envs, 1), SERVO_WN, **f32)
        self.zeta = torch.full((num_envs, 1), SERVO_ZETA, **f32)
        self.step_count = 0
        self._arange = torch.arange(num_envs, device=device)

    def _delay_steps(self, seconds):
        # the numpy version applies a latched value once t - delay >= latch time
        return min(self.HIST - 1, math.ceil(seconds / self.dt - 1e-9))

    def randomize(self, env_ids, delay_frac=0.3, wn_frac=0.2, zeta_frac=0.2):
        """Domain randomisation of the measured chain (SIM_RL_PLAN D4)."""
        k = len(env_ids)
        u = lambda frac: 1.0 + frac * (2 * torch.rand(k, device=self.device) - 1)   # noqa: E731
        delays = CHAIN_DELAY_S * u(delay_frac)
        self.delay_steps[env_ids] = torch.clamp(torch.ceil(delays / self.dt - 1e-6).long(), 0, self.HIST - 1)
        self.wn[env_ids, 0] = SERVO_WN * u(wn_frac)
        self.zeta[env_ids, 0] = SERVO_ZETA * u(zeta_frac)
        self.tick_phase[env_ids] = torch.randint(0, self.steps_per_tick, (k,), device=self.device)

    def reset(self, env_ids, q):
        """Hold wherever the wheels are now. q: (len(env_ids), 3) wheel angles from PhysX."""
        q = q.to(torch.float64)
        self.q_prev[env_ids] = q
        self.q_unwrapped[env_ids] = q
        self.target[env_ids] = q
        for t in (self.speed, self.latched, self.ref, self.ref_rate):
            t[env_ids] = 0.0
        self.hist[:, env_ids] = 0.0

    def command_twist(self, twist: torch.Tensor):
        """twist: (N, 3) body-frame (vx [m/s], vy [m/s], w [rad/s]), what /cmd_vel means."""
        self.speed[:] = twist @ self.M.T

    def step(self, q: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Advance one physics step. q: (N, 3) wheel angles as PhysX reports them.
        Returns (position_target, velocity_target), both (N, 3) float32."""
        q64 = q.to(torch.float64)
        d = torch.remainder(q64 - self.q_prev + WRAP / 2, WRAP) - WRAP / 2
        self.q_unwrapped += d
        self.q_prev = q64
        if self.ideal:
            self.ref = self.speed.clone()
        else:
            # drive_node's 30 Hz loop: each env samples on its own phase. The +1
            # matches kaya_servos.py, whose clock advances before its tick check.
            tick = ((self.step_count + 1 + self.tick_phase) % self.steps_per_tick) == 0
            self.latched = torch.where(tick[:, None], self.speed, self.latched)
            # delay line
            self.hist[self.ptr] = self.latched
            u = self.hist[(self.ptr - self.delay_steps) % self.HIST, self._arange]
            self.ptr = (self.ptr + 1) % self.HIST
            # servo speed loop, 2nd order (semi-implicit Euler, as in kaya_servos.py)
            acc = self.wn**2 * (u - self.ref) - 2 * self.zeta * self.wn * self.ref_rate
            self.ref_rate += acc * self.dt
            self.ref += self.ref_rate * self.dt
        self.step_count += 1
        self.target += self.ref.to(torch.float64) * self.dt
        sent = q64 + (self.target - self.q_unwrapped)
        return sent.to(torch.float32), self.ref.clone()
