"""Approximate bus-servo dynamics; all torques are output-shaft N m.

The published torque/speed endpoints are manufacturer specifications. PID gains,
latency, friction, rotor inertia and thermal time constants require identification.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class ServoSpec:
    stall_torque: float
    continuous_torque: float
    no_load_speed: float
    kp: float = 55.0
    kd: float = 1.2
    ki: float = 8.0
    command_speed: float = 2.0
    command_acceleration: float = 5.0
    latency: float = 0.02
    encoder_steps: int = 4096
    thermal_time_constant: float = 20.0

    def __post_init__(self):
        for name in ("stall_torque", "continuous_torque", "no_load_speed", "kp",
                     "command_speed", "command_acceleration", "thermal_time_constant"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if (not math.isfinite(self.latency) or not 0 <= self.latency < 1 or
                any(not math.isfinite(v) or v < 0 for v in (self.kd,self.ki))):
            raise ValueError("Invalid latency or servo gains")
        if self.continuous_torque > self.stall_torque or not isinstance(self.encoder_steps,int) or self.encoder_steps < 2:
            raise ValueError("Invalid servo torque ratings or encoder resolution")


# https://docs.waveshare.com/ST3215_Servo — kgf cm converted using 0.0980665.
PROFILES = {
    "7.4v": ServoSpec(19.5 * .0980665, 5 * .0980665, 52 * 2 * math.pi / 60),
    "12v": ServoSpec(30 * .0980665, 10 * .0980665, 45 * 2 * math.pi / 60),
}


class ServoBank:
    """Delayed commands, encoder feedback, slew limits and bounded effort.

    The caller holds bus commands between updates while running this feedback
    loop at the physics rate. Latency is represented in feedback-loop ticks.

    This produces motor torques only. It never changes simulation positions or
    velocities. The continuous-load derating is an estimated I²t approximation,
    not a calibrated temperature model or a prediction of hardware protection.
    """
    def __init__(self, spec, positions, period=.01):
        if not math.isfinite(period) or period <= 0:
            raise ValueError("Servo period must be finite and positive")
        self.spec, self.period = spec, period
        self.target = np.array(positions, dtype=float, copy=True)
        self.velocity = np.zeros_like(self.target)
        self.integral = np.zeros_like(self.target)
        self.load = np.zeros_like(self.target)
        self.torque = np.zeros_like(self.target)
        self.saturated = np.zeros_like(self.target, dtype=bool)
        self.queue = deque([self.target.copy() for _ in range(round(spec.latency / period))])

    def update(self, request, position, velocity, enabled=True):
        s, dt = self.spec, self.period
        request, position, velocity = [np.asarray(x, dtype=float) for x in (request, position, velocity)]
        if any(x.shape != self.target.shape or not np.isfinite(x).all()
               for x in (request, position, velocity)):
            raise ValueError("Servo vectors must be finite and match the motor count")
        if not enabled:
            self.queue = deque([position.copy() for _ in self.queue])
            self.target = position.copy()
            self.velocity.fill(0)
            self.integral.fill(0)
            self.torque.fill(0)
            self.saturated.fill(False)
            self.load = np.maximum(0., self.load - dt / s.thermal_time_constant)
            return self.torque.copy()
        self.queue.append(request.copy())
        delayed = self.queue.popleft()
        old_target = self.target.copy()
        distance = delayed - self.target
        stopping_speed = np.sqrt(2 * s.command_acceleration * np.abs(distance))
        desired_v = np.sign(distance) * np.minimum(s.command_speed, stopping_speed)
        self.velocity += np.clip(desired_v - self.velocity,
                                 -s.command_acceleration * dt, s.command_acceleration * dt)
        step = self.velocity * dt
        self.target += step
        crossed = (delayed - old_target) * (delayed - self.target) <= 0
        self.target[crossed] = delayed[crossed]
        self.velocity = (self.target - old_target) / dt
        quantum = 2 * math.pi / s.encoder_steps
        measured = np.round(position / quantum) * quantum
        error = self.target - measured
        candidate_integral = np.clip(self.integral + s.ki * error * dt,
                                     -s.continuous_torque, s.continuous_torque)
        requested = s.kp * error + s.kd * (self.velocity - velocity) + candidate_integral
        # Motoring loses available torque near no-load speed; braking remains bounded.
        speed_factor = np.where(requested * velocity > 0,
                               np.clip(1 - np.abs(velocity) / s.no_load_speed, 0, 1), 1.)
        peak = s.continuous_torque + (s.stall_torque - s.continuous_torque) * np.clip(1 - self.load, 0, 1)
        limit = peak * speed_factor
        self.torque = np.clip(requested, -limit, limit)
        self.saturated = np.abs(requested) > limit + 1e-12
        unwind = self.saturated & (error * requested < 0)
        self.integral = np.where(~self.saturated | unwind, candidate_integral, self.integral)
        self.load = np.maximum(0., self.load + dt / s.thermal_time_constant *
                               ((self.torque / s.continuous_torque) ** 2 - 1))
        return self.torque.copy()
