from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PID:
    """PID with derivative-on-measurement, output clamp and anti-windup."""
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    out_limit: float = float("inf")
    i_limit: float = float("inf")
    deadband: float = 0.0
    d_filter: float = 0.7          # 0 = no filtering, ->1 = heavy low-pass on D term
    i_zone: float = float("inf")   # only integrate when |error| is below this (stops windup
                                   # during long catch-ups, e.g. after re-finding the target)

    _integral: float = field(default=0.0, init=False)
    _prev_error: Optional[float] = field(default=None, init=False)
    _d_state: float = field(default=0.0, init=False)

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None
        self._d_state = 0.0

    def update(self, error: float, dt: float) -> float:
        if abs(error) < self.deadband:
            error = 0.0
        if dt <= 0:
            dt = 1e-3

        d_raw = 0.0 if self._prev_error is None else (error - self._prev_error) / dt
        self._d_state = self.d_filter * self._d_state + (1 - self.d_filter) * d_raw
        self._prev_error = error

        p = self.kp * error
        d = self.kd * self._d_state
        candidate_i = self._integral + (error * dt if abs(error) < self.i_zone else 0.0)
        candidate_i = max(-self.i_limit, min(self.i_limit, candidate_i))
        out = p + self.ki * candidate_i + d

        # anti-windup: only integrate if we're not saturated in the same direction
        if abs(out) < self.out_limit or (out > 0) != (error > 0):
            self._integral = candidate_i
        out = p + self.ki * self._integral + d
        return max(-self.out_limit, min(self.out_limit, out))
