from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from drone_core.domain.models import BoundingBox


class Kalman1D:
    """Constant-velocity Kalman filter for one scalar: state = [value, rate]."""

    def __init__(self, q: float, r: float):
        self.q, self.r = q, r          # process accel noise (1/s^2)^2-ish, measurement variance
        self.x = 0.0
        self.v = 0.0
        self.p = [[1.0, 0.0], [0.0, 1.0]]
        self.initialised = False

    def reset(self) -> None:
        self.initialised = False

    def predict(self, dt: float) -> None:
        if not self.initialised:
            return
        self.x += self.v * dt
        p, q = self.p, self.q
        p00 = p[0][0] + dt * (p[1][0] + p[0][1]) + dt * dt * p[1][1] + q * dt ** 4 / 4
        p01 = p[0][1] + dt * p[1][1] + q * dt ** 3 / 2
        p10 = p[1][0] + dt * p[1][1] + q * dt ** 3 / 2
        p11 = p[1][1] + q * dt * dt
        self.p = [[p00, p01], [p10, p11]]

    def correct(self, z: float) -> None:
        if not self.initialised:
            self.x, self.v = z, 0.0
            self.p = [[self.r, 0.0], [0.0, 1.0]]
            self.initialised = True
            return
        p = self.p
        s = p[0][0] + self.r
        k0, k1 = p[0][0] / s, p[1][0] / s
        y = z - self.x
        self.x += k0 * y
        self.v += k1 * y
        self.p = [[(1 - k0) * p[0][0], (1 - k0) * p[0][1]],
                  [p[1][0] - k1 * p[0][0], p[1][1] - k1 * p[0][1]]]


@dataclass
class TargetEstimate:
    cx: float            # normalised 0..1
    cy: float            # normalised 0..1
    h: float             # box height / image height
    age: float           # seconds since the last real measurement (0 = fresh)
    rate_cx: float = 0.0
    rate_log_h: float = 0.0


@dataclass
class TargetFilterConfig:
    q_pos: float = 4.0          # how agile the target can be in image space
    r_pos: float = 0.02 ** 2    # ~2% of the frame of detector jitter
    q_size: float = 2.0
    r_size: float = 0.06 ** 2   # box height is the noisiest thing YOLO outputs
    coast_s: float = 0.6        # keep predicting this long without a detection
    latency_s: float = 0.0      # known camera->command delay to predict across


class TargetFilter:
    """Smooths the tracked box and bridges short detection gaps.

    Works in normalised image space on (cx, cy, log h). log h is used because
    it is linear in log-distance, which is what the distance PID regulates.
    """

    def __init__(self, cfg: TargetFilterConfig | None = None):
        self.cfg = cfg or TargetFilterConfig()
        c = self.cfg
        self._cx = Kalman1D(c.q_pos, c.r_pos)
        self._cy = Kalman1D(c.q_pos, c.r_pos)
        self._lh = Kalman1D(c.q_size, c.r_size)
        self._last_t: Optional[float] = None
        self._last_meas_t: Optional[float] = None

    def reset(self) -> None:
        for k in (self._cx, self._cy, self._lh):
            k.reset()
        self._last_t = None
        self._last_meas_t = None

    def update(self, box: Optional[BoundingBox], frame_w: int, frame_h: int, t: float) -> Optional[TargetEstimate]:
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        for k in (self._cx, self._cy, self._lh):
            k.predict(dt)

        if box is not None:
            self._cx.correct(box.cx / frame_w)
            self._cy.correct(box.cy / frame_h)
            self._lh.correct(math.log(max(box.h / frame_h, 1e-3)))
            self._last_meas_t = t
        if self._last_meas_t is None:
            return None
        age = t - self._last_meas_t
        if age > self.cfg.coast_s:
            self.reset()
            return None

        lead = self.cfg.latency_s
        return TargetEstimate(
            cx=self._cx.x + self._cx.v * lead,
            cy=self._cy.x + self._cy.v * lead,
            h=math.exp(self._lh.x + self._lh.v * lead),
            age=age,
            rate_cx=self._cx.v,
            rate_log_h=self._lh.v,
        )
