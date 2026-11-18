from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from drone_core.domain.models import FollowStatus, Telemetry, VelocityCommand

FOLLOW, HOLD, RTH = "FOLLOW", "HOLD", "RTH"


@dataclass
class SupervisorConfig:
    rth_altitude: float = 6.0        # climb to this before/while returning
    rth_speed: float = 3.0           # m/s
    rth_kp: float = 0.8              # speed = kp * distance, capped at rth_speed
    arrive_radius: float = 1.0       # m: switch to HOLD inside this
    low_battery: float = 0.20        # fraction: auto RTH below this
    geofence_radius: float = 80.0    # m from home: auto RTH beyond this (0 = off)
    telemetry_timeout_s: float = 1.0 # stale telemetry -> HOLD


class FlightSupervisor:
    """Decides which behaviour owns the drone and enforces failsafes.

    FOLLOW  pass the follow controller's command through
    HOLD    hover
    RTH     fly back to where telemetry first reported us, then HOLD
            (also when the follow behaviour gives up on a lost target)

    Failsafes (low battery, geofence) latch RTH: an operator request for
    FOLLOW is refused until the cause clears.
    """

    def __init__(self, cfg: SupervisorConfig | None = None):
        self.cfg = cfg or SupervisorConfig()
        self.mode = FOLLOW
        self.reason = ""
        self.home: Optional[tuple[float, float, float]] = None
        self._last_telem_t: Optional[float] = None
        self._last_telem_stamp: Optional[float] = None
        self._failsafe_cause: Optional[str] = None

    def request(self, mode: str) -> bool:
        """Operator command. Returns False if a failsafe refuses it."""
        if mode == FOLLOW and self._failsafe_cause is not None:
            return False
        self.mode, self.reason = mode, "operator"
        return True

    def _distance_home(self, telem: Telemetry) -> float:
        return math.hypot(self.home[0] - telem.position[0], self.home[1] - telem.position[1])

    def _check_failsafes(self, telem: Telemetry) -> Optional[str]:
        c = self.cfg
        if telem.battery < c.low_battery:
            return "low battery"
        if c.geofence_radius > 0 and self._distance_home(telem) > c.geofence_radius:
            return "geofence"
        # geofence clears with hysteresis once we're well inside again
        if self._failsafe_cause == "geofence" and self._distance_home(telem) > 0.8 * c.geofence_radius:
            return "geofence"
        return None

    def step(self, follow: FollowStatus, telem: Optional[Telemetry], now: float) -> FollowStatus:
        if telem is not None:
            if self.home is None:
                self.home = telem.position
            if telem.timestamp != self._last_telem_stamp:
                self._last_telem_stamp, self._last_telem_t = telem.timestamp, now
            self._failsafe_cause = self._check_failsafes(telem)
            if self._failsafe_cause and self.mode != RTH and not (self.mode == HOLD and self.reason == "arrived home"):
                self.mode, self.reason = RTH, self._failsafe_cause

        if self.mode == FOLLOW and follow.state == "GAVE_UP":
            # searched, waited at the last-seen spot, nothing: come home
            self.mode, self.reason = RTH, "target lost"
        if self.mode == FOLLOW:
            return follow

        stale = (telem is None or self._last_telem_t is None
                 or now - self._last_telem_t > self.cfg.telemetry_timeout_s)
        if self.mode == RTH and not stale:
            cmd, arrived = self._return_home(telem)
            if arrived:
                self.mode, self.reason = HOLD, "arrived home"
            else:
                return FollowStatus(f"RTH ({self.reason})", follow.target, cmd,
                                    {"home_dist": self._distance_home(telem)})
        label = "HOLD (no telemetry)" if self.mode == RTH and stale else f"HOLD ({self.reason})"
        return FollowStatus(label, follow.target, VelocityCommand.hover())

    def _return_home(self, telem: Telemetry) -> tuple[VelocityCommand, bool]:
        c = self.cfg
        dx, dy = self.home[0] - telem.position[0], self.home[1] - telem.position[1]
        dist = math.hypot(dx, dy)
        alt_err = c.rth_altitude - telem.altitude
        if dist < c.arrive_radius:
            return VelocityCommand.hover(), True
        # yaw is a compass heading (0 = +y, clockwise), so bearing = atan2(east, north)
        bearing = math.atan2(dx, dy)
        yaw_err = (bearing - telem.yaw + math.pi) % (2 * math.pi) - math.pi
        speed = min(c.rth_speed, c.rth_kp * dist)
        if alt_err > 1.0:            # climb first, travel once near RTH altitude
            speed *= 0.3
        return VelocityCommand(
            forward=speed * math.cos(yaw_err),
            right=speed * math.sin(yaw_err),
            up=max(-1.0, min(1.0, alt_err)),
            yaw_rate=max(-1.0, min(1.0, 1.5 * yaw_err)),   # nose toward home so the front sensor leads
        ), False
