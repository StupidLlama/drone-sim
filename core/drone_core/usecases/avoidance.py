from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from drone_core.domain.models import Telemetry, VelocityCommand


@dataclass
class AvoidanceConfig:
    stop_distance: float = 1.5     # m: no forward motion closer than this
    slow_distance: float = 4.0     # m: start scaling forward speed down
    side_distance: float = 1.2     # m: push sideways away from walls closer than this
    side_push: float = 1.0         # m/s max sideways push from side sensors
    steer_speed: float = 1.5       # m/s max sidestep around something ahead
    min_altitude: float = 1.0      # m: never descend below this (uses "down" range or altitude)
    # 3-D dodging (needs a 3-D map: front_up / front_down / up / below keys).
    vertical_dodge: bool = False   # allow going over/under something ahead (branches)
    climb_speed: float = 1.0       # m/s max vertical dodge speed
    max_altitude: float = 15.0
    overhead_stop: float = 1.0     # m: no climbing with something closer than this above
    underneath_stop: float = 1.0   # m: no descending with something closer than this below
    max_dodge_climb: float = 3.0   # m: climbed this much and still blocked -> sidestep instead
    max_dodge_s: float = 4.0       # s: same, if the vertical dodge isn't getting anywhere


class AvoidanceFilter:
    """Safety layer on top of any behaviour (follow, RTH, manual).

    It never adds motion toward an obstacle. Ahead of an obstacle it slows the
    forward speed and sidesteps toward the more open diagonal, so the drone
    goes around a pillar instead of parking in front of it.

    Range keys: front, back, left, right, front_left, front_right, down, and
    from a 3-D map: front_up, front_down, up (overhead), below (obstacles
    underneath, not the ground). A missing key means "nothing within range".

    With vertical_dodge on, a blockage that is open above or below (a branch,
    a fence top, a sign) is passed by climbing or dropping instead of
    sidestepping, whichever way is clearly more open.
    """

    def __init__(self, cfg: AvoidanceConfig | None = None):
        self.cfg = cfg or AvoidanceConfig()
        self._steer_dir = 0.0   # remembered so we don't dither left/right
        self._steer_vert = 0.0  # +1 climb / -1 drop over/under the thing ahead (3-D maps)
        self._dodge_alt = 0.0   # altitude when the vertical dodge started
        self._dodge_t = 0.0

    def apply(self, cmd: VelocityCommand, telem: Optional[Telemetry]) -> VelocityCommand:
        if telem is None:
            return cmd
        c = self.cfg
        r = telem.ranges or {}
        inf = float("inf")
        fwd, right, up = cmd.forward, cmd.right, cmd.up

        # The nearest thing in the forward cone decides braking.
        front = min(r.get("front", inf), r.get("front_left", inf) * 1.2, r.get("front_right", inf) * 1.2)
        if fwd > 0 and front < c.slow_distance:
            blockage = 1.0 - max(0.0, front - c.stop_distance) / (c.slow_distance - c.stop_distance)
            fwd *= (1.0 - blockage)
            # pick a way round once, keep it until the way ahead is clear
            if self._steer_dir == 0.0 and self._steer_vert == 0.0:
                l = min(r.get("front_left", inf), r.get("left", inf))
                rt = min(r.get("front_right", inf), r.get("right", inf))
                vert = self._vertical_choice(r, telem, max(l, rt))
                if vert:
                    self._steer_vert = vert
                    self._dodge_alt = telem.altitude
                    self._dodge_t = telem.timestamp
                elif abs(rt - l) > 1.5 or (rt == inf) != (l == inf):
                    self._steer_dir = 1.0 if rt > l else -1.0
                else:
                    # Both sides look similar: go around on the side the
                    # behaviour is already turning toward (where it wants to go).
                    self._steer_dir = 1.0 if cmd.yaw_rate >= 0 else -1.0
            if self._steer_vert and (abs(telem.altitude - self._dodge_alt) > c.max_dodge_climb
                                     or telem.timestamp - self._dodge_t > c.max_dodge_s):
                # it's taller (or deeper) than it looked: go round it instead
                self._steer_vert = 0.0
                l = min(r.get("front_left", inf), r.get("left", inf))
                rt = min(r.get("front_right", inf), r.get("right", inf))
                self._steer_dir = 1.0 if rt >= l else -1.0
            right += self._steer_dir * c.steer_speed * blockage
            up += self._steer_vert * c.climb_speed * max(blockage, 0.5)
        elif front >= c.slow_distance + 0.5:      # a little hysteresis against flicker
            self._steer_dir = 0.0
            self._steer_vert = 0.0

        back = r.get("back")
        if back is not None and fwd < 0 and back <= c.stop_distance:
            fwd = 0.0

        # Side sensors have the last word: never slide into a wall.
        left, rgt = r.get("left"), r.get("right")
        if left is not None and left < c.side_distance:
            right = max(right, c.side_push * (1 - left / c.side_distance))
        if rgt is not None and rgt < c.side_distance:
            right = min(right, -c.side_push * (1 - rgt / c.side_distance))

        height = r.get("down", telem.altitude)
        if height is not None and height <= c.min_altitude and up < 0:
            up = 0.0
        if up > 0 and (r.get("up", inf) < c.overhead_stop or telem.altitude >= c.max_altitude):
            up = 0.0                       # don't climb into the canopy
        if up < 0 and r.get("below", inf) < c.underneath_stop:
            up = 0.0                       # don't sink onto a branch / car roof

        return VelocityCommand(fwd, right, up, cmd.yaw_rate)

    def _vertical_choice(self, r: dict, telem: Telemetry, lateral: float) -> float:
        """+1 climb, -1 drop, 0 = sidestep instead."""
        c, inf = self.cfg, float("inf")
        if not c.vertical_dodge:
            return 0.0
        alt = telem.altitude
        up = min(r.get("front_up", inf), r.get("up", inf)) \
            if alt < c.max_altitude - 1.0 and r.get("up", inf) > 1.5 * c.overhead_stop else -1.0
        down = min(r.get("front_down", inf), r.get("below", inf)) \
            if alt > c.min_altitude + 1.5 and r.get("below", inf) > 1.5 * c.underneath_stop else -1.0
        # Open above/below and blocked straight ahead = something thin and
        # horizontal (branch, wire, bar): going over beats sidestepping along it.
        # Prefer climbing: dropping brings the drone down toward the person.
        for way, clear in ((1.0, up), (-1.0, down)):
            if clear >= c.slow_distance and (clear == inf or clear > lateral + 1.5):
                return way
        return 0.0
