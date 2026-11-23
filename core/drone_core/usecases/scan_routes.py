"""Scan routes for each scene (v3, Evan's idea A).

A route is a list of waypoints, each with a point to point the nose at and
how long to hover there. The cameras keep watching the whole way, so the
belief's negative information and re-lock work during the flight ("scan on
the way"), not only at the stops.

    WALL  (high)  swing wide past the end they went round (the far side of the
                  turn gives the widest view), look down the hidden side near
                  first, then fly along it looking further down the wall.
    WALL  (low)   first look over it from our side, 4 m off and high enough
                  to see 2 m past it (not straight over the top: the cameras
                  tilt only 20 deg down, so right behind it stays hidden
                  without a gimbal), then round the end like a high wall.
    HALL          go to the mouth on the centre line, then down the hall,
                  looking from where they were last seen to the far end.
    ROOM          watch the door; then look along the two sides next to it
                  from just past each corner (with the door still at the edge
                  of view), back to the door, and round again until found
                  (the belief only knows the door they used; other doors are
                  what the side looks are for).

Every point is kept inside the line-of-sight ceiling (max_alt) and above min_alt.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from drone_core.usecases.obstacle_type import HALL, ROOM, WALL, Scene


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    lx: float               # look at (x, y)
    ly: float
    dwell: float = 0.0      # s to hover once there
    label: str = ""


@dataclass
class RouteConfig:
    swing_out: float = 3.0      # m past the wall end
    side_off: float = 4.0       # m off the hidden side of a wall
    leg: float = 4.0            # m between waypoints along a wall / down a hall
    low_off: float = 4.0        # m off our side of a low wall
    low_beyond: float = 2.0     # m past a low wall that must be visible
    head_h: float = 1.6
    see_margin: float = 0.3
    door_off: float = 4.5       # m out from the door
    door_hold: float = 5.0      # s watching the door between looks round the sides
    corner_off: float = 3.0     # m past a corner
    max_len: float = 16.0       # m of wall / hall scanned at most
    max_alt: float = 10.0
    min_alt: float = 1.5


def _p(a, b, k=1.0):
    return (a[0] + b[0] * k, a[1] + b[1] * k)


def build_route(sc: Scene, base_alt: float, cfg: RouteConfig | None = None) -> list:
    c = cfg or RouteConfig()
    z = max(c.min_alt, min(c.max_alt, base_alt))
    if sc.kind == WALL and sc.end is not None:
        e, o, n = sc.end, sc.out, sc.hidden
        back = (-o[0], -o[1])
        length = min(c.max_len, max(sc.length, 8.0))     # only the near part may be mapped yet
        pts = []
        if sc.low and sc.top is not None:
            # first a look over it from our side, high enough to see a head
            # low_beyond past it (right behind it stays hidden: the cameras only
            # tilt 20 deg down), then round the end like any wall
            D = c.low_off + c.low_beyond
            f = c.low_off / D
            zz = (sc.top + c.see_margin - c.head_h * f) / (1.0 - f)
            zz = max(z, min(c.max_alt, zz))
            p = _p(_p(e, back, 2.0), n, -c.low_off)
            look = _p(_p(e, back, 4.0), n, c.low_beyond)
            pts.append(Waypoint(p[0], p[1], zz, look[0], look[1], 0.5, "over"))
        p0 = _p(e, o, c.swing_out)
        near = _p(e, n, 2.0)
        pts.append(Waypoint(p0[0], p0[1], z, near[0], near[1], 0.4, "round"))          # look down near the end
        p1 = _p(p0, n, c.side_off)
        far = _p(_p(e, back, 6.0), n, 1.0)
        pts.append(Waypoint(p1[0], p1[1], z, far[0], far[1], 0.6, "far side"))         # then far along the wall
        s = c.leg
        while s <= length + 1e-6:
            p = _p(_p(e, back, s), n, c.side_off)
            look = _p(_p(e, back, s + 6.0), n, 1.0)
            pts.append(Waypoint(p[0], p[1], z, look[0], look[1], 0.0, "along"))
            s += c.leg
        return pts
    if sc.kind == HALL and sc.centre is not None:
        cen, a = sc.centre, sc.axis
        depth = min(c.max_len, max(sc.depth, c.leg))
        pts = []
        p0 = _p(cen, a, -1.0)
        look = _p(cen, a, 8.0)
        pts.append(Waypoint(p0[0], p0[1], z, look[0], look[1], 0.5, "mouth"))
        s = c.leg
        while s <= depth - 1.0 + 1e-6:
            p = _p(cen, a, s)
            look = _p(cen, a, s + 8.0)
            pts.append(Waypoint(p[0], p[1], z, look[0], look[1], 0.0, "down hall"))
            s += c.leg
        return pts
    if sc.kind == ROOM and sc.door is not None:
        d = sc.door
        h = sc.heading
        outw = (-h[0], -h[1])                               # out of the building, toward us
        side = (h[1], -h[0])                                # along the door's face
        wp, wm = 4.0, 4.0                                   # how far the face runs each way from the door
        if sc.box is not None:
            x0, y0, x1, y1 = sc.box
            ext = [((x - d[0]) * side[0] + (y - d[1]) * side[1]) for x in (x0, x1) for y in (y0, y1)]
            wp = max(1.0, min(8.0, max(ext)))
            wm = max(1.0, min(8.0, -min(ext)))
        p0 = _p(d, outw, c.door_off)
        pts = [Waypoint(p0[0], p0[1], z, d[0], d[1], c.door_hold, "door")]
        for sgn, w in ((1.0, wp), (-1.0, wm)):
            corner = _p(d, side, sgn * w)
            p = _p(_p(corner, side, sgn * c.corner_off), outw, c.corner_off)
            look = _p(corner, h, 4.0)                       # along the side wall, door at the edge of view
            pts.append(Waypoint(p[0], p[1], z, look[0], look[1], 1.0, "side"))
        return pts                                          # RouteRunner loops a room's route
    return []


class RouteRunner:
    """Walks a route: fly to each waypoint, hover its dwell, move on."""

    def __init__(self, reach_m: float = 1.0, reach_z: float = 0.8):
        self.reach_m, self.reach_z = reach_m, reach_z
        self.route: list = []
        self.i = 0
        self._arrived: Optional[float] = None

    def start(self, route: list, loop: bool = False) -> None:
        self.route, self.i, self._arrived = list(route), 0, None
        self.loop = loop

    def clear(self) -> None:
        self.start([])

    @property
    def done(self) -> bool:
        return self.i >= len(self.route)

    def current(self, pos, t: float) -> Optional[Waypoint]:
        while not self.done:
            w = self.route[self.i]
            if math.hypot(w.x - pos[0], w.y - pos[1]) < self.reach_m and abs(w.z - pos[2]) < self.reach_z:
                if self._arrived is None:
                    self._arrived = t
                if t - self._arrived >= w.dwell:
                    self.i += 1
                    self._arrived = None
                    if self.done and getattr(self, "loop", False):
                        self.i = 0
                    continue
            return w
        return None
