"""Where should the drone go to look for a hidden target? (v2)

Search theory's rule (Koopman; RPF-Search 2025; Hague & Wolek 2025): go where
you would see the most remaining probability, sooner rather than later.

    score(viewpoint) = sum over particles of  weight x P(detect | range) x line-of-sight
                       -----------------------------------------------------------------
                                       flight time + 2 s

Candidates sit on rings round the heaviest clusters of the belief, plus the
drone's own position (so it only moves when that pays). When the map says a
wall is in the way, the candidate is raised just enough to see over it (the
obstacle-height query), never above the line-of-sight ceiling and never into
something overhead (<1 m clearance rule).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class ViewpointConfig:
    every_s: float = 0.33          # re-plan rate (~3 Hz)
    rings: tuple = (4.0, 7.5)      # m around each cluster
    n_angles: int = 12
    clusters: int = 2              # rings round this many top clusters
    particles: int = 200           # particles scored per candidate
    speed: float = 3.0             # m/s, for the flight-time term
    time_bias_s: float = 2.0
    hysteresis: float = 1.2        # a new viewpoint must score 20% better to switch
    max_alt: float = 10.0          # line-of-sight ceiling
    min_alt: float = 1.5
    overhead_clear: float = 1.0    # m free above a climbed viewpoint
    see_over_margin: float = 0.3   # m above the wall top
    head_h: float = 1.6            # aim the line of sight at the head
    obstacle_margin: float = 1.0   # m: no viewpoints this close (vertically) to an obstacle top
    unknown_credit: float = 0.6    # unmapped line of sight: probably clear, count 60%


@dataclass
class Viewpoint:
    x: float
    y: float
    z: float
    face: tuple          # (x, y) to point the nose at
    score: float
    climbed: bool = False


class ViewpointPlanner:
    def __init__(self, cfg: ViewpointConfig | None = None, seed: int = 11):
        self.cfg = cfg or ViewpointConfig()
        self.rng = np.random.default_rng(seed)
        self.current: Optional[Viewpoint] = None
        self._t = -1e9

    def reset(self) -> None:
        self.current = None
        self._t = -1e9

    def see_over_height(self, vmap, eye_xy, target_xy) -> Optional[float]:
        """Lowest eye height that sees the head at target_xy over every mapped
        obstacle on the way. 0 = nothing in the way, None = can't (wall right
        next to the person, or above the ceiling)."""
        c = self.cfg
        if vmap is None:
            return 0.0
        r = vmap.top_between(eye_xy, target_xy, 0.3, 0.5, with_dist=True)
        if r is None:
            return 0.0
        h_wall, s = r
        D = math.hypot(target_xy[0] - eye_xy[0], target_xy[1] - eye_xy[1])
        if h_wall + c.see_over_margin > c.head_h and hasattr(vmap, "free_above"):
            # Only trust the height if the map has seen open air above that top:
            # the camera may simply not have looked higher (a tall pillar, a tree).
            wx = eye_xy[0] + (target_xy[0] - eye_xy[0]) * s / max(D, 1e-6)
            wy = eye_xy[1] + (target_xy[1] - eye_xy[1]) * s / max(D, 1e-6)
            if not vmap.free_above(wx, wy, h_wall, 0.6):
                return None
        D = math.hypot(target_xy[0] - eye_xy[0], target_xy[1] - eye_xy[1])
        f = s / max(D, 1e-6)
        if h_wall + c.see_over_margin <= c.head_h:
            return 0.0
        if f > 0.92:
            return None
        z = (h_wall + c.see_over_margin - c.head_h * f) / (1.0 - f)
        return z if z <= c.max_alt else None

    def plan(self, t: float, drone, yaw: float, base_alt: float, belief, cams, vmap,
             force: bool = False) -> Optional[Viewpoint]:
        c = self.cfg
        if not force and self.current is not None and t - self._t < c.every_s:
            return self.current
        self._t = t
        cl = belief.clusters(max(3, c.clusters))
        if not cl:
            return self.current
        px, py, pm = belief.sample(c.particles, self.rng)
        # For planning, INSIDE particles count at their door: watching the door
        # is how you see someone come out of a shop.
        pm = None
        pw = np.full(len(px), 1.0 / len(px))
        cands = [(drone[0], drone[1], drone[2], cl[0].x, cl[0].y, False)]
        for k in cl[:c.clusters]:
            for r in c.rings:
                for i in range(c.n_angles):
                    a = 2 * math.pi * i / c.n_angles
                    x, y = k.x + r * math.sin(a), k.y + r * math.cos(a)
                    z = base_alt
                    climbed = False
                    need = self.see_over_height(vmap, (x, y), (k.x, k.y))
                    if need is not None and need > z:
                        z, climbed = min(c.max_alt, need), True
                    if vmap is not None:
                        top = vmap.height_at(x, y)
                        if top is not None and top > z - c.obstacle_margin:
                            continue                       # inside / on top of something
                        if climbed and vmap.occupied_above(x, y, base_alt, z + c.overhead_clear):
                            z, climbed = base_alt, False   # canopy above: don't climb into it
                    cands.append((x, y, max(c.min_alt, z), k.x, k.y, climbed))
        best, cur_score = None, None
        for x, y, z, fx, fy, climbed in cands:
            face_yaw = math.atan2(fx - x, fy - y)
            vis = belief.visible(px, py, (x, y, z), face_yaw, cams, vmap, pm, c.unknown_credit)
            gain = float((pw * vis).sum())
            d = math.dist((x, y, z), tuple(drone[:3]))
            s = gain / (d / c.speed + c.time_bias_s)
            if best is None or s > best.score:
                best = Viewpoint(x, y, z, (fx, fy), s, climbed)
            if self.current is not None and math.dist((x, y), (self.current.x, self.current.y)) < 0.5:
                cur_score = s
        if self.current is not None:
            # re-score where we are going; only switch for a clear improvement
            if cur_score is None:
                fy_ = math.atan2(self.current.face[0] - self.current.x, self.current.face[1] - self.current.y)
                vis = belief.visible(px, py, (self.current.x, self.current.y, self.current.z), fy_, cams, vmap, pm,
                                     c.unknown_credit)
                d = math.dist((self.current.x, self.current.y, self.current.z), tuple(drone[:3]))
                cur_score = float((pw * vis).sum()) / (d / c.speed + c.time_bias_s)
            if best is not None and best.score < c.hysteresis * cur_score:
                self.current.score = cur_score
                return self.current
        self.current = best
        return best
