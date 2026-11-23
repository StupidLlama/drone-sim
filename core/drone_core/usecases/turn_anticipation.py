"""See the turn coming (v4, Evan: "if the target is heading toward the
obstacle speed up, if they turn, turn with them but from the far side to
ensure max view").

While following, look along the target's path on the 3-D map's person-height
layer for the two things that make people vanish:

    a wall ahead       their path runs into an obstacle within a few seconds:
                       they must turn one way or the other (the side with more
                       open space is the likely one)
    a corner ahead     an obstacle runs beside their path and ends: they may
                       turn round its end, toward the obstacle side

When either is coming up, the advice is to

  * close in (follow at close_frac of the normal distance), so the drone is
    near when they turn and the obstacle hides less of the view, and
  * move to the OUTSIDE of the turn (swing_m sideways, away from the side they
    will turn to): from the far side the line of sight wraps round the corner,
    from the inside the corner cuts it.

Pure geometry on the map; no map or a slow target = no advice (v3 behaviour).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class AnticipateConfig:
    enabled: bool = True
    look_s: float = 3.0          # s of their path to check
    look_min_m: float = 3.0      # ... but at least this far
    side_m: float = 2.5          # an obstacle this close beside the path = they may turn round it
    step: float = 0.2
    close_frac: float = 0.65     # follow distance while a turn is coming
    min_close_m: float = 3.0     # ... but never closer than this (m)
    swing_m: float = 2.5         # m to the outside of the turn
    gain: float = 0.8            # m/s per m of sideways error
    max_side: float = 2.0        # m/s sideways at most
    min_speed: float = 0.4       # m/s: slower than this = standing, nothing to anticipate
    hold_s: float = 1.5          # keep the advice this long after the reason has passed (they're turning)


@dataclass
class Advice:
    dist_scale: float            # multiply the follow distance by this
    side: tuple                  # unit world (x, y): the outside of the turn (move this way)
    offset: float                # wanted sideways offset from their path line, m (along side)
    reason: str                  # "wall ahead" | "corner ahead"
    turn: int                    # +1 = they'll likely turn right, -1 = left


class TurnAnticipator:
    def __init__(self, cfg: AnticipateConfig | None = None):
        self.cfg = cfg or AnticipateConfig()
        self._last: Optional[Advice] = None
        self._last_t = -1e9

    def reset(self) -> None:
        self._last, self._last_t = None, -1e9

    @staticmethod
    def _occ(g, pts):
        occ, org, res = g
        ix = np.floor((pts[..., 0] - org[0]) / res).astype(int)
        iy = np.floor((pts[..., 1] - org[1]) / res).astype(int)
        ok = (ix >= 0) & (ix < occ.shape[1]) & (iy >= 0) & (iy < occ.shape[0])
        o = np.zeros(ix.shape, bool)
        o[ok] = occ[iy[ok], ix[ok]]
        return o

    def _free_len(self, g, p, u, max_m):
        s = np.arange(self.cfg.step, max_m + 1e-9, self.cfg.step)
        pts = np.asarray(p)[None, :] + np.asarray(u)[None, :] * s[:, None]
        o = self._occ(g, pts)
        return float(s[o.argmax()]) if o.any() else max_m

    def advise(self, t: float, target_xy, target_vel, vmap) -> Optional[Advice]:
        c = self.cfg
        adv = self._advise(target_xy, target_vel, vmap) if c.enabled else None
        if adv is not None:
            self._last, self._last_t = adv, t
            return adv
        if self._last is not None and t - self._last_t < c.hold_s:
            return self._last
        self._last = None
        return None

    def _advise(self, target_xy, target_vel, vmap) -> Optional[Advice]:
        c = self.cfg
        if vmap is None or target_xy is None or target_vel is None:
            return None
        sp = math.hypot(target_vel[0], target_vel[1])
        if sp < c.min_speed:
            return None
        om = vmap.obstacle_mask()
        if om is None:
            return None
        g = om
        u = np.array([target_vel[0] / sp, target_vel[1] / sp])
        right = np.array([u[1], -u[0]])                       # yaw convention: x = sin, y = cos
        p0 = np.array([float(target_xy[0]), float(target_xy[1])])
        L = max(c.look_min_m, sp * c.look_s)
        s = np.arange(c.step, L + 1e-9, c.step)
        path = p0[None, :] + u[None, :] * s[:, None]
        # 1. wall ahead: they must turn; the likely way is the more open side
        ahead = self._occ(g, path)
        if ahead.any():
            k = int(ahead.argmax())
            q = path[max(0, k - 3)]                               # just before it
            fr = self._free_len(g, q, right, 6.0)
            fl = self._free_len(g, q, -right, 6.0)
            if abs(fr - fl) >= 1.0:
                turn = 1 if fr > fl else -1
                out = -right * turn
                return Advice(c.close_frac, (float(out[0]), float(out[1])), c.swing_m, "wall ahead", turn)
        # 2. corner ahead: an obstacle beside the path that ends before L
        lat = np.arange(c.step, c.side_m + 1e-9, c.step)
        for sgn in (1, -1):
            side_pts = path[:, None, :] + (sgn * right)[None, None, :] * lat[None, :, None]
            beside = self._occ(g, side_pts).any(axis=1)          # (len(s),)
            if not beside[:3].any():
                continue                                          # nothing beside them now
            idx = np.nonzero(~beside)[0]
            idx = idx[idx > 2]
            if len(idx) == 0:
                continue                                          # the wall goes on: no corner yet
            turn = sgn
            out = -right * turn
            return Advice(c.close_frac, (float(out[0]), float(out[1])), c.swing_m, "corner ahead", turn)
        return None

    def side_speed(self, adv: Advice, target_xy, drone_xy) -> tuple:
        """World (vx, vy) that moves the drone toward adv.offset metres on the
        outside of the turn, measured from the target's path line."""
        c = self.cfg
        off = (drone_xy[0] - target_xy[0]) * adv.side[0] + (drone_xy[1] - target_xy[1]) * adv.side[1]
        v = max(-c.max_side, min(c.max_side, c.gain * (adv.offset - off)))
        return v * adv.side[0], v * adv.side[1]
