"""Where is the hidden target? A map-aware particle filter (v2).

v1 guessed one point: the last position plus the last velocity, for up to
8 s. Real people stop, turn round corners, double back and walk into shops,
so that single point ran ahead of them (9.6 m off in `decoy_at_exit`).

Here the guess is a cloud of particles. Each particle is one "story" of what
the person did: x, y, speed, heading and a mode:

    WALK    keeps walking, speed relaxes to its own preferred speed
    TURN    turns at 30-90 deg/s for ~2.5 s (round a corner, a U-turn)
    STOP    stands still (planned stop ~0.9 s, then waits ~25 s on average)
    INSIDE  walked into something solid on the map: a door, a shop. Sits there.

Every tick the cloud is
  1. predicted with its mode's motion (walls stop particles; they slide along
     them or go INSIDE),
  2. weighted down where the cameras can clearly see but nobody matching was
     detected ("negative information": if I can see that spot and it's empty,
     they're not there),
  3. resampled when too few particles carry the weight.

Numbers come from pedestrian research (see research/Drone v2 lost target
estimation.md): walking 1.3 +- 0.26 m/s, stopping in 0.6-0.9 s, a turn or stop
about every 6.7 s, 25 deg heading noise over ~5 s.

Depends only on numpy and an optional obstacle map with obstacle_mask() and
lines_blocked() (the VoxelMap satisfies it; ports.ObstacleMap).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

WALK, TURN, STOP, INSIDE = 0, 1, 2, 3
MODE_NAMES = ("walk", "turn", "stop", "inside")


@dataclass
class BeliefConfig:
    n: int = 2000
    seed: int = 7
    # start
    speed_sd_frac: float = 0.15
    heading_sd_deg: float = 12.0
    walk_frac: float = 0.75
    turn_frac: float = 0.15                # rest = STOP
    default_walk_speed: float = 1.3        # preferred speed if it was standing when lost
    max_walk_speed: float = 2.0
    max_jog_speed: float = 4.0
    # motion (rates per second)
    walk_to_turn: float = 0.10
    walk_to_stop: float = 0.05
    turn_to_walk: float = 0.40             # mean turn 2.5 s
    stop_to_walk: float = 0.04             # mean wait 25 s
    inside_to_walk: float = 0.03
    turn_rate_deg: tuple = (30.0, 90.0)
    heading_noise: float = 0.15            # rad / sqrt(s)
    speed_tau: float = 0.5                 # s, relax to preferred speed
    stop_tau: float = 0.35                 # s, braking
    enter_prob: float = 0.25               # chance a particle that hits a wall goes INSIDE
    # map
    person_z: tuple = (0.3, 1.8)           # heights that block walking
    mask_every_s: float = 0.5
    # negative information
    pd_near: float = 0.85                  # detection chance per look at <= pd_near_m
    pd_far: float = 0.3                    # ... at pd_far_m, 0 beyond
    pd_near_m: float = 10.0
    pd_far_m: float = 20.0
    look_s: float = 0.25                   # P_d is per look of this long
    min_range: float = 1.0
    chest_h: float = 1.0                   # height of the point we check line of sight to
    unknown_max: float = 0.85              # line mostly through unmapped space: no info
    near_person_m: float = 0.8             # someone else stands there: could hide the target
    fov_margin_deg: float = 4.0
    # resampling
    resample_ess: float = 0.5
    birth_frac: float = 0.02
    birth_sd_m: float = 1.5
    jitter_m: float = 0.05
    # clusters
    cluster_cell_m: float = 1.0
    cluster_radius_m: float = 1.5


@dataclass
class Cluster:
    x: float
    y: float
    w: float           # share of the belief
    spread: float      # m, weighted std of its particles
    mode: str          # most common mode in it


class TargetBelief:
    def __init__(self, cfg: BeliefConfig | None = None):
        self.cfg = cfg or BeliefConfig()
        self.rng = np.random.default_rng(self.cfg.seed)
        self.active = False
        self.t: Optional[float] = None
        self.t0: Optional[float] = None
        self._mask = None
        self._mask_t = -1e9
        self._clusters: Optional[list] = None

    # --- life cycle ----------------------------------------------------------
    def reset(self) -> None:
        self.active = False
        self.t = self.t0 = None
        self._clusters = None

    def seed(self, pos, vel, t: float) -> None:
        """Start a cloud at the last position and velocity seen (at time t)."""
        c, n, rng = self.cfg, self.cfg.n, self.rng
        v0 = float(math.hypot(vel[0], vel[1]))
        h0 = math.atan2(vel[0], vel[1])                     # compass heading, 0 = north
        self.x = np.full(n, float(pos[0])) + rng.normal(0, 0.15, n)
        self.y = np.full(n, float(pos[1])) + rng.normal(0, 0.15, n)
        vmax = c.max_jog_speed if v0 > 1.8 else c.max_walk_speed
        if v0 >= 0.4:
            self.vpref = np.clip(rng.normal(v0, c.speed_sd_frac * v0, n), 0.3, vmax)
            self.h = h0 + rng.normal(0, math.radians(c.heading_sd_deg), n)
            self.v = np.clip(rng.normal(v0, 0.1 * v0, n), 0.0, vmax)
            u = rng.random(n)
            self.mode = np.where(u < c.walk_frac, WALK, np.where(u < c.walk_frac + c.turn_frac, TURN, STOP))
        else:                                               # was standing: may set off any way
            self.vpref = np.clip(rng.normal(c.default_walk_speed, 0.26, n), 0.3, c.max_walk_speed)
            self.h = rng.uniform(-math.pi, math.pi, n)
            self.v = np.full(n, v0)
            self.mode = np.where(rng.random(n) < 0.6, STOP, WALK)
        self.mode = self.mode.astype(np.int8)
        self.turn = np.radians(rng.uniform(*c.turn_rate_deg, n)) * rng.choice((-1.0, 1.0), n)
        self.w = np.full(n, 1.0 / n)
        self.active = True
        self.t = self.t0 = t
        self._clusters = None

    # --- motion ----------------------------------------------------------------
    def set_map(self, obstacle_map, t: float) -> None:
        """Refresh the 2-D 'can't walk here' mask from the 3-D map (cheap, 2 Hz)."""
        if obstacle_map is None or t - self._mask_t < self.cfg.mask_every_s:
            return
        self._mask_t = t
        self._mask = obstacle_map.obstacle_mask(*self.cfg.person_z)

    def _blocked(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        if self._mask is None:
            return np.zeros(len(x), bool)
        mask, origin, res = self._mask
        ix = np.floor((x - origin[0]) / res).astype(int)
        iy = np.floor((y - origin[1]) / res).astype(int)
        ok = (ix >= 0) & (ix < mask.shape[1]) & (iy >= 0) & (iy < mask.shape[0])
        out = np.zeros(len(x), bool)
        out[ok] = mask[iy[ok], ix[ok]]
        return out

    def predict(self, t: float) -> None:
        if not self.active or t <= self.t:
            return
        total = t - self.t
        steps = max(1, int(math.ceil(total / 0.2)))
        for _ in range(steps):
            self._step(total / steps)
        self.t = t
        self._clusters = None

    def _step(self, dt: float) -> None:
        c, rng, n = self.cfg, self.rng, self.cfg.n
        p = lambda rate: 1.0 - math.exp(-rate * dt)
        u = rng.random(n)
        m = self.mode
        new = m.copy()
        walk, turn, stop, inside = m == WALK, m == TURN, m == STOP, m == INSIDE
        new[walk & (u < p(c.walk_to_turn))] = TURN
        new[walk & (u >= p(c.walk_to_turn)) & (u < p(c.walk_to_turn) + p(c.walk_to_stop))] = STOP
        new[turn & (u < p(c.turn_to_walk))] = WALK
        go = (stop | inside) & (u < np.where(stop, p(c.stop_to_walk), p(c.inside_to_walk)))
        new[go] = WALK
        started_turn = (new == TURN) & (m != TURN)
        k = int(started_turn.sum())
        if k:
            self.turn[started_turn] = np.radians(rng.uniform(*c.turn_rate_deg, k)) * rng.choice((-1.0, 1.0), k)
        if go.any():                                      # set off again: same way, or back
            back = go & (rng.random(n) < 0.3)
            self.h[back] += math.pi
            self.h[go] += rng.normal(0, 0.5, int(go.sum()))
            self.v[go] = 0.3
        self.mode = m = new

        moving = (m == WALK) | (m == TURN)
        target_v = np.where(m == TURN, 0.9 * self.vpref, self.vpref)
        self.v = np.where(moving, self.v + (target_v - self.v) * min(1.0, dt / c.speed_tau),
                          self.v * math.exp(-dt / c.stop_tau))
        self.v[m == INSIDE] = 0.0
        self.h = self.h + np.where(m == TURN, self.turn * dt, 0.0) \
            + np.where(moving, rng.normal(0, c.heading_noise * math.sqrt(dt), n), 0.0)
        nx = self.x + self.v * np.sin(self.h) * dt
        ny = self.y + self.v * np.cos(self.h) * dt
        hit = self._blocked(nx, ny) & (self.v > 0.05)
        if hit.any():
            # Walked into a wall. Most slide along it (turn 90 deg either way),
            # some went through a door we can't see (INSIDE: wait in there).
            enter = hit & (rng.random(n) < c.enter_prob) & (m != INSIDE)
            slide = hit & ~enter
            self.h[slide] += rng.choice((-0.5 * math.pi, 0.5 * math.pi), int(slide.sum()))
            # INSIDE particles stay at the door they went through (where they'll come out)
            nx[hit], ny[hit] = self.x[hit], self.y[hit]
            self.mode[enter] = INSIDE
            self.v[enter] = 0.0
        self.x, self.y = nx, ny

    # --- negative information ---------------------------------------------------
    def pd(self, r: np.ndarray) -> np.ndarray:
        c = self.cfg
        far = c.pd_near + (c.pd_far - c.pd_near) * (r - c.pd_near_m) / (c.pd_far_m - c.pd_near_m)
        return np.where(r <= c.pd_near_m, c.pd_near, np.where(r <= c.pd_far_m, far, 0.0))

    def visible(self, x, y, eye, yaw: float, cams, obstacle_map, mode=None,
                unknown_credit: float = 0.0) -> np.ndarray:
        """Chance (0-1) that a person at each (x, y) would be detected from
        `eye` (drone x, y, z; compass yaw) by any of `cams` (CameraGeometry),
        given the map. 0 out of view or behind something. A line through
        unmapped space counts unknown_credit x P_d: 0 when deciding what was
        seen empty (never 'clear' a place we haven't mapped), more when
        planning where to look (it may well be clear)."""
        c = self.cfg
        x, y = np.asarray(x, float), np.asarray(y, float)
        dx, dy = x - eye[0], y - eye[1]
        r = np.hypot(dx, dy)
        bearing = np.arctan2(dx, dy) - yaw
        dep = np.arctan2(eye[2] - c.chest_h, np.maximum(r, 1e-3))      # down = +
        inview = np.zeros(len(x), bool)
        for cam in cams:
            half = math.radians(cam.hfov_deg / 2 - c.fov_margin_deg)
            vhalf = math.atan(math.tan(math.radians(cam.hfov_deg) / 2) * 0.75) - math.radians(c.fov_margin_deg)
            rel = (bearing - math.radians(cam.yaw_deg) + math.pi) % (2 * math.pi) - math.pi
            pitch = math.radians(cam.pitch_deg)
            inview |= (np.abs(rel) < half) & (dep > pitch - vhalf) & (dep < pitch + vhalf)
        inview &= (r >= c.min_range)
        if mode is not None:
            inview &= mode != INSIDE
        out = np.zeros(len(x))
        idx = np.nonzero(inview)[0]
        if len(idx) == 0:
            return out
        pd = self.pd(r[idx])
        if obstacle_map is not None:
            pts = np.stack([x[idx], y[idx], np.full(len(idx), c.chest_h)], -1)
            blocked, unknown = obstacle_map.lines_blocked(eye, pts)
            pd = np.where(blocked, 0.0, np.where(unknown > c.unknown_max, unknown_credit * pd, pd))
        out[idx] = pd
        return out

    def observe_empty(self, eye, yaw: float, cams, obstacle_map, others=(), dt: float = 0.05) -> None:
        """No matching person detected this frame: lower the weight of every
        particle the cameras could have seen. `others`: ground (x, y) of the
        people that WERE detected (not the target); the target could be
        standing right behind one of them, so those spots are not 'seen empty'."""
        if not self.active:
            return
        c = self.cfg
        q = self.visible(self.x, self.y, eye, yaw, cams, obstacle_map, self.mode)
        for ox, oy in others:
            q[np.hypot(self.x - ox, self.y - oy) < c.near_person_m] = 0.0
        if not q.any():
            return
        q = 1.0 - (1.0 - q) ** (dt / c.look_s)               # per-frame detection chance
        w = self.w * (1.0 - q)
        s = w.sum()
        if s <= 1e-300:
            return                                         # everything seen empty: keep the old belief
        self.w = w / s
        self._clusters = None
        if self.ess() < c.resample_ess * c.n:
            self.resample()

    # --- resampling ---------------------------------------------------------------
    def ess(self) -> float:
        return float(1.0 / np.sum(self.w ** 2))

    def resample(self) -> None:
        c, rng, n = self.cfg, self.rng, self.cfg.n
        pos = (rng.random() + np.arange(n)) / n
        idx = np.minimum(np.searchsorted(np.cumsum(self.w), pos), n - 1)
        for a in ("x", "y", "v", "h", "vpref", "turn", "mode"):
            setattr(self, a, getattr(self, a)[idx].copy())
        self.x += rng.normal(0, c.jitter_m, n)
        self.y += rng.normal(0, c.jitter_m, n)
        # A few "births": the person could be somewhere near what we believe
        # but where no particle went. Keeps the cloud from collapsing on a wrong guess.
        k = int(c.birth_frac * n)
        if k:
            b = rng.choice(n, k, replace=False)
            self.x[b] += rng.normal(0, c.birth_sd_m, k)
            self.y[b] += rng.normal(0, c.birth_sd_m, k)
            self.mode[b] = np.where(rng.random(k) < 0.5, STOP, WALK)
            self.h[b] = rng.uniform(-math.pi, math.pi, k)
            self.v[b] = 0.0
            bad = self._blocked(self.x[b], self.y[b])
            self.mode[b[bad]] = INSIDE
        self.w = np.full(n, 1.0 / n)
        self._clusters = None

    # --- queries ----------------------------------------------------------------------
    def density(self, x: float, y: float, r: float = 1.5) -> float:
        """Share of the belief within r metres of (x, y)."""
        if not self.active:
            return 0.0
        return float(self.w[np.hypot(self.x - x, self.y - y) < r].sum())

    def clusters(self, k: int = 3) -> list[Cluster]:
        """The k heaviest groups of particles (not the mean: a cloud split
        between 'stopped behind the pillar' and 'walked on' has its mean where
        nobody is)."""
        if not self.active:
            return []
        if self._clusters is not None and len(self._clusters) >= k:
            return self._clusters[:k]
        c = self.cfg
        cell = c.cluster_cell_m
        g = np.stack([np.floor(self.x / cell), np.floor(self.y / cell)], -1).astype(np.int64)
        left = np.ones(len(self.x), bool)
        out = []
        for _ in range(k):
            if not left.any():
                break
            uk, inv = np.unique(g[left], axis=0, return_inverse=True)
            mass = np.bincount(inv.reshape(-1), weights=self.w[left])
            bx, by = uk[int(np.argmax(mass))]
            cx, cy = (bx + 0.5) * cell, (by + 0.5) * cell
            sel = left & (np.hypot(self.x - cx, self.y - cy) < c.cluster_radius_m)
            ws = self.w[sel]
            if ws.sum() <= 0:
                break
            mx = float(np.average(self.x[sel], weights=ws))
            my = float(np.average(self.y[sel], weights=ws))
            sel = left & (np.hypot(self.x - mx, self.y - my) < c.cluster_radius_m)
            ws = self.w[sel]
            if ws.sum() <= 0:
                break
            mx = float(np.average(self.x[sel], weights=ws))
            my = float(np.average(self.y[sel], weights=ws))
            spread = float(np.sqrt(np.average((self.x[sel] - mx) ** 2 + (self.y[sel] - my) ** 2, weights=ws)))
            modes = np.bincount(self.mode[sel], weights=ws, minlength=4)
            out.append(Cluster(mx, my, float(ws.sum()), spread, MODE_NAMES[int(np.argmax(modes))]))
            left &= ~sel
        self._clusters = out
        return out

    def sample(self, m: int, rng: Optional[np.random.Generator] = None):
        """m particles drawn by weight: (x, y, mode), for display and scoring."""
        rng = rng or self.rng
        idx = rng.choice(len(self.w), size=min(m, len(self.w)), p=self.w)
        return self.x[idx], self.y[idx], self.mode[idx]

    def mode_shares(self) -> dict:
        s = np.bincount(self.mode, weights=self.w, minlength=4)
        return {MODE_NAMES[i]: round(float(s[i]), 3) for i in range(4)}
