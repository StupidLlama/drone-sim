"""Top-down obstacle map around the drone, built from the depth images of
every camera on the rig.

Each depth pixel is turned into a 3-D point. Points that stand up off the
ground mark their grid cell as occupied; the cells the ray passed through on
the way are marked free. Log-odds accumulation makes single bad depth
pixels (common with monocular depth networks) fade out.

The map can then answer "how far is the nearest obstacle in direction X",
which is exactly what AvoidanceFilter consumes, so the real drone can avoid
walls with cameras only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from drone_core.domain.models import MultiFrame, Telemetry
from drone_core.usecases.target_geo import CameraGeometry


@dataclass
class MapConfig:
    size_m: float = 60.0          # map is size_m x size_m, re-centred on the drone as it travels
    resolution: float = 0.25      # metres per cell
    max_range: float = 15.0       # ignore depth beyond this (noisy, and far walls don't matter yet)
    obstacle_min_h: float = 0.4   # point this high above ground = obstacle (lower = ground)
    obstacle_max_above: float = 2.0  # ignore points this far above the drone (tree tops, roofs)
    hit: float = 0.9              # log-odds added for an obstacle hit
    miss: float = 0.35            # log-odds removed for a cell seen through
    clamp: float = 4.0
    occupied_at: float = 0.8      # log-odds above this counts as occupied
    free_samples: int = 12        # points sampled along each ray to clear free space
    pixel_stride: int = 1         # use every n-th depth pixel


_RAYS: dict = {}


def camera_rays(cam: CameraGeometry, h: int, w: int) -> np.ndarray:
    """Unit ray for each pixel in BODY axes (forward, right, up), shape (h, w, 3)."""
    key = (cam.hfov_deg, cam.pitch_deg, cam.yaw_deg, h, w)
    if key in _RAYS:
        return _RAYS[key]
    f = (w / 2) / math.tan(math.radians(cam.hfov_deg) / 2)
    u = (np.arange(w) + 0.5 - w / 2) / f
    v = (np.arange(h) + 0.5 - h / 2) / f
    uu, vv = np.meshgrid(u, v)
    th = math.radians(cam.pitch_deg)
    fwd = math.cos(th) - vv * math.sin(th)
    up = -(vv * math.cos(th) + math.sin(th))
    right = uu
    r = np.stack([fwd, right, up], -1)
    r /= np.linalg.norm(r, axis=-1, keepdims=True)
    # rotate around the vertical axis by the mount yaw
    a = math.radians(cam.yaw_deg)
    f2 = r[..., 0] * math.cos(a) - r[..., 1] * math.sin(a)
    r2 = r[..., 0] * math.sin(a) + r[..., 1] * math.cos(a)
    out = np.stack([f2, r2, r[..., 2]], -1).astype(np.float32)
    _RAYS[key] = out
    return out


class SurroundMap:
    def __init__(self, rig: dict, cfg: MapConfig | None = None):
        self.rig: dict[str, CameraGeometry] = rig
        self.cfg = cfg or MapConfig()
        n = int(round(self.cfg.size_m / self.cfg.resolution))
        self.n = n
        self.logodds = np.zeros((n, n), np.float32)   # [row=y, col=x]
        self.origin = None                            # world (x, y) of cell (0, 0)

    # --- geometry ------------------------------------------------------------
    def _unit_rays(self, cam: CameraGeometry, h: int, w: int) -> np.ndarray:
        return camera_rays(cam, h, w)

    def _recentre(self, x: float, y: float) -> None:
        c, half = self.cfg, self.cfg.size_m / 2
        if self.origin is None:
            self.origin = np.array([x - half, y - half])
            return
        # shift the grid when the drone is within a quarter of the edge
        cx, cy = (x - self.origin[0]) / c.resolution, (y - self.origin[1]) / c.resolution
        if self.n * 0.25 < cx < self.n * 0.75 and self.n * 0.25 < cy < self.n * 0.75:
            return
        new_origin = np.array([x - half, y - half])
        shift = np.round((new_origin - self.origin) / c.resolution).astype(int)
        grid = np.zeros_like(self.logodds)
        sx, sy = int(shift[0]), int(shift[1])
        if abs(sx) < self.n and abs(sy) < self.n:   # bigger jump (reset, teleport) = start empty
            src = self.logodds[max(0, sy):self.n + min(0, sy), max(0, sx):self.n + min(0, sx)]
            grid[max(0, -sy):max(0, -sy) + src.shape[0], max(0, -sx):max(0, -sx) + src.shape[1]] = src
        self.logodds = grid
        self.origin = self.origin + shift * c.resolution

    def _cells(self, wx: np.ndarray, wy: np.ndarray):
        col = ((wx - self.origin[0]) / self.cfg.resolution).astype(int)
        row = ((wy - self.origin[1]) / self.cfg.resolution).astype(int)
        ok = (col >= 0) & (col < self.n) & (row >= 0) & (row < self.n)
        return row[ok], col[ok]

    # --- update --------------------------------------------------------------
    def update(self, multi: MultiFrame, telem: Telemetry, ignore_xy=None, ignore_radius: float = 0.9) -> None:
        """ignore_xy: world (x, y) of the person being followed, so they don't
        end up in the map as a wall the drone has to avoid."""
        if not multi.depth:
            return
        c = self.cfg
        px, py = telem.position[0], telem.position[1]
        alt = telem.altitude
        self._recentre(px, py)
        cy, sy = math.cos(telem.yaw), math.sin(telem.yaw)
        hits_r, hits_c, free_r, free_c = [], [], [], []
        for name, depth in multi.depth.items():
            cam = self.rig.get(name)
            if cam is None:
                continue
            d = depth[::c.pixel_stride, ::c.pixel_stride]
            rays = self._unit_rays(cam, *d.shape)
            ok = np.isfinite(d) & (d > 0.2) & (d < c.max_range)
            if not ok.any():
                continue
            dist = d[ok][:, None]
            r = rays[ok]
            body = r * dist                       # forward, right, up (m)
            # body -> world (x east, y north); yaw is a compass heading
            wx = px + body[:, 0] * sy + body[:, 1] * cy
            wy = py + body[:, 0] * cy - body[:, 1] * sy
            h = alt + body[:, 2]                  # height above ground
            obstacle = (h > c.obstacle_min_h) & (h < alt + c.obstacle_max_above)
            if ignore_xy is not None:
                obstacle &= np.hypot(wx - ignore_xy[0], wy - ignore_xy[1]) > ignore_radius
            rr, cc = self._cells(wx[obstacle], wy[obstacle])
            hits_r.append(rr); hits_c.append(cc)
            # free space: sample along the horizontal projection of each ray,
            # stopping a cell short of the end point
            fr = np.linspace(0.0, 1.0, c.free_samples, endpoint=False)[1:]
            hx = px + (wx[:, None] - px) * fr[None, :]
            hy = py + (wy[:, None] - py) * fr[None, :]
            end_gap = c.resolution * 1.5 / np.maximum(np.hypot(wx - px, wy - py), 1e-3)
            keep = fr[None, :] < (1.0 - end_gap[:, None])
            rr, cc = self._cells(hx[keep], hy[keep])
            free_r.append(rr); free_c.append(cc)
        if free_r:
            # bincount instead of np.add.at: same counts, ~10x faster
            free = np.bincount(np.concatenate(free_r) * self.n + np.concatenate(free_c),
                               minlength=self.n * self.n).reshape(self.n, self.n).astype(np.float32)
            self.logodds -= c.miss * np.minimum(free, 3.0)
        if hits_r:
            hit = np.bincount(np.concatenate(hits_r) * self.n + np.concatenate(hits_c),
                              minlength=self.n * self.n).reshape(self.n, self.n).astype(np.float32)
            self.logodds += c.hit * np.minimum(hit, 3.0)
        np.clip(self.logodds, -c.clamp, c.clamp, out=self.logodds)

    # --- queries -------------------------------------------------------------
    def occupied(self) -> np.ndarray:
        return self.logodds > self.cfg.occupied_at

    def range_in_direction(self, x: float, y: float, heading: float, max_range: float = 20.0,
                           half_width: float = 0.3) -> Optional[float]:
        """Distance to the first occupied cell along a compass heading, checking a
        corridor about the drone's width. None = clear within max_range."""
        if self.origin is None:
            return None
        occ = self.occupied()
        step = self.cfg.resolution * 0.5
        dx, dy = math.sin(heading), math.cos(heading)
        nx, ny = dy, -dx
        for k in range(1, int(max_range / step)):
            s = k * step
            for o in (-half_width, 0.0, half_width):
                cx = x + dx * s + nx * o
                cy = y + dy * s + ny * o
                col = int((cx - self.origin[0]) / self.cfg.resolution)
                row = int((cy - self.origin[1]) / self.cfg.resolution)
                if 0 <= col < self.n and 0 <= row < self.n and occ[row, col]:
                    return s
        return None

    def ranges(self, telem: Telemetry, max_range: float = 20.0) -> dict:
        """Same keys as range sensors: front, back, left, right, front_left, front_right."""
        dirs = {"front": 0.0, "front_right": 45.0, "right": 90.0, "back": 180.0,
                "left": -90.0, "front_left": -45.0}
        out = {}
        for k, a in dirs.items():
            r = self.range_in_direction(telem.position[0], telem.position[1],
                                        telem.yaw + math.radians(a), max_range)
            if r is not None:
                out[k] = r
        return out
