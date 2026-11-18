"""3-D obstacle map around the drone from the ultrasonic (sonar) array and,
optionally, the cameras' depth images.

Why 3-D: branches, overhangs and wires sit *above* or *below* the drone's
flight level, and the right dodge is often to climb or drop, not to sidestep.
The 2-D SurroundMap can't tell a branch at head height from one at 5 m.

How sonar becomes a map
-----------------------
A sonar beam is a cone (about 30 deg wide). An echo at distance r means
"something is on the cone's cap at r, and nothing is closer inside the cone".
So each reading:
  * marks every cell on the cap at r a little more likely occupied, and
  * marks every cell inside the cone closer than r more likely free.
One reading smears a branch over a 1-2 m patch. As the drone moves, caps
from different positions overlap only where the branch really is, and the
free-space updates erase the rest (log-odds accumulation). No echo clears the
whole cone, which is also how a missed thin twig quietly disappears from the
map, as it does with real sonar.

Depth points (from Unity or Depth Anything) go into the same grid as
single-ray hits with free space along the ray.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from drone_core.domain.models import MultiFrame, SonarScan, Telemetry
from drone_core.usecases.surround_map import camera_rays


@dataclass
class VoxelMapConfig:
    size_xy: float = 30.0          # m, cube side around the drone (re-centred as it flies)
    z_min: float = -1.0            # m, world height range kept
    z_max: float = 15.0
    resolution: float = 0.2        # m per voxel
    ground_h: float = 0.3          # echoes/points below this height are ground, not obstacles
    hit: float = 0.55              # log-odds per hit (two agreeing readings -> occupied)
    miss: float = 0.3              # log-odds per pass-through
    clamp: float = 3.5
    occupied_at: float = 0.8
    cone_rings: tuple = ((0.33, 6), (0.67, 10), (1.0, 14))   # (fraction of half-angle, points)
    depth_max_range: float = 15.0
    depth_free_samples: int = 10
    use_depth: bool = True
    depth_every: int = 3           # add depth images every n-th update (sonar goes in every time)
    drone_radius: float = 0.35     # m, corridor half-width for clearance queries


# Directions reported by ranges(): (yaw offset deg, pitch deg) in the body frame.
RANGE_DIRS = {
    "front": (0.0, 0.0), "front_left": (-45.0, 0.0), "front_right": (45.0, 0.0),
    "left": (-90.0, 0.0), "right": (90.0, 0.0), "back": (180.0, 0.0),
    "front_up": (0.0, 35.0), "front_down": (0.0, -35.0),
    "up": (0.0, 90.0), "below": (0.0, -90.0),
}


def _body_dir(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    a, p = math.radians(yaw_deg), math.radians(pitch_deg)
    return np.array([math.cos(p) * math.cos(a), math.cos(p) * math.sin(a), math.sin(p)])


def cone_dirs(yaw_deg: float, pitch_deg: float, fov_deg: float, rings) -> np.ndarray:
    """Unit vectors (forward, right, up) sampling a cone, shape (n, 3)."""
    a, p = math.radians(yaw_deg), math.radians(pitch_deg)
    d = _body_dir(yaw_deg, pitch_deg)
    u1 = np.array([-math.sin(a), math.cos(a), 0.0])
    u2 = np.array([-math.sin(p) * math.cos(a), -math.sin(p) * math.sin(a), math.cos(p)])
    half = math.radians(fov_deg) / 2
    out = [d]
    for frac, n in rings:
        th = half * frac
        for k in range(n):
            phi = 2 * math.pi * k / n
            out.append(math.cos(th) * d + math.sin(th) * (math.cos(phi) * u1 + math.sin(phi) * u2))
    return np.array(out)


def body_to_world(v: np.ndarray, yaw: float) -> np.ndarray:
    """(…, 3) body (forward, right, up) -> world (x east, y north, z up). yaw = compass heading."""
    s, c = math.sin(yaw), math.cos(yaw)
    x = v[..., 0] * s + v[..., 1] * c
    y = v[..., 0] * c - v[..., 1] * s
    return np.stack([x, y, v[..., 2]], -1)


class VoxelMap:
    def __init__(self, rig: Optional[dict] = None, cfg: VoxelMapConfig | None = None):
        self.rig = rig or {}
        self.cfg = cfg or VoxelMapConfig()
        c = self.cfg
        self.nxy = int(round(c.size_xy / c.resolution))
        self.nz = int(round((c.z_max - c.z_min) / c.resolution))
        self.logodds = np.zeros((self.nz, self.nxy, self.nxy), np.float32)   # [z, y, x]
        self.origin: Optional[np.ndarray] = None      # world (x, y) of voxel column (0, 0)
        self._cones: dict = {}
        self._updates = 0

    # --- indexing -----------------------------------------------------------
    def _recentre(self, x: float, y: float) -> None:
        c, half, n = self.cfg, self.cfg.size_xy / 2, self.nxy
        if self.origin is None:
            self.origin = np.array([x - half, y - half])
            return
        cx, cy = (x - self.origin[0]) / c.resolution, (y - self.origin[1]) / c.resolution
        if n * 0.25 < cx < n * 0.75 and n * 0.25 < cy < n * 0.75:
            return
        shift = np.round((np.array([x - half, y - half]) - self.origin) / c.resolution).astype(int)
        sx, sy = int(shift[0]), int(shift[1])
        grid = np.zeros_like(self.logodds)
        if abs(sx) < n and abs(sy) < n:     # a jump bigger than the map (reset, teleport) = start empty
            src = self.logodds[:, max(0, sy):n + min(0, sy), max(0, sx):n + min(0, sx)]
            grid[:, max(0, -sy):max(0, -sy) + src.shape[1], max(0, -sx):max(0, -sx) + src.shape[2]] = src
        self.logodds = grid
        self.origin = self.origin + np.array([sx, sy]) * c.resolution

    def _flat(self, pts: np.ndarray) -> np.ndarray:
        """World points (n, 3) -> unique flat voxel indices inside the map."""
        c = self.cfg
        ix = np.floor((pts[:, 0] - self.origin[0]) / c.resolution).astype(int)
        iy = np.floor((pts[:, 1] - self.origin[1]) / c.resolution).astype(int)
        iz = np.floor((pts[:, 2] - c.z_min) / c.resolution).astype(int)
        ok = (ix >= 0) & (ix < self.nxy) & (iy >= 0) & (iy < self.nxy) & (iz >= 0) & (iz < self.nz)
        return np.unique((iz[ok] * self.nxy + iy[ok]) * self.nxy + ix[ok])

    def _apply(self, hits: list, frees: list) -> None:
        c = self.cfg
        flat = self.logodds.reshape(-1)
        h = np.unique(np.concatenate(hits)) if hits else np.zeros(0, int)
        f = np.unique(np.concatenate(frees)) if frees else np.zeros(0, int)
        if len(h) and len(f):
            f = np.setdiff1d(f, h, assume_unique=True)
        # only touch the voxels this update saw (clipping the whole grid is slow)
        flat[f] = np.maximum(flat[f] - c.miss, -c.clamp)
        flat[h] = np.minimum(flat[h] + c.hit, c.clamp)

    # --- updates ------------------------------------------------------------
    def update(self, telem: Telemetry, multi: Optional[MultiFrame] = None, ignore_xy=None,
               ignore_radius: float = 0.9) -> None:
        """Add the latest sonar scan (telem.sonar) and, if enabled, depth images.
        ignore_xy: the followed person's ground position; returns there below
        head height are the person, not an obstacle."""
        if telem.sonar is None and (multi is None or not multi.depth):
            return
        self._recentre(telem.position[0], telem.position[1])
        hits, frees = [], []
        if telem.sonar is not None:
            self._sonar(telem, telem.sonar, hits, frees, ignore_xy, ignore_radius)
        self._updates += 1
        if (self.cfg.use_depth and multi is not None and multi.depth
                and self._updates % max(1, self.cfg.depth_every) == 0):
            self._depth(telem, multi, hits, frees, ignore_xy, ignore_radius)
        self._apply(hits, frees)

    def _origin3(self, telem: Telemetry) -> np.ndarray:
        # height above ground; telemetry.altitude may come from a rangefinder
        return np.array([telem.position[0], telem.position[1], telem.position[2]])

    def _sonar(self, telem, scan: SonarScan, hits, frees, ignore_xy, ignore_radius) -> None:
        c = self.cfg
        o = self._origin3(telem)
        for b in scan.beams:
            key = (b.yaw_deg, b.pitch_deg, scan.fov_deg)
            if key not in self._cones:
                self._cones[key] = cone_dirs(b.yaw_deg, b.pitch_deg, scan.fov_deg, c.cone_rings)
            dirs = body_to_world(self._cones[key], telem.yaw)          # (n, 3)
            r = b.range
            free_to = (r - 1.5 * c.resolution) if r is not None else scan.max_range * 0.9
            if free_to > scan.min_range:
                steps = np.arange(scan.min_range, free_to, c.resolution * 0.8)
                pts = o + dirs[None, :, :] * steps[:, None, None]
                frees.append(self._flat(pts.reshape(-1, 3)))
            if r is not None and self._is_ground_echo(r, b.pitch_deg, scan.fov_deg, telem.altitude):
                r = None        # the cone's lower edge reaches the ground first: that's the echo
            if r is not None:
                pts = o + dirs * r
                keep = pts[:, 2] > c.ground_h
                if ignore_xy is not None:
                    keep &= (np.hypot(pts[:, 0] - ignore_xy[0], pts[:, 1] - ignore_xy[1]) > ignore_radius) | (pts[:, 2] > 2.2)
                if keep.any():
                    hits.append(self._flat(pts[keep]))

    @staticmethod
    def _is_ground_echo(r: float, pitch_deg: float, fov_deg: float, alt: float) -> bool:
        """A down-tilted cone always gets an echo from the ground at about
        alt / sin(steepest angle). Treating that as an obstacle would paint a
        ring of phantom obstacles around the drone."""
        steep = math.radians(-(pitch_deg - fov_deg / 2))
        if steep <= math.radians(1.0):
            return False
        return r >= alt / math.sin(min(steep, math.pi / 2)) - 0.2

    def _depth(self, telem, multi, hits, frees, ignore_xy, ignore_radius) -> None:
        c = self.cfg
        o = self._origin3(telem)
        for name, depth in multi.depth.items():
            cam = self.rig.get(name)
            if cam is None:
                continue
            rays = camera_rays(cam, *depth.shape)
            ok = np.isfinite(depth) & (depth > 0.2) & (depth < c.depth_max_range)
            if not ok.any():
                continue
            d = depth[ok][:, None]
            pts = o + body_to_world(rays[ok], telem.yaw) * d
            keep = pts[:, 2] > c.ground_h
            if ignore_xy is not None:
                keep &= (np.hypot(pts[:, 0] - ignore_xy[0], pts[:, 1] - ignore_xy[1]) > ignore_radius) | (pts[:, 2] > 2.2)
            hits.append(self._flat(pts[keep]))
            fr = np.linspace(0.0, 1.0, c.depth_free_samples, endpoint=False)[1:]
            gap = 1.0 - c.resolution * 1.5 / np.maximum(d[:, 0], 1e-3)
            seg = o + (pts - o)[:, None, :] * fr[None, :, None]
            frees.append(self._flat(seg[fr[None, :] < gap[:, None]]))

    # --- queries ------------------------------------------------------------
    def occupied(self) -> np.ndarray:
        return self.logodds > self.cfg.occupied_at

    def occupied_points(self) -> np.ndarray:
        """World (x, y, z) centres of occupied voxels, (n, 3)."""
        if self.origin is None:
            return np.zeros((0, 3))
        iz, iy, ix = np.nonzero(self.occupied())
        r = self.cfg.resolution
        return np.stack([self.origin[0] + (ix + 0.5) * r, self.origin[1] + (iy + 0.5) * r,
                         self.cfg.z_min + (iz + 0.5) * r], -1)

    def clearance(self, start: np.ndarray, direction: np.ndarray, max_range: float = 10.0,
                  radius: Optional[float] = None) -> Optional[float]:
        """Distance along a world direction to the first occupied voxel, checking a
        tube of the drone's radius. None = clear within max_range."""
        if self.origin is None:
            return None
        c = self.cfg
        radius = c.drone_radius if radius is None else radius
        d = direction / np.linalg.norm(direction)
        u1 = np.cross(d, [0.0, 0.0, 1.0])
        if np.linalg.norm(u1) < 1e-6:
            u1 = np.array([1.0, 0.0, 0.0])
        u1 /= np.linalg.norm(u1)
        u2 = np.cross(d, u1)
        offs = np.array([[0, 0, 0], u1 * radius, -u1 * radius, u2 * radius, -u2 * radius])
        s = np.arange(c.resolution * 0.5, max_range, c.resolution * 0.5)
        pts = start + d * s[:, None, None] + offs[None, :, :]          # (steps, 5, 3)
        ix = np.floor((pts[..., 0] - self.origin[0]) / c.resolution).astype(int)
        iy = np.floor((pts[..., 1] - self.origin[1]) / c.resolution).astype(int)
        iz = np.floor((pts[..., 2] - c.z_min) / c.resolution).astype(int)
        inside = (ix >= 0) & (ix < self.nxy) & (iy >= 0) & (iy < self.nxy) & (iz >= 0) & (iz < self.nz)
        occ = np.zeros(pts.shape[:2], bool)
        occ[inside] = self.logodds[iz[inside], iy[inside], ix[inside]] > c.occupied_at
        hit_steps = np.nonzero(occ.any(axis=1))[0]
        return float(s[hit_steps[0]]) if len(hit_steps) else None

    def line_blocked(self, a, b, skip_start: float = 0.6, skip_end: float = 0.8) -> bool:
        """True if an occupied voxel lies on the straight line a -> b (world x, y, z).
        Used as a line-of-sight test from the drone to the person's head; the
        first/last bit of the line is skipped (the drone itself, the person)."""
        if self.origin is None:
            return False
        a, b = np.asarray(a, float), np.asarray(b, float)
        L = float(np.linalg.norm(b - a))
        if L <= skip_start + skip_end:
            return False
        s = np.arange(skip_start, L - skip_end, self.cfg.resolution * 0.5)
        pts = a + (b - a)[None, :] * (s / L)[:, None]
        c = self.cfg
        ix = np.floor((pts[:, 0] - self.origin[0]) / c.resolution).astype(int)
        iy = np.floor((pts[:, 1] - self.origin[1]) / c.resolution).astype(int)
        iz = np.floor((pts[:, 2] - c.z_min) / c.resolution).astype(int)
        ok = (ix >= 0) & (ix < self.nxy) & (iy >= 0) & (iy < self.nxy) & (iz >= 0) & (iz < self.nz)
        return bool((self.logodds[iz[ok], iy[ok], ix[ok]] > c.occupied_at).any())

    def ranges(self, telem: Telemetry, max_range: float = 10.0) -> dict:
        """Clearances in the range-sensor key style, plus front_up / front_down /
        up / below for vertical dodging. Missing key = clear."""
        o = self._origin3(telem)
        out = {}
        for k, (yaw_off, pitch) in RANGE_DIRS.items():
            d = body_to_world(_body_dir(yaw_off, pitch), telem.yaw)
            r = self.clearance(o, d, max_range)
            if r is not None:
                out[k] = r
        return out
