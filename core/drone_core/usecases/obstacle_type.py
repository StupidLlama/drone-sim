"""What did the target disappear behind? (v3, Evan's idea A)

When the target is lost, look at the 3-D map round where it was last seen and
decide which of three scenes it is, with a certainty:

    WALL   an obstacle on one side, open on the other: they went round its end
           or are walking along its hidden side.
    HALL   two walls facing each other with open space down the middle: they
           went into a corridor.
    ROOM   they walked straight into something thick (a shop, a building):
           they are inside and will come out of a door.

How: 36 rays are cast on the map's person-height layer from the last-seen spot
(and, for a hall, from a few spots round it, because people are usually lost
at the hall's mouth). Each scene gets a score from what the rays hit:

    room  the ray along their heading hits within 1.5 m, and what it hits is
          thick (not seen free behind the face), helped by the belief's INSIDE share
    hall  opposite rays both hit within 3.5 m (two walls) and rays down the
          gap are open for 6 m+
    wall  some rays hit within 3.5 m, no opposite pairs, the obstacle is long
          (a round pillar is left to the v2 search)

probability = score / (sum of scores + 0.1 "none of these"), and the
certainty is that times how much of the area the map has actually seen. The
drone only commits to a scene's scan route at 80% (FollowTarget), and keeps
re-classifying while it scans, because the map fills in as it flies.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

WALL, HALL, ROOM = "wall", "hall", "room"
KINDS = (WALL, HALL, ROOM)


@dataclass
class SceneConfig:
    rays: int = 36
    reach: float = 8.0          # m, ray length
    step: float = 0.1           # m between samples along a ray
    near: float = 3.5           # m: a hit closer than this counts as "beside them"
    open_m: float = 6.0         # m: a ray this long without a hit is "open"
    into_m: float = 1.5         # m: a hit this close along the heading = walked into it
    thick_m: float = 1.0        # m behind the face that must not be seen free (a building, not a wall)
    hall_probe_m: float = 2.5   # also test for a hall this far round the last-seen spot
    geom_r: float = 7.0         # m: obstacle cells this close shape the wall geometry
    elongated: float = 2.5      # wall: long axis / short axis at least this (else: pillar)
    unsure: float = 0.1         # "none of these" mass
    commit: float = 0.8         # certainty to commit to a scan route
    every_s: float = 0.5        # re-classify this often while lost


@dataclass
class Scene:
    kind: Optional[str]                    # wall | hall | room | None
    cert: float                            # 0..1
    probs: dict = field(default_factory=dict)
    lost: tuple = (0.0, 0.0)               # last-seen ground point
    heading: tuple = (0.0, 1.0)            # unit, their direction of travel when lost
    # wall: the end nearest the last-seen spot, direction out past that end,
    # unit normal to the side they are probably on, length, top height, low enough to see over
    end: Optional[tuple] = None
    out: Optional[tuple] = None
    hidden: Optional[tuple] = None
    length: float = 0.0
    top: Optional[float] = None
    low: bool = False
    # hall: centre-line point at its mouth, unit axis into it, width, open length
    centre: Optional[tuple] = None
    axis: Optional[tuple] = None
    width: float = 0.0
    depth: float = 0.0
    # room: bounding box of the building (x0, y0, x1, y1) and the door (last seen)
    box: Optional[tuple] = None
    door: Optional[tuple] = None


def _unit(v):
    n = math.hypot(v[0], v[1])
    return (v[0] / n, v[1] / n) if n > 1e-6 else (0.0, 1.0)


class ObstacleClassifier:
    def __init__(self, cfg: SceneConfig | None = None):
        self.cfg = cfg or SceneConfig()
        n = self.cfg.rays
        a = 2 * math.pi * np.arange(n) / n
        self.dirs = np.stack([np.sin(a), np.cos(a)], -1)          # (n, 2), yaw convention: x = sin, y = cos
        self.s = np.arange(self.cfg.step, self.cfg.reach + 1e-9, self.cfg.step)

    # --- rays ------------------------------------------------------------------
    def _grids(self, vmap):
        om = vmap.obstacle_mask() if vmap is not None else None
        if om is None:
            return None
        occ, org, res = om
        known = vmap.known_mask() if hasattr(vmap, "known_mask") else np.ones_like(occ)
        return occ, known, org, res

    def _cast(self, g, p):
        """Rays from p: (hit distance, inf = none; fraction of the ray before
        the hit that the map has never seen)."""
        occ, known, org, res = g
        q = np.asarray(p, float)[None, None, :] + self.dirs[:, None, :] * self.s[None, :, None]
        ix = np.floor((q[..., 0] - org[0]) / res).astype(int)
        iy = np.floor((q[..., 1] - org[1]) / res).astype(int)
        ok = (ix >= 0) & (ix < occ.shape[1]) & (iy >= 0) & (iy < occ.shape[0])
        o = np.zeros(ix.shape, bool)
        k = np.zeros(ix.shape, bool)
        o[ok] = occ[iy[ok], ix[ok]]
        k[ok] = known[iy[ok], ix[ok]]
        has = o.any(axis=1)
        first = np.where(has, o.argmax(axis=1), len(self.s))
        d = np.where(has, self.s[np.minimum(first, len(self.s) - 1)], np.inf)
        before = np.arange(len(self.s))[None, :] < first[:, None]
        unk = ((~k) & before).sum(axis=1) / np.maximum(before.sum(axis=1), 1)
        return d, unk

    def _is_free(self, g, p) -> Optional[bool]:
        occ, known, org, res = g
        ix, iy = int(math.floor((p[0] - org[0]) / res)), int(math.floor((p[1] - org[1]) / res))
        if not (0 <= ix < occ.shape[1] and 0 <= iy < occ.shape[0]):
            return None
        if occ[iy, ix]:
            return False
        return True if known[iy, ix] else None

    # --- scores ----------------------------------------------------------------
    def _hall(self, g, p, heading=(0.0, 1.0)):
        """(score, centre, axis, width, open length) of the best hall at p."""
        c = self.cfg
        d, unk = self._cast(g, p)
        n = c.rays
        h = n // 2
        best = (0.0, None, None, 0.0, 0.0)
        pairs = (d[:h] < c.near) & (d[h:] < c.near)
        cnt = int(pairs.sum())
        if cnt == 0:
            return best
        # the gap runs at right angles to the paired rays
        ang = 2 * math.pi * np.arange(h)[pairs] / n
        mean2 = math.atan2(np.sin(2 * ang).mean(), np.cos(2 * ang).mean()) / 2   # mean direction mod pi
        ax = (math.sin(mean2 + math.pi / 2), math.cos(mean2 + math.pi / 2))
        # open length both ways along the axis
        i_ax = int(round((math.atan2(ax[0], ax[1]) % (2 * math.pi)) / (2 * math.pi) * n)) % n
        j_ax = (i_ax + h) % n
        lens = [float(np.min(d[[(i + k) % n for k in (-1, 0, 1)]])) for i in (i_ax, j_ax)]
        seen = [float(np.mean(unk[[(i + k) % n for k in (-1, 0, 1)]])) < 0.6 for i in (i_ax, j_ax)]
        open_way = [L >= c.open_m and s for L, s in zip(lens, seen)]
        if not any(open_way):
            return best
        # centre: half way between the two walls of the pair closest to the normal
        i_n = int(round((mean2 % math.pi) / (2 * math.pi) * n)) % h
        d1, d2 = float(d[i_n]), float(d[i_n + h])
        if not (math.isfinite(d1) and math.isfinite(d2)):
            return best
        nrm = self.dirs[i_n]
        centre = (p[0] + nrm[0] * (d1 - d2) / 2, p[1] + nrm[1] * (d1 - d2) / 2)
        width = d1 + d2
        if width > 2 * c.near:
            return best
        score = min(1.0, cnt / 4.0)
        # into the hall = the open way closest to where they were heading
        al = ax[0] * heading[0] + ax[1] * heading[1]
        k = 0 if (al >= 0 and open_way[0]) or not open_way[1] else 1
        axis = ax if k == 0 else (-ax[0], -ax[1])
        return score, centre, axis, width, self._ray_len(g, centre, axis, 20.0)

    def _ray_len(self, g, p, u, max_m: float) -> float:
        """Free length from p along unit u (unknown counts as free), up to max_m."""
        occ, known, org, res = g
        s = np.arange(self.cfg.step, max_m + 1e-9, self.cfg.step)
        ix = np.floor((p[0] + u[0] * s - org[0]) / res).astype(int)
        iy = np.floor((p[1] + u[1] * s - org[1]) / res).astype(int)
        ok = (ix >= 0) & (ix < occ.shape[1]) & (iy >= 0) & (iy < occ.shape[0])
        hit = np.zeros(len(s), bool)
        hit[ok] = occ[iy[ok], ix[ok]]
        return float(s[hit.argmax()]) if hit.any() else max_m

    def classify(self, vmap, lost_xy, heading, inside_share: float = 0.0,
                 belief_xy=None, drone_xy=None, max_alt: float = 10.0) -> Scene:
        c = self.cfg
        L = (float(lost_xy[0]), float(lost_xy[1]))
        hd = _unit(heading) if heading is not None else (0.0, 1.0)
        g = self._grids(vmap)
        if g is None:
            return Scene(None, 0.0, {}, L, hd)
        d, unk = self._cast(g, L)
        n = c.rays
        near = d < c.near
        seen = unk < 0.5
        known_frac = float(seen.mean())

        # ROOM: walked into something thick along their heading
        i_h = int(round((math.atan2(hd[0], hd[1]) % (2 * math.pi)) / (2 * math.pi) * n)) % n
        idx = [(i_h + k) % n for k in (-1, 0, 1)]
        j = idx[int(np.argmin(d[idx]))]
        fwd = float(d[j])
        into = max(0.0, min(1.0, (c.into_m + 0.5 - fwd) / 1.0)) if math.isfinite(fwd) else 0.0
        thick = 0.0
        if into > 0:
            behind = [self._is_free(g, (L[0] + self.dirs[j][0] * (fwd + s), L[1] + self.dirs[j][1] * (fwd + s)))
                      for s in np.arange(0.3, c.thick_m + 0.6, 0.2)]
            thick = 1.0 - sum(1 for b in behind if b is True) / max(1, len(behind))
        s_room = into * thick * (0.6 + 0.4 * float(inside_share))

        # HALL: two facing walls, open down the gap (tested round the last-seen spot too)
        hall = self._hall(g, L, hd)
        for a in range(8):
            pa = (L[0] + c.hall_probe_m * math.sin(a * math.pi / 4), L[1] + c.hall_probe_m * math.cos(a * math.pi / 4))
            if self._is_free(g, pa) is False:
                continue
            h2 = self._hall(g, pa, hd)
            if h2[0] > hall[0]:
                hall = h2
        s_hall = hall[0] * (1.0 - s_room)

        # WALL: something beside them, long, not paired
        s_wall = 0.0
        geo = self._wall_geometry(g, L, belief_xy, drone_xy, vmap, max_alt)
        if near.any() and geo is not None:
            s_wall = min(1.0, near.mean() / 0.15) * (1.0 - hall[0]) * (1.0 - s_room)
            if geo["ratio"] < c.elongated:
                s_wall *= 0.5                      # a round pillar: the v2 search handles it

        sc = {WALL: s_wall, HALL: s_hall, ROOM: s_room}
        tot = sum(sc.values()) + c.unsure
        probs = {k: v / tot for k, v in sc.items()}
        kind = max(probs, key=probs.get)
        cert = probs[kind] * (0.7 + 0.3 * known_frac)
        if sc[kind] <= 0:
            kind, cert = None, 0.0
        out = Scene(kind, round(float(cert), 3), {k: round(float(v), 3) for k, v in probs.items()}, L, hd)
        if geo is not None:
            out.end, out.out, out.hidden = geo["end"], geo["out"], geo["hidden"]
            out.length, out.top, out.low = geo["length"], geo["top"], geo["low"]
            out.box = geo["box"]
        if hall[1] is not None:
            out.centre, out.axis, out.width, out.depth = hall[1], hall[2], hall[3], hall[4]
        if kind == ROOM:
            out.door = L
        return out

    # --- geometry --------------------------------------------------------------
    def _wall_geometry(self, g, L, belief_xy, drone_xy, vmap, max_alt):
        c = self.cfg
        occ, known, org, res = g
        r = int(math.ceil(c.geom_r / res))
        cx, cy = int(math.floor((L[0] - org[0]) / res)), int(math.floor((L[1] - org[1]) / res))
        y0, y1 = max(0, cy - r), min(occ.shape[0], cy + r + 1)
        x0, x1 = max(0, cx - r), min(occ.shape[1], cx + r + 1)
        ys, xs = np.nonzero(occ[y0:y1, x0:x1])
        if len(xs) < 5:
            return None
        px = org[0] + (xs + x0 + 0.5) * res
        py = org[1] + (ys + y0 + 0.5) * res
        keep = np.hypot(px - L[0], py - L[1]) <= c.geom_r
        px, py = px[keep], py[keep]
        if len(px) < 5:
            return None
        m = np.array([px.mean(), py.mean()])
        P = np.stack([px - m[0], py - m[1]], -1)
        ev, evec = np.linalg.eigh(P.T @ P / len(P))
        u = evec[:, 1]                                      # long axis
        ratio = math.sqrt(max(ev[1], 1e-9) / max(ev[0], 1e-4))
        t = P @ u
        tl = float(np.dot(np.array(L) - m, u))
        tend = float(t.max()) if abs(tl - t.max()) <= abs(tl - t.min()) else float(t.min())
        sgn = 1.0 if tend >= 0 else -1.0
        end = (float(m[0] + u[0] * tend), float(m[1] + u[1] * tend))
        out = (float(u[0] * sgn), float(u[1] * sgn))
        nrm = (-out[1], out[0])
        # which side are they on? where the belief is, else away from the drone
        side = 0.0
        if belief_xy is not None:
            side = (belief_xy[0] - m[0]) * nrm[0] + (belief_xy[1] - m[1]) * nrm[1]
        if abs(side) < 0.3 and drone_xy is not None:
            side = -((drone_xy[0] - m[0]) * nrm[0] + (drone_xy[1] - m[1]) * nrm[1])
        if side < 0:
            nrm = (-nrm[0], -nrm[1])
        # height of the wall, and can we see over it (open air seen above the top)?
        top, low = None, False
        if vmap is not None and hasattr(vmap, "height_at"):
            hs = [vmap.height_at(x, y) for x, y in zip(px[::3], py[::3])]
            hs = [h for h in hs if h is not None]
            if hs:
                top = float(np.percentile(hs, 90))
                if hasattr(vmap, "free_above"):
                    idx = np.argsort(np.hypot(px - end[0], py - end[1]))[:12]
                    low = top + 1.0 <= max_alt - 1.0 and any(vmap.free_above(px[i], py[i], top) for i in idx)
        box = (float(px.min()), float(py.min()), float(px.max()), float(py.max()))
        return {"end": end, "out": out, "hidden": nrm, "length": float(t.max() - t.min()), "ratio": ratio,
                "top": top, "low": low, "box": box}
