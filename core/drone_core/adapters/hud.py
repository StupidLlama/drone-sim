"""Debug overlay (OpenCV window). Presentation only — not used by use cases."""
from __future__ import annotations

import math
import time

import cv2
import numpy as np

from drone_core.usecases.follow_mission import TickResult

_COLORS = {"TRACKING": (0, 220, 0), "COASTING": (0, 220, 160), "LOST": (0, 200, 255), "TURNING": (255, 255, 0),
           "REACQUIRING": (0, 160, 255),
           "SEARCHING": (0, 140, 255), "LOITER": (0, 120, 255), "CONFIRMING": (0, 255, 200),
           "GAVE_UP": (120, 120, 255), "IDLE": (180, 180, 180), "RTH": (255, 120, 0), "HOLD": (255, 200, 0)}


def draw(result: TickResult):
    img = result.frame.image.copy()
    st = result.status
    locked = st.target.track_id if st.target else None
    for t in result.tracks:
        b = t.box
        color = (0, 255, 0) if t.track_id == locked else (200, 200, 200)
        cv2.rectangle(img, (int(b.x1), int(b.y1)), (int(b.x2), int(b.y2)), color, 2)
        cv2.putText(img, f"#{t.track_id} {t.class_name} {t.confidence:.2f}", (int(b.x1), int(b.y1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    h, w = img.shape[:2]
    cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
    c = st.command
    lines = [st.state,
             f"fwd {c.forward:+.2f}  right {c.right:+.2f}  up {c.up:+.2f}  yaw {c.yaw_rate:+.2f}",
             "  ".join(f"{k} {v:+.2f}" if isinstance(v, float) else f"{k} {v}" for k, v in st.errors.items())]
    tm = result.telemetry
    if tm is not None:
        lines.append(f"alt {tm.altitude:.1f} m  batt {tm.battery * 100:.0f}%"
                     + (f"  front {tm.ranges['front']:.1f} m" if "front" in tm.ranges else ""))
    lines.append("keys: f follow  h return home  space hold  q quit")
    for i, s in enumerate(lines):
        cv2.putText(img, s, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    _COLORS.get(st.state.split(" ")[0], (255, 255, 255)) if i == 0 else (255, 255, 255), 1)
    return img


_CAM_TEXT = {"front": "FRONT", "right": "RIGHT", "back": "BACK", "left": "LEFT"}
_CAM_ARROW = {"front": "^", "right": ">", "back": "v", "left": "<"}


def label(img, text, org=(0, 0), color=(255, 255, 255), bg=(0, 0, 0), scale=0.6):
    """Text on a solid badge so it reads on any background."""
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
    x, y = org
    cv2.rectangle(img, (x, y), (x + tw + 10, y + th + base + 8), bg, -1)
    cv2.putText(img, text, (x + 5, y + th + 4), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def _thumb(img, tracks, locked, name, w=320):
    h = int(img.shape[0] * w / img.shape[1])
    sx = w / img.shape[1]
    out = cv2.resize(img, (w, h))
    has_target = False
    for t in tracks:
        b = t.box
        mine = (t.track_id, t.camera) == locked
        has_target |= mine
        color = (0, 255, 0) if mine else (200, 200, 200)
        cv2.rectangle(out, (int(b.x1 * sx), int(b.y1 * sx)), (int(b.x2 * sx), int(b.y2 * sx)), color, 2)
    text = f"{_CAM_ARROW.get(name, '')} {_CAM_TEXT.get(name, name.upper())}".strip()
    label(out, text, (0, 0), bg=(0, 110, 0) if has_target else (40, 40, 40))
    if has_target:
        label(out, "TARGET", (w - 82, 0), color=(0, 0, 0), bg=(0, 255, 0), scale=0.5)
    cv2.rectangle(out, (0, 0), (w - 1, h - 1), (90, 90, 90), 1)    # tile border
    return out


def draw_map(result: TickResult, surround_map=None, follow=None, size: int = 480, view_m: float = 30.0,
             voxel_map=None):
    """Top-down map centred on the drone, nose up. Grey = obstacles from the
    cameras, green dot = followed target estimate, orange = predicted spot.
    3-D map voxels: red = above the drone, yellow = at its height, blue = below."""
    img = np.full((size, size, 3), 30, np.uint8)
    tm = result.telemetry
    if tm is None:
        return img
    px, py, yaw = tm.position[0], tm.position[1], tm.yaw
    scale = size / view_m                      # pixels per metre
    c, s = math.cos(yaw), math.sin(yaw)

    def to_px(wx, wy):
        dx, dy = wx - px, wy - py
        fwd = dx * s + dy * c                   # world -> body (nose up)
        right = dx * c - dy * s
        return int(size / 2 + right * scale), int(size / 2 - fwd * scale)

    def to_px_arr(wx, wy):
        dx, dy = wx - px, wy - py
        fwd = dx * s + dy * c
        right = dx * c - dy * s
        x = (size / 2 + right * scale).astype(int)
        y = (size / 2 - fwd * scale).astype(int)
        ok = (x >= 0) & (x < size) & (y >= 0) & (y < size)
        return x[ok], y[ok], ok

    # Vectorised: drawing cell by cell in Python cost ~50 ms a frame.
    if surround_map is not None and surround_map.origin is not None:
        res = surround_map.cfg.resolution
        o = surround_map.origin
        rows, cols = np.nonzero(surround_map.logodds < -0.5)
        x, y, _ = to_px_arr(o[0] + (cols + 0.5) * res, o[1] + (rows + 0.5) * res)
        img[y, x] = (55, 55, 55)
        rows, cols = np.nonzero(surround_map.occupied())
        x, y, _ = to_px_arr(o[0] + (cols + 0.5) * res, o[1] + (rows + 0.5) * res)
        r_px = max(1, int(res * scale / 2))
        occ = np.zeros((size, size), np.uint8)
        occ[y, x] = 255
        occ = cv2.dilate(occ, np.ones((2 * r_px + 1, 2 * r_px + 1), np.uint8))
        img[occ > 0] = (200, 200, 200)
    if voxel_map is not None:
        pts = voxel_map.occupied_points()
        if len(pts):
            z0 = tm.position[2]
            x, y, ok = to_px_arr(pts[:, 0], pts[:, 1])
            z = pts[ok, 2]
            col = np.where((z > z0 + 0.6)[:, None], [[60, 60, 255]],
                           np.where((z < z0 - 0.6)[:, None], [[255, 140, 40]], [[0, 230, 255]])).astype(np.uint8)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    xx, yy = np.clip(x + dx, 0, size - 1), np.clip(y + dy, 0, size - 1)
                    img[yy, xx] = col
    for m in range(5, int(view_m / 2) + 1, 5):   # range rings every 5 m
        cv2.circle(img, (size // 2, size // 2), int(m * scale), (70, 70, 70), 1)
    if follow is not None and follow.world.pos is not None:
        cv2.circle(img, to_px(*follow.world.pos), 6, (0, 220, 0), -1)
        pred = follow.world.predict(follow._last_t or 0.0)
        if pred is not None and result.status.state.startswith(("REACQ", "SEARCH", "LOITER")):
            cv2.circle(img, to_px(*pred), 8, (0, 160, 255), 2)
    # drone: triangle pointing up
    cx, cy = size // 2, size // 2
    cv2.fillPoly(img, [np.array([(cx, cy - 10), (cx - 7, cy + 7), (cx + 7, cy + 7)])], (255, 200, 0))
    cv2.putText(img, f"map {view_m:.0f} m  (nose up)", (8, size - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    return img


def draw_map3d(result: TickResult, voxel_map=None, follow=None, size: int = 480, view_yaw_deg: float = 0.0,
               view_dist: float = 30.0, view_pitch_deg: float = 50.0):
    """3-D view of the voxel map from behind and above the drone (perspective).
    Blocks are coloured by height above the ground (blue low -> red high).
    view_yaw_deg spins the view around the drone ([ and ] keys)."""
    img = np.full((size, size, 3), 22, np.uint8)
    tm = result.telemetry
    if tm is None:
        label(img, "3D MAP (no telemetry)", (0, 0), bg=(40, 40, 40))
        return img
    centre = np.array([tm.position[0], tm.position[1], tm.position[2] * 0.5])
    yaw = tm.yaw + math.radians(view_yaw_deg)       # look the way the drone faces (+ offset)
    pitch = math.radians(view_pitch_deg)
    fwd = np.array([math.sin(yaw) * math.cos(pitch), math.cos(yaw) * math.cos(pitch), -math.sin(pitch)])
    eye = centre - fwd * view_dist
    right = np.array([math.cos(yaw), -math.sin(yaw), 0.0])
    up = np.cross(right, fwd)
    f = size * 1.0                                   # ~30 m across at the drone

    def project(pts):
        rel = np.atleast_2d(pts) - eye
        z = rel @ fwd
        x = rel @ right
        y = rel @ up
        zc = np.maximum(z, 0.1)
        return size / 2 + f * x / zc, size / 2 - f * y / zc, z

    # ground grid every 2 m around the drone
    g0x, g0y = math.floor(tm.position[0] / 2) * 2, math.floor(tm.position[1] / 2) * 2
    for k in range(-15, 16):
        for a, b in (((g0x + 2 * k, g0y - 30, 0), (g0x + 2 * k, g0y + 30, 0)),
                     ((g0x - 30, g0y + 2 * k, 0), (g0x + 30, g0y + 2 * k, 0))):
            u, v, z = project(np.array([a, b], float))
            if (z > 0.5).all():
                cv2.line(img, (int(u[0]), int(v[0])), (int(u[1]), int(v[1])), (55, 55, 55), 1)
    n = 0
    if voxel_map is not None:
        pts = voxel_map.occupied_points()
        n = len(pts)
        if n:
            if n > 4000:                                    # keep it quick
                pts = pts[np.random.default_rng(0).choice(n, 4000, replace=False)]
            u, v, z = project(pts)
            ok = (z > 0.5) & (u > -10) & (u < size + 10) & (v > -10) & (v < size + 10)
            pts, u, v, z = pts[ok], u[ok], v[ok], z[ok]
        if n and len(pts):
            order = np.argsort(-z)                          # far first: near blocks draw on top
            hue = np.clip(pts[order, 2] / 6.0, 0, 1)        # 0 m blue .. 6 m red
            cmap = cv2.applyColorMap((hue * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
            half = np.clip(f * voxel_map.cfg.resolution / z[order] * 0.55, 1, 12).astype(int)
            for x, y, hs, col in zip(u[order].astype(int), v[order].astype(int), half, cmap):
                c = tuple(int(q) for q in col)
                cv2.rectangle(img, (x - hs, y - hs), (x + hs, y + hs), c, -1)
                if hs >= 4:
                    cv2.rectangle(img, (x - hs, y - hs), (x + hs, y + hs), (0, 0, 0), 1)

    def marker(p, color, r, text=None, left=False):
        u, v, z = project(np.array(p, float))
        if z[0] > 0.5:
            pg = project(np.array([p[0], p[1], 0.0]))       # stalk down to the ground
            cv2.line(img, (int(u[0]), int(v[0])), (int(pg[0][0]), int(pg[1][0])), color, 1)
            cv2.circle(img, (int(u[0]), int(v[0])), r, color, -1)
            if text:
                (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                x = int(u[0]) + r + 3 if left is False else int(u[0]) - r - 3 - tw
                cv2.putText(img, text, (x, int(v[0]) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    if follow is not None and follow.world.pos is not None:
        top = follow.cfg.camera.target_bottom_m + follow.cfg.camera.target_height_m
        marker((follow.world.pos[0], follow.world.pos[1], top), (0, 220, 0), 6, "target", left=True)
    marker(tm.position, (255, 200, 0), 7, f"drone {tm.altitude:.1f} m")
    # nose direction
    p0 = np.array(tm.position, float)
    p1 = p0 + 1.5 * np.array([math.sin(tm.yaw), math.cos(tm.yaw), 0.0])
    u, v, z = project(np.array([p0, p1]))
    if (z > 0.5).all():
        cv2.arrowedLine(img, (int(u[0]), int(v[0])), (int(u[1]), int(v[1])), (255, 200, 0), 2, tipLength=0.3)
    label(img, "3D MAP", (0, 0), bg=(40, 40, 40))
    cv2.putText(img, f"{n} blocks   colour = height (blue 0 m, red 6 m+)   [ ] rotate",
                (8, size - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
    return img


def draw_rig(result: TickResult, surround_map=None, follow=None, voxel_map=None, view_yaw_deg: float = 0.0):
    """2x2 camera mosaic + top-down map (+ 3-D map view when there is a voxel map)."""
    st = result.status
    locked = (st.target.track_id, st.target.camera) if st.target else None
    order = ["front", "right", "left", "back"]
    thumbs = []
    for name in order:
        f = result.multi.frames.get(name) if result.multi else None
        if f is None:
            continue
        tr = result.tracks if name == "front" else result.other_tracks.get(name, [])
        thumbs.append(_thumb(f.image, tr, locked, name))
    while len(thumbs) < 4:
        thumbs.append(np.zeros_like(thumbs[0]))
    mosaic = np.vstack([np.hstack(thumbs[:2]), np.hstack(thumbs[2:4])])
    m = draw_map(result, surround_map, follow, size=mosaic.shape[0], voxel_map=voxel_map)
    label(m, "TOP-DOWN MAP", (0, 0), bg=(40, 40, 40))
    panels = [mosaic, m]
    if voxel_map is not None:
        panels.append(draw_map3d(result, voxel_map, follow, size=mosaic.shape[0], view_yaw_deg=view_yaw_deg))
    out = np.hstack(panels)
    c = st.command
    lines = [st.state, f"fwd {c.forward:+.2f} right {c.right:+.2f} up {c.up:+.2f} yaw {c.yaw_rate:+.2f}"]
    tm = result.telemetry
    if tm is not None:
        lines.append(f"alt {tm.altitude:.1f} m  batt {tm.battery * 100:.0f}%  "
                     + "  ".join(f"{k} {v:.1f}" for k, v in sorted(tm.ranges.items()) if k != "down"))
    lines.append("keys: f follow  h return home  space hold  [ ] turn 3D view  q quit")
    for i, s in enumerate(lines):
        cv2.putText(out, s, (8, out.shape[0] - 12 - 18 * (len(lines) - 1 - i)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    _COLORS.get(st.state.split(" ")[0], (255, 255, 255)) if i == 0 else (255, 255, 255), 1)
    return out


class HudWindow:
    def __init__(self, name: str = "drone core", on_key=None, surround_map=None, follow=None, voxel_map=None):
        self.name = name
        self.quit = False
        self.on_key = on_key      # called with the pressed character
        self.surround_map = surround_map
        self.follow = follow
        self.voxel_map = voxel_map
        self.max_fps = 10.0
        self.view_yaw = 0.0       # 3-D map view angle around the drone, degrees
        self._last_draw = 0.0

    def __call__(self, result: TickResult) -> None:
        # Drawing + imshow is the slowest part of a tick: refresh ~10 times a second.
        now = time.monotonic()
        if now - self._last_draw < 1.0 / self.max_fps:
            return
        self._last_draw = now
        if result.multi is not None and len(result.multi.frames) > 1:
            img = draw_rig(result, self.surround_map, self.follow, self.voxel_map, self.view_yaw)
        else:
            img = draw(result)
        cv2.imshow(self.name, img)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self.quit = True
        elif key in (ord("["), ord("]")):
            self.view_yaw += 45.0 if key == ord("]") else -45.0
        elif key != 255 and self.on_key:
            self.on_key(chr(key))
