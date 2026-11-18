from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from drone_core.domain.models import BoundingBox, Telemetry


@dataclass
class CameraGeometry:
    """Must match the camera that produces the frames (Unity SceneBootstrap,
    the mock sim, or your real camera's calibration)."""
    hfov_deg: float = 90.0
    pitch_deg: float = 20.0          # tilted down
    target_height_m: float = 1.8     # height of what the tracker boxes (whole person for YOLO)
    yaw_deg: float = 0.0             # mount direction relative to the nose (+ = right)
    target_bottom_m: float = 0.0     # height of the box's bottom edge above the ground
                                     # (0 = feet; the Unity colour tracker sees only the shirt)

    def focal_px(self, frame_w: int) -> float:
        return (frame_w / 2) / math.tan(math.radians(self.hfov_deg) / 2)

    def bearing(self, box: BoundingBox, frame_w: int) -> float:
        """Direction of the box centre relative to the drone's nose, radians (+ = right)."""
        return math.radians(self.yaw_deg) + math.atan((box.cx - frame_w / 2) / self.focal_px(frame_w))


def ground_range(box: BoundingBox, frame_w: int, frame_h: int, telem: Telemetry,
                 cam: CameraGeometry) -> Optional[tuple[float, bool]]:
    """Horizontal distance (m) to the target, and whether the box is cut off by
    the top or bottom of the image.

    Two estimates:
      * size: pinhole range from box height. Fine when the drone is level with
        the person, but from above the person is foreshortened and the box
        shrinks, which reads as "too far" and pulls the drone over the top.
      * geometry: altitude / tan(depression) of the box's bottom (or top) edge,
        assuming flat ground. Accurate exactly when the size estimate is not:
        when looking down steeply.
    They are blended by how steeply the camera looks down at the target.
    """
    if box.h < 2:
        return None
    f = cam.focal_px(frame_w)
    pitch = math.radians(cam.pitch_deg)
    margin = 0.01 * frame_h
    cut_top = box.y1 <= margin
    cut_bottom = box.y2 >= frame_h - 1 - margin
    elev = pitch + math.atan((box.cy - frame_h / 2) / f)
    d_size = f * cam.target_height_m / box.h * math.cos(elev)

    best = None   # (depression, distance) from an edge that is in view
    alt = telem.altitude
    for cut, y, z in ((cut_bottom, box.y2, cam.target_bottom_m),
                      (cut_top, box.y1, cam.target_bottom_m + cam.target_height_m)):
        dep = pitch + math.atan((y - frame_h / 2) / f)
        if not cut and alt - z > 0.3 and dep > math.radians(3):
            d = (alt - z) / math.tan(dep)
            if best is None or dep > best[0]:
                best = (dep, d)
    if best is None:
        d = d_size
    else:
        # 0 when nearly level (5 deg), 1 when looking down 20 deg or more
        w = min(1.0, max(0.0, (math.degrees(best[0]) - 5.0) / 15.0))
        d = w * best[1] + (1 - w) * d_size
    return max(0.0, d), (cut_top or cut_bottom)


def box_to_world(box: BoundingBox, frame_w: int, frame_h: int, telem: Telemetry,
                 cam: CameraGeometry) -> Optional[tuple[float, float]]:
    """Rough ground position (x east, y north) of a target from its box
    (range from ground_range, bearing from the horizontal offset)."""
    r = ground_range(box, frame_w, frame_h, telem, cam)
    if r is None:
        return None
    horiz = r[0]
    bearing = telem.yaw + cam.bearing(box, frame_w)
    x, y = telem.position[0], telem.position[1]
    return x + horiz * math.sin(bearing), y + horiz * math.cos(bearing)


class TargetWorldTrack:
    """Smoothed world position + velocity of the target, so we can predict
    where it went while it is hidden."""

    def __init__(self, alpha_pos: float = 0.35, alpha_vel: float = 0.25, max_speed: float = 6.0):
        self.alpha_pos, self.alpha_vel, self.max_speed = alpha_pos, alpha_vel, max_speed
        self.pos: Optional[tuple[float, float]] = None
        self.vel = (0.0, 0.0)
        self.t: Optional[float] = None

    def reset(self) -> None:
        self.pos, self.vel, self.t = None, (0.0, 0.0), None

    def update(self, p: tuple[float, float], t: float) -> None:
        if self.pos is None or self.t is None or t - self.t > 1.0:
            self.pos, self.t, self.vel = p, t, (0.0, 0.0)
            self._n = 1
            return
        dt = max(1e-3, t - self.t)
        px = self.pos[0] + self.vel[0] * dt
        py = self.pos[1] + self.vel[1] * dt
        nx = px + self.alpha_pos * (p[0] - px)
        ny = py + self.alpha_pos * (p[1] - py)
        vx = self.vel[0] + self.alpha_vel * ((nx - self.pos[0]) / dt - self.vel[0])
        vy = self.vel[1] + self.alpha_vel * ((ny - self.pos[1]) / dt - self.vel[1])
        s = math.hypot(vx, vy)
        if s > self.max_speed:
            vx, vy = vx * self.max_speed / s, vy * self.max_speed / s
        self.pos, self.vel, self.t = (nx, ny), (vx, vy), t

    def predict(self, t: float, horizon_cap: float = 4.0) -> Optional[tuple[float, float]]:
        if self.pos is None or self.t is None:
            return None
        dt = min(max(0.0, t - self.t), horizon_cap)
        return self.pos[0] + self.vel[0] * dt, self.pos[1] + self.vel[1] * dt


def rig_geometry(mounts, target_height_m: float = 1.8, target_bottom_m: float = 0.0) -> dict:
    """CameraMount list -> {name: CameraGeometry}."""
    return {m.name: CameraGeometry(m.hfov_deg, m.pitch_deg, target_height_m, m.yaw_deg, target_bottom_m)
            for m in mounts}
