"""A tiny pure-Python simulator implementing every port, so the whole core can
be run and unit-tested without Unity or YOLO.

World frame: x = east, y = north, z = up (metres). Yaw = compass heading in
radians, 0 = +y, positive = clockwise (turn right).

Lock-step: each CameraSource.read() advances the world by 1/fps, so tests are
deterministic and run faster than real time.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Optional

import cv2
import numpy as np

from drone_core.domain.models import (BoundingBox, CameraMount, Frame, MultiFrame, SonarBeam, SonarScan,
                                      Telemetry, Track, VelocityCommand, sonar_layout)
from drone_core.usecases.voxel_map import body_to_world, cone_dirs

TargetPath = Callable[[float], tuple[float, float]]


def circle_path(radius: float = 6.0, speed: float = 1.2, center=(0.0, 10.0)) -> TargetPath:
    w = speed / radius
    return lambda t: (center[0] + radius * math.sin(w * t), center[1] - radius * math.cos(w * t) + radius)


def straight_path(speed: float = 1.5, start=(0.0, 6.0), heading_deg: float = 30.0) -> TargetPath:
    h = math.radians(heading_deg)
    return lambda t: (start[0] + speed * t * math.sin(h), start[1] + speed * t * math.cos(h))


@dataclass
class MockPerson:
    """Someone else walking around (a decoy). Colours are BGR."""
    path: TargetPath
    top: tuple = (200, 120, 40)            # shirt
    bottom: tuple = (60, 50, 40)           # trousers
    size: tuple = (0.5, 0.4, 1.8)


@dataclass
class MockWorldConfig:
    fps: float = 20.0
    width: int = 640
    height: int = 480
    hfov_deg: float = 90.0
    cam_pitch_deg: float = 20.0           # camera tilted down
    drone_start: tuple[float, float, float] = (0.0, 0.0, 3.0)
    tau: float = 0.35                     # velocity response time constant (s)
    wind: tuple[float, float, float] = (0.0, 0.0, 0.0)  # m/s^2 disturbance accel (world)
    drag: float = 0.1                     # extra linear drag (1/s)
    target_size: tuple[float, float, float] = (0.5, 0.4, 1.8)  # w, d, h (m)
    target_base: float = 0.0              # m above ground of the red part (0.88 = shirt only, as in Unity)
    target_colors: tuple = ((30, 30, 220), (30, 30, 220))  # BGR shirt, trousers (all red by default)
    people: tuple = ()                    # other people (MockPerson), seen by MockPersonTracker
    target_hidden: tuple = ()             # ((t0, t1), ...) target out of sight (went into a shop, a tent...)
    noise: float = 4.0                    # pixel noise std-dev
    latency_frames: int = 0               # camera delay: frames arrive this many ticks late
    obstacles: tuple = ()                 # vertical pillars: (x, y, radius) in metres
    range_max: float = 20.0
    range_sensors: bool = True            # False = no rangefinders (cameras + map only)
    battery_drain_per_s: float = 0.0      # fraction per second (0 = infinite battery)
    # Thin branches: ((x1, y1, z1), (x2, y2, z2), radius). Solid for the sonar and
    # for collisions, but NOT drawn in the depth images (too thin for a 32x24
    # depth grid / a monocular depth net), which is the point of the sonar.
    branches: tuple = ()
    sonar: bool = False                   # simulate the 18-transducer ultrasonic array
    sonar_fov_deg: float = 30.0
    sonar_max: float = 7.0
    sonar_min_echo: float = 0.03          # fraction of the beam an object must fill to echo
    target_path: TargetPath = field(default_factory=circle_path)


class MockWorld:
    def __init__(self, cfg: MockWorldConfig | None = None):
        self.cfg = cfg or MockWorldConfig()
        self.reset()
        self._f = (self.cfg.width / 2) / math.tan(math.radians(self.cfg.hfov_deg) / 2)
        self._rng = np.random.default_rng(0)

    def reset(self) -> None:
        self.t = 0.0
        self.frame_id = 0
        self.pos = np.array(self.cfg.drone_start, dtype=float)
        self.vel = np.zeros(3)
        self.yaw = 0.0
        self.cmd = VelocityCommand()
        self.battery = 1.0
        self.min_clearance = float("inf")   # closest approach to any obstacle surface
        self.min_branch_clearance = float("inf")
        self.target_visible_override: Optional[bool] = None  # tests can hide the target

    # --- physics -----------------------------------------------------------
    def _axes(self, extra_yaw: float = 0.0):
        y = self.yaw + extra_yaw
        fwd = np.array([math.sin(y), math.cos(y), 0.0])
        right = np.array([math.cos(y), -math.sin(y), 0.0])
        return fwd, right

    def target_pos(self) -> np.ndarray:
        x, y = self.cfg.target_path(self.t)
        return np.array([x, y, 0.0])

    def step(self, dt: float) -> None:
        fwd, right = self._axes()
        v_cmd = fwd * self.cmd.forward + right * self.cmd.right + np.array([0, 0, self.cmd.up])
        acc = (v_cmd - self.vel) / self.cfg.tau + np.array(self.cfg.wind) - self.cfg.drag * self.vel
        self.vel = self.vel + acc * dt
        self.pos = self.pos + self.vel * dt
        self.pos[2] = max(0.0, self.pos[2])
        self.yaw = (self.yaw + self.cmd.yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        self.t += dt
        self.battery = max(0.0, self.battery - self.cfg.battery_drain_per_s * dt)
        for ox, oy, r in self.cfg.obstacles:
            self.min_clearance = min(self.min_clearance, math.hypot(self.pos[0] - ox, self.pos[1] - oy) - r)
        for a, b, r in self.cfg.branches:
            d = _point_segment_dist(self.pos, np.array(a, float), np.array(b, float)) - r
            self.min_branch_clearance = min(self.min_branch_clearance, d)
            self.min_clearance = min(self.min_clearance, d)

    def _line_of_sight_blocked(self) -> bool:
        """Pillars are treated as infinitely tall: they hide the target."""
        return self._blocked_to(self.target_pos())

    def _blocked_to(self, tp) -> bool:
        ax, ay = self.pos[0], self.pos[1]
        bx, by = tp[0], tp[1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 < 1e-9:
            return False
        for ox, oy, r in self.cfg.obstacles:
            u = max(0.0, min(1.0, ((ox - ax) * dx + (oy - ay) * dy) / L2))
            cx, cy = ax + u * dx, ay + u * dy
            if math.hypot(ox - cx, oy - cy) < r:
                return True
        return False

    # --- range sensors -----------------------------------------------------
    def _ray(self, d: np.ndarray) -> Optional[float]:
        """Distance along horizontal unit vector d to the nearest pillar."""
        best = None
        px, py = self.pos[0], self.pos[1]
        for ox, oy, r in self.cfg.obstacles:
            fx, fy = px - ox, py - oy
            b = fx * d[0] + fy * d[1]
            c = fx * fx + fy * fy - r * r
            disc = b * b - c
            if disc < 0:
                continue
            t = -b - math.sqrt(disc)
            if 0 <= t <= self.cfg.range_max and (best is None or t < best):
                best = t
        return best

    def ranges(self) -> dict:
        fwd, right = self._axes()
        s = math.sqrt(0.5)
        dirs = {"front": fwd, "back": -fwd, "left": -right, "right": right,
                "front_left": (fwd - right) * s, "front_right": (fwd + right) * s}
        out = {"down": float(self.pos[2])}
        if not self.cfg.range_sensors:
            return out
        for k, d in dirs.items():
            dist = self._ray(d)
            if dist is not None:
                out[k] = float(dist)
        return out

    # --- sonar -------------------------------------------------------------
    _SONAR_RINGS = ((0.25, 6), (0.5, 12), (0.75, 18), (1.0, 24))

    def _hit_dists(self, dirs: np.ndarray, max_t: float, ray_cell: float = 0.0):
        """Distance along each world unit ray to pillars, ground and branches
        (inf = none), and how much of that ray's patch of the beam the object
        fills (1 = all of it; a thin branch fills only a sliver)."""
        o = self.pos
        t = np.full(len(dirs), np.inf)
        cover = np.ones(len(dirs))
        dz = dirs[:, 2]
        g = dz < -1e-6
        t[g] = -o[2] / dz[g]                                   # ground
        for ox, oy, r in self.cfg.obstacles:                   # vertical pillars, infinitely tall
            fx, fy = o[0] - ox, o[1] - oy
            dx, dy = dirs[:, 0], dirs[:, 1]
            a = dx * dx + dy * dy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - r * r
            disc = b * b - a * c
            ok = (disc >= 0) & (a > 1e-9)
            tt = np.where(ok, (-b - np.sqrt(np.maximum(disc, 0))) / np.maximum(a, 1e-9), np.inf)
            tt[tt < 0] = np.inf
            t = np.minimum(t, tt)
        for a3, b3, r in self.cfg.branches:
            a3, b3 = np.array(a3, float), np.array(b3, float)
            tr, dist = _ray_segment(o, dirs, a3, b3, max_t)
            hit = dist < r
            tb = np.where(hit, np.maximum(0.0, tr - np.sqrt(np.maximum(r * r - dist * dist, 0.0))), np.inf)
            closer = tb < t
            if ray_cell > 0:
                cover[closer] = np.minimum(1.0, 2 * r / np.maximum(tb[closer] * ray_cell, 1e-6))
            t = np.where(closer, tb, t)
        return t, cover

    def sonar(self) -> SonarScan:
        c = self.cfg
        beams = []
        for yaw, pitch in sonar_layout():
            dirs = body_to_world(cone_dirs(yaw, pitch, c.sonar_fov_deg, self._SONAR_RINGS), self.yaw)
            cell = math.radians(c.sonar_fov_deg) / 2 / len(self._SONAR_RINGS)   # angle between rings
            t, cover = self._hit_dists(dirs, c.sonar_max, cell)
            rng = None
            # First echo strong enough to hear: an object must fill a big enough
            # part of the beam. Weaker ones (a far-away twig) are missed and the
            # next surface behind answers instead.
            alive = np.isfinite(t) & (t <= c.sonar_max)
            while alive.any():
                t0 = t[alive].min()
                group = alive & (t < t0 + 0.3)
                if cover[group].sum() / len(t) >= c.sonar_min_echo:
                    rng = float(t0 + self._rng.normal(0.0, 0.01 + 0.01 * t0))
                    break
                alive &= ~group
            beams.append(SonarBeam(yaw, pitch, None if rng is None else max(0.2, rng)))
        return SonarScan(tuple(beams), c.sonar_fov_deg, 0.2, c.sonar_max)

    # --- camera ------------------------------------------------------------
    def _to_cam(self, p: np.ndarray, mount_yaw_deg: float = 0.0) -> np.ndarray:
        fwd, right = self._axes(math.radians(mount_yaw_deg))
        rel = p - self.pos
        f_b, r_b, u_b = rel @ fwd, rel @ right, rel[2]
        th = math.radians(self.cfg.cam_pitch_deg)
        z = f_b * math.cos(th) - u_b * math.sin(th)
        y = -(u_b * math.cos(th) + f_b * math.sin(th))
        return np.array([r_b, y, z])

    def _target_hidden(self) -> bool:
        return self.target_visible_override is False or any(a <= self.t < b for a, b in self.cfg.target_hidden)

    def target_box(self, mount_yaw_deg: float = 0.0) -> Optional[BoundingBox]:
        if self._target_hidden():
            return None
        if self._line_of_sight_blocked():
            return None
        w, d, h = self.cfg.target_size
        return self._box(self.target_pos(), w, d, self.cfg.target_base, self.cfg.target_base + h, mount_yaw_deg)

    def _box(self, c, w, d, z0, z1, mount_yaw_deg) -> Optional[BoundingBox]:
        us, vs = [], []
        for dx in (-w / 2, w / 2):
            for dy in (-d / 2, d / 2):
                for dz in (z0, z1):
                    xc, yc, zc = self._to_cam(c + np.array([dx, dy, dz]), mount_yaw_deg)
                    if zc < 0.2:
                        return None
                    us.append(self.cfg.width / 2 + self._f * xc / zc)
                    vs.append(self.cfg.height / 2 + self._f * yc / zc)
        x1, x2 = max(0.0, min(us)), min(self.cfg.width - 1.0, max(us))
        y1, y2 = max(0.0, min(vs)), min(self.cfg.height - 1.0, max(vs))
        if x2 - x1 < 2 or y2 - y1 < 2:
            return None
        return BoundingBox(x1, y1, x2, y2)

    # --- several people ------------------------------------------------------
    def person_pos(self, i: int) -> np.ndarray:
        """0 = the target, 1.. = cfg.people."""
        if i == 0:
            return self.target_pos()
        x, y = self.cfg.people[i - 1].path(self.t)
        return np.array([x, y, 0.0])

    def _person(self, i: int):
        """(size, base, top colour, bottom colour)"""
        if i == 0:
            return self.cfg.target_size, self.cfg.target_base, self.cfg.target_colors[0], self.cfg.target_colors[1]
        p = self.cfg.people[i - 1]
        return p.size, 0.0, p.top, p.bottom

    def visible_people(self, mount_yaw_deg: float = 0.0, max_cover: float = 0.6) -> list:
        """[(index, box, distance)] near first, without people hidden behind
        pillars or mostly covered by someone nearer."""
        out = []
        for i in range(1 + len(self.cfg.people)):
            if i == 0 and self._target_hidden():
                continue
            p = self.person_pos(i)
            if self._blocked_to(p):
                continue
            (w, d, h), base, _, _ = self._person(i)
            b = self._box(p, w, d, base, base + h, mount_yaw_deg)
            if b is not None:
                out.append((i, b, float(np.hypot(*(p[:2] - self.pos[:2])))))
        out.sort(key=lambda e: e[2])
        kept = []
        for i, b, dist in out:
            covered = max((_overlap(b, k[1]) / max(b.area, 1e-6) for k in kept), default=0.0)
            if covered < max_cover:
                kept.append((i, b, dist))
        return kept

    def render(self, mount_yaw_deg: float = 0.0) -> np.ndarray:
        H, W = self.cfg.height, self.cfg.width
        img = np.full((H, W, 3), (200, 180, 150), np.uint8)          # sky
        # horizon from camera pitch
        horizon = int(H / 2 - self._f * math.tan(math.radians(self.cfg.cam_pitch_deg)))
        horizon = max(0, min(H, horizon))
        img[horizon:] = (70, 110, 70)                                  # grass
        if not self.cfg.people:
            box = self.target_box(mount_yaw_deg)
            if box is not None:
                self._draw_person(img, 0, mount_yaw_deg)
        else:
            people = []
            for i in range(1 + len(self.cfg.people)):
                if i == 0 and self._target_hidden():
                    continue
                p = self.person_pos(i)
                if not self._blocked_to(p):
                    people.append((float(np.hypot(*(p[:2] - self.pos[:2]))), i))
            for _, i in sorted(people, reverse=True):                  # far first
                self._draw_person(img, i, mount_yaw_deg)
        if self.cfg.noise > 0:
            n = self._rng.normal(0, self.cfg.noise, img.shape)
            img = np.clip(img.astype(np.float32) + n, 0, 255).astype(np.uint8)
        return img

    def _draw_person(self, img, i, mount_yaw_deg):
        (w, d, h), base, top, bottom = self._person(i)
        c = self.person_pos(i)
        mid = base + 0.5 * h
        for z0, z1, col in ((base, mid, bottom), (mid, base + h, top)):
            b = self._box(c, w, d, z0, z1, mount_yaw_deg)
            if b is not None:
                cv2.rectangle(img, (int(b.x1), int(b.y1)), (int(b.x2), int(b.y2)), col, -1)

    def depth(self, mount_yaw_deg: float = 0.0, w: int = 64, h: int = 48) -> np.ndarray:
        """Ground-truth depth (metres along each pixel ray) at low resolution:
        ground plane + infinitely tall pillars. inf = sky / nothing in range."""
        f = (w / 2) / math.tan(math.radians(self.cfg.hfov_deg) / 2)
        th = math.radians(self.cfg.cam_pitch_deg)
        fwd, right = self._axes(math.radians(mount_yaw_deg))
        us = (np.arange(w) + 0.5 - w / 2) / f            # x / z in camera
        vs = (np.arange(h) + 0.5 - h / 2) / f            # y(down) / z
        out = np.full((h, w), np.inf, np.float32)
        alt = max(self.pos[2], 1e-3)
        for j, v in enumerate(vs):
            for i, u in enumerate(us):
                # camera ray -> body (forward, right, up), unit length
                f_b = math.cos(th) - v * math.sin(th)
                u_b = -(v * math.cos(th) + math.sin(th))
                r_b = u
                n = math.sqrt(f_b * f_b + r_b * r_b + u_b * u_b)
                f_b, r_b, u_b = f_b / n, r_b / n, u_b / n
                best = alt / -u_b if u_b < -1e-6 else np.inf          # ground
                hlen = math.hypot(f_b, r_b)
                if hlen > 1e-6:
                    d2 = (fwd * f_b + right * r_b) / hlen                # horizontal direction
                    hit = self._ray(d2)
                    if hit is not None:
                        best = min(best, hit / hlen)
                if best <= self.cfg.range_max * 1.5:
                    out[j, i] = best
        return out

    def telemetry(self) -> Telemetry:
        fwd, right = self._axes()
        return Telemetry(
            timestamp=self.t,
            position=tuple(self.pos),
            velocity_body=(float(self.vel @ fwd), float(self.vel @ right), float(self.vel[2])),
            yaw=self.yaw,
            altitude=float(self.pos[2]),
            battery=self.battery,
            ranges=self.ranges(),
            sonar=self.sonar() if self.cfg.sonar else None,
        )

    def distance_to_target(self) -> float:
        d = self.target_pos() - self.pos
        return float(math.hypot(d[0], d[1]))


# --- port adapters over the mock world ----------------------------------------
def _point_segment_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    u = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + u * ab)))


def _ray_segment(o: np.ndarray, d: np.ndarray, a: np.ndarray, b: np.ndarray, max_t: float):
    """Closest approach between rays o + t d (d: (n, 3) unit, 0 <= t <= max_t) and
    segment ab. Returns (t along each ray, distance), both shape (n,)."""
    e = b - a
    w = o - a
    B, C = d @ e, float(e @ e)
    D, E = d @ w, float(e @ w)
    den = C - B * B
    t = np.where(den > 1e-12, (B * E - C * D) / np.where(den > 1e-12, den, 1.0), 0.0)
    t = np.clip(t, 0.0, max_t)
    u = np.clip((B * t + E) / C, 0.0, 1.0) if C > 1e-12 else np.zeros_like(t)
    t = np.clip(B * u - D, 0.0, max_t)          # re-project onto the ray
    p = o + t[:, None] * d - (a + u[:, None] * e)
    return t, np.linalg.norm(p, axis=1)


class MockCamera:
    def __init__(self, world: MockWorld):
        self.world = world
        self._delay: list[Frame] = []

    def read(self) -> Optional[Frame]:
        self.world.step(1.0 / self.world.cfg.fps)
        self.world.frame_id += 1
        self._delay.append(Frame(self.world.render(), self.world.t, self.world.frame_id))
        if len(self._delay) > self.world.cfg.latency_frames + 1:
            self._delay.pop(0)
        old = self._delay[0]   # delayed picture, but always a fresh frame id
        return Frame(old.image, old.timestamp, self.world.frame_id)

    def close(self) -> None:
        pass


class MockMultiCamera:
    """All cameras of a rig, rendered from the same instant. Optionally
    includes ground-truth depth, like the Unity sim."""

    def __init__(self, world: MockWorld, mounts=None, with_depth: bool = False):
        from drone_core.domain.models import FOUR_CAMERA_RIG
        self.world = world
        self.mounts = tuple(mounts or FOUR_CAMERA_RIG)
        self.with_depth = with_depth

    def read_all(self) -> Optional[MultiFrame]:
        w = self.world
        w.step(1.0 / w.cfg.fps)
        w.frame_id += 1
        frames = {m.name: Frame(w.render(m.yaw_deg), w.t, w.frame_id, m.name) for m in self.mounts}
        depth = {m.name: w.depth(m.yaw_deg) for m in self.mounts} if self.with_depth else {}
        return MultiFrame(frames, w.t, w.frame_id, depth)

    def close(self) -> None:
        pass


class MockCommander:
    def __init__(self, world: MockWorld):
        self.world = world
        self.history: list[VelocityCommand] = []

    def send_velocity(self, cmd: VelocityCommand) -> None:
        self.world.cmd = cmd
        self.history.append(cmd)

    def hover(self) -> None:
        self.send_velocity(VelocityCommand.hover())

    def close(self) -> None:
        self.hover()


class MockTelemetry:
    def __init__(self, world: MockWorld):
        self.world = world

    def latest(self) -> Optional[Telemetry]:
        return self.world.telemetry()


class MockClock:
    def __init__(self, world: MockWorld):
        self.world = world

    def now(self) -> float:
        return self.world.t


class ColorBlobTracker:
    """ObjectTracker that finds the red target by colour. Lets you run the
    full loop with no neural net (and in CI). Also works on the Unity feed if
    you give the target a pure red material."""

    def __init__(self, class_name: str = "person", min_area: int = 30):
        self.class_name = class_name
        self.min_area = min_area

    def update(self, frame: Frame) -> list[Track]:
        hsv = cv2.cvtColor(frame.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 120, 80), (10, 255, 255)) | cv2.inRange(hsv, (170, 120, 80), (180, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        # Roughly person-shaped blobs only (not much wider than tall): thin red lines, roofs or
        # other red stuff in the scene shouldn't steal the lock.
        best = None
        for c in contours:
            area = cv2.contourArea(c)
            x, y, w, h = cv2.boundingRect(c)
            if area >= self.min_area and h >= 0.4 * w and area >= 0.3 * w * h and (best is None or area > best[0]):
                best = (area, x, y, w, h)
        if best is None:
            return []
        _, x, y, w, h = best
        return [Track(1, BoundingBox(x, y, x + w, y + h), self.class_name, 0.9)]


def _overlap(a: BoundingBox, b: BoundingBox) -> float:
    w = min(a.x2, b.x2) - max(a.x1, b.x1)
    h = min(a.y2, b.y2) - max(a.y1, b.y1)
    return max(0.0, w) * max(0.0, h)


def _iou(a: BoundingBox, b: BoundingBox) -> float:
    o = _overlap(a, b)
    return o / max(a.area + b.area - o, 1e-6)


class MockPersonTracker:
    """A person detector + tracker for the mock world that behaves like YOLO +
    ByteTrack: it sees everyone (target and decoys), keeps an id while a person
    stays visible, gives a NEW id after they were hidden longer than
    buffer_frames, and may swap two ids when people cross (swap_prob).
    gt[track_id] = person index (0 = the real target), for scoring runs."""

    def __init__(self, world: "MockWorld", buffer_frames: int = 30, swap_prob: float = 0.0,
                 mount_yaw_deg: float = 0.0, seed: int = 3, class_name: str = "person"):
        self.world, self.buffer, self.swap_prob = world, buffer_frames, swap_prob
        self.mount = mount_yaw_deg
        self.class_name = class_name
        self._rng = np.random.default_rng(seed)
        self._ids: dict[int, int] = {}          # person index -> current track id
        self._unseen: dict[int, int] = {}
        self._next = 1
        self._touching: set = set()             # pairs of person indices overlapping
        self.gt: dict[int, int] = {}
        self.swaps = 0

    def update(self, frame: Frame) -> list[Track]:
        vis = self.world.visible_people(self.mount)
        seen = {i for i, _, _ in vis}
        for i in list(self._ids):
            if i not in seen:
                self._unseen[i] = self._unseen.get(i, 0) + 1
                if self._unseen[i] > self.buffer:
                    del self._ids[i]             # track dropped: comes back with a new id
        for i in seen:
            self._unseen[i] = 0
            if i not in self._ids:
                self._ids[i] = self._next
                self._next += 1
        boxes = {i: b for i, b, _ in vis}
        now = set()
        idx = sorted(boxes)
        for a_ in range(len(idx)):
            for b_ in range(a_ + 1, len(idx)):
                i, j = idx[a_], idx[b_]
                if _iou(boxes[i], boxes[j]) > 0.15 or _overlap(boxes[i], boxes[j]) > 0.5 * min(boxes[i].area, boxes[j].area):
                    now.add((i, j))
        for pair in self._touching - now:          # these two just separated
            i, j = pair
            if i in self._ids and j in self._ids and self._rng.random() < self.swap_prob:
                self._ids[i], self._ids[j] = self._ids[j], self._ids[i]
                self.swaps += 1
        self._touching = now
        self.gt = {tid: i for i, tid in self._ids.items()}
        return [Track(self._ids[i], b, self.class_name, 0.9) for i, b, _ in vis]


class NoisyTracker:
    """Wraps any ObjectTracker and degrades it the way a real detector
    misbehaves: jittery boxes, missed frames, and occasional id switches."""

    def __init__(self, inner, jitter_frac: float = 0.03, drop_prob: float = 0.15,
                 id_switch_prob: float = 0.0, burst_len: int = 1, seed: int = 1):
        self.inner = inner
        self.jitter_frac, self.drop_prob = jitter_frac, drop_prob
        self.id_switch_prob, self.burst_len = id_switch_prob, burst_len
        self._rng = np.random.default_rng(seed)
        self._drop_left = 0
        self._id_offset = 0

    def update(self, frame: Frame) -> list[Track]:
        tracks = self.inner.update(frame)
        if self._drop_left > 0:
            self._drop_left -= 1
            return []
        if self._rng.random() < self.drop_prob:
            self._drop_left = self.burst_len - 1
            return []
        if self._rng.random() < self.id_switch_prob:
            self._id_offset += 100
        out = []
        for t in tracks:
            b = t.box
            jx, jy, jw, jh = self._rng.normal(0, self.jitter_frac, 4) * np.array([b.w, b.h, b.w, b.h])
            cx, cy, w, h = b.cx + jx, b.cy + jy, max(2.0, b.w + jw), max(2.0, b.h + jh)
            out.append(Track(t.track_id + self._id_offset,
                             BoundingBox(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2),
                             t.class_name, t.confidence))
        return out
