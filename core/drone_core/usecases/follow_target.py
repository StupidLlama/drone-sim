from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from drone_core.domain.models import BoundingBox, FollowStatus, Telemetry, Track, VelocityCommand
from drone_core.usecases.pid import PID
from drone_core.usecases.target_filter import TargetFilter
from drone_core.usecases.target_geo import (CameraGeometry, TargetWorldTrack, box_to_world, ground_range,
                                           rig_geometry)
from drone_core.domain.models import FOUR_CAMERA_RIG
from drone_core.usecases.target_selection import TargetSelector
from drone_core.usecases.target_belief import BeliefConfig, TargetBelief
from drone_core.usecases.viewpoints import ViewpointConfig, ViewpointPlanner
from drone_core.usecases.obstacle_type import ObstacleClassifier, Scene, SceneConfig
from drone_core.usecases.scan_routes import RouteConfig, RouteRunner, build_route


@dataclass
class FollowConfig:
    # Framing goals (fractions of the image)
    desired_box_height: float = 0.40   # target box height / image height  -> sets distance
    desired_cx: float = 0.5            # horizontal position of box centre
    desired_cy: float = 0.5            # vertical position of box centre

    # Gains. Inputs are normalised errors, outputs are m/s or rad/s.
    yaw: PID = field(default_factory=lambda: PID(kp=1.8, ki=0.3, kd=0.15, out_limit=1.2, i_limit=1.5))
    vertical: PID = field(default_factory=lambda: PID(kp=1.5, ki=0.05, kd=0.1, out_limit=1.0, i_limit=0.5, deadband=0.03))
    distance: PID = field(default_factory=lambda: PID(kp=2.5, ki=0.6, kd=0.3, out_limit=4.0, i_limit=6.0, deadband=0.03, i_zone=0.8))

    lateral_speed: float = 0.0          # m/s, non-zero = orbit the target while following

    # With telemetry the follow distance is held in metres (ground distance from
    # altitude + where the box sits), not from box height alone: box height
    # shrinks when looking down at someone, which used to pull the drone over
    # the top of them. None = the distance desired_box_height gives for a
    # 1.8 m person, whatever part of the person the tracker boxes.
    follow_distance: Optional[float] = None
    min_separation: float = 0.6         # fraction of follow distance: closer than this = brake and back off
    backoff_speed: float = 1.5          # m/s when braking

    # Limits (applied after the PIDs)
    max_xy: float = 4.0
    max_z: float = 1.0
    max_yaw: float = 1.2

    # Camera rig. "front" drives the visual servo; the others are used to keep
    # track of the target when it leaves the front view (turn to face it).
    rig: dict = field(default_factory=lambda: rig_geometry(FOUR_CAMERA_RIG))
    turn_kp: float = 2.0                # rad/s per rad of bearing error while TURNING

    # Loss handling
    reacquire_s: float = 8.0            # fly toward where the target should be (needs telemetry)
    reacquire_speed: float = 3.0        # m/s
    reacquire_climb: float = 2.0        # m above the altitude we lost it at: see over low walls
    reacquire_standoff: float = 3.0     # m: stop this far from the predicted spot
    lost_hover_s: float = 1.0           # without telemetry: hover this long after losing target
    search_s: float = 8.0               # without telemetry: rotate toward last-seen side this long
    search_yaw_rate: float = 0.5        # rad/s
    # With telemetry, after REACQUIRING: circle the predicted spot (SEARCHING),
    # then go back to where it was last seen and wait (LOITER), then give up
    # (GAVE_UP: the supervisor flies home).
    orbit_s: float = 20.0               # s of circling
    orbit_radius: float = 4.0           # m, first lap
    orbit_growth: float = 2.0           # m added per lap
    orbit_speed: float = 2.5            # m/s along the circle
    loiter_s: float = 60.0              # s hovering at the last-seen spot
    # Re-lock only after the candidate has been seen this long near the
    # predicted spot (after a real loss, not a one-frame dropout).
    confirm_s: float = 0.5
    # Climb when the view of the person is (about to be) blocked. Needs a
    # line-of-sight test (FollowTarget.line_of_sight, set from the 3-D map).
    los_max_altitude: float = 10.0      # m, never climb above this for a better view
    los_climb_speed: float = 1.0        # m/s
    los_hold_s: float = 3.0             # keep the height this long after the view is clear
    los_lookahead_s: float = 1.0        # also check where the person will be in this many s
    min_altitude: float = 1.5
    max_altitude: float = 15.0
    # v2: where the hidden target is = a particle cloud (TargetBelief), not one
    # extrapolated point. False = the v1 behaviour, for comparison.
    use_belief: bool = True
    belief: BeliefConfig = field(default_factory=BeliefConfig)
    viewpoints: ViewpointConfig = field(default_factory=ViewpointConfig)
    # Re-lock gate with a belief: a candidate passes if this share of the
    # belief is within belief_gate_r of it (or it is near a top cluster).
    belief_gate_mass: float = 0.004
    belief_gate_r: float = 1.5
    # v3: work out what they vanished behind (wall / hall / room) and fly that
    # scene's scan route once sure enough (SceneConfig.commit, 80%). Needs the map.
    use_scenes: bool = True
    scenes: SceneConfig = field(default_factory=SceneConfig)
    routes: RouteConfig = field(default_factory=RouteConfig)
    route_bias: float = 2.0             # the v2 search overrules the route only when this much better


    @property
    def camera(self) -> CameraGeometry:
        return self.rig["front"]


class FollowTarget:
    """Visual-servo follow: one PID per axis, driven by where the tracked box
    sits in the image.

        yaw      <- horizontal offset of the box centre
        up/down  <- vertical offset of the box centre
        forward  <- log(desired_height / box_height)  (≈ linear in distance,
                    because box height ∝ 1 / distance)
    """

    def __init__(self, selector: TargetSelector, cfg: FollowConfig | None = None,
                 target_filter: TargetFilter | None = None):
        self.selector = selector
        self.cfg = cfg or FollowConfig()
        self.filter = target_filter or TargetFilter()
        self._last_seen_t: Optional[float] = None
        self._last_seen_side: float = 1.0
        self._last_t: Optional[float] = None
        self.world = TargetWorldTrack()
        self._lost_alt: Optional[float] = None
        self._last_front_t: Optional[float] = None
        self._lost_pos: Optional[tuple] = None      # target ground position when it was lost
        self._orbit: Optional[dict] = None          # circle-search state
        self._cand: Optional[tuple] = None          # (key, first seen t, last seen t) re-lock candidate
        self._los_until = -1.0                      # hold the climbed height until then
        # (drone xyz, point xyz) -> True if an obstacle is in between. Set by
        # FollowMission from the 3-D map; None = no line-of-sight climbing.
        self.line_of_sight = None
        # Optional appearance memory (TargetAppearance): re-lock only on someone
        # who looks like the target, and catch tracker id swaps.
        self.appearance = None
        self.swaps_caught = 0
        self._unban: list = []
        # v2: particle belief over where the hidden target is, the 3-D map it
        # uses (walls, line of sight; set by FollowMission, None = no map), and
        # the search that picks where to look from.
        self.belief = TargetBelief(self.cfg.belief)
        self.obstacle_map = None
        self.planner = ViewpointPlanner(self.cfg.viewpoints)
        self._cams_seen = {"front"}
        self._people_xy: list = []                  # ground (x, y) of everyone detected this frame
        # v3: scene classifier + scan route
        self.classifier = ObstacleClassifier(self.cfg.scenes)
        self.runner = RouteRunner()
        self.scene: Optional[Scene] = None          # latest classification while lost
        self.route_kind: Optional[str] = None       # scene whose route is being flown
        self._scene_t = -1e9
        self._lost_vel: Optional[tuple] = None
        self._route_check_t = -1e9
        self._route_overruled = False

    def reset(self) -> None:
        if self.appearance is not None:
            self.appearance.reset()
        for p in (self.cfg.yaw, self.cfg.vertical, self.cfg.distance):
            p.reset()
        self.selector.reset()
        self.filter.reset()
        self.world.reset()
        self._last_seen_t = None
        self._last_front_t = None
        self._last_t = None
        self._lost_pos = None
        self._orbit = None
        self._cand = None
        self._los_until = -1.0
        self.belief.reset()
        self.planner.reset()
        self._clear_scene()

    def step(self, tracks: list[Track], frame_w: int, frame_h: int, t: float,
             telem: Optional[Telemetry] = None,
             other_tracks: Optional[dict] = None) -> FollowStatus:
        """tracks: front-camera tracks. other_tracks: {camera name: tracks}
        from the rest of the rig (same frame size), if the drone has them."""
        c = self.cfg
        dt = 0.05 if self._last_t is None else max(1e-3, t - self._last_t)
        self._last_t = t
        self._dt = dt
        if other_tracks:
            self._cams_seen.update(other_tracks.keys())
        self._people_xy = self._detected_people(tracks, other_tracks, frame_w, frame_h, telem)

        target = self.selector.select(tracks, frame_w, frame_h, self._looks_gate(self._gate(frame_w, frame_h, t, telem)),
                                      accept=self._looks_ok if self.appearance is not None else None)
        if target is not None and not self._confirmed(("front", target.track_id), t):
            self.selector.locked_id = None      # not sure yet: don't commit the lock
            target = None
        est = self.filter.update(target.box if target else None, frame_w, frame_h, t)
        if target is not None and telem is not None:
            p = box_to_world(target.box, frame_w, frame_h, telem, c.camera)
            if p is not None:
                self.world.update(p, t)
        if est is None:
            if other_tracks:
                turning = self._follow_in_other_camera(other_tracks, frame_w, frame_h, t, telem)
                if turning is not None:
                    return turning
            return self._handle_lost(t, telem)

        # est is the Kalman-smoothed box. With no detection this frame it is a
        # short prediction (COASTING), so one missed frame doesn't stop the drone.
        if target is not None:
            self._last_seen_t = t
            self._last_front_t = t
            self._lost_alt = None
            self._lost_pos = None
            self._orbit = None
            self.belief.reset()
            self.planner.reset()
            self._clear_scene()
        ex = est.cx - c.desired_cx                           # + = target right of centre
        ey = c.desired_cy - est.cy                           # + = target above centre
        ed = math.log(c.desired_box_height / max(est.h, 1e-3))  # + = too far
        gdist = want = None
        if telem is not None:
            hw = est.h * frame_h * 0.25
            ebox = BoundingBox(est.cx * frame_w - hw, (est.cy - est.h / 2) * frame_h,
                               est.cx * frame_w + hw, (est.cy + est.h / 2) * frame_h)
            r = ground_range(ebox, frame_w, frame_h, telem, c.camera)
            if r is not None:
                gdist, cut = r
                if target is not None:
                    cut = ground_range(target.box, frame_w, frame_h, telem, c.camera)[1]
                want = self.follow_distance(frame_w, frame_h)
                ed = math.log(max(gdist, 0.1) / want)
                if cut:
                    ed = min(ed, 0.0)    # box cut off by the frame edge: we're close, never close in
        # Search toward where the target was heading, else the side it was on.
        if abs(est.rate_cx) > 0.05:
            self._last_seen_side = 1.0 if est.rate_cx > 0 else -1.0
        elif abs(ex) > 0.05:
            self._last_seen_side = 1.0 if ex > 0 else -1.0

        cmd = VelocityCommand(
            forward=c.distance.update(ed, dt),
            right=c.lateral_speed,
            up=c.vertical.update(ey, dt),
            yaw_rate=c.yaw.update(ex, dt),
        )
        errors = {"ex": ex, "ey": ey, "ed": ed}
        if self.appearance is not None and self.appearance.last_score is not None:
            errors["look"] = round(self.appearance.last_score, 2)
        if gdist is not None:
            errors["dist_m"] = round(gdist, 2)
            if gdist < c.min_separation * want:
                # Too close (it walked back at us, or we overshot): back off now
                # instead of waiting for the PID, and don't climb over it.
                c.distance.reset()
                cmd = VelocityCommand(min(cmd.forward, -c.backoff_speed), cmd.right, min(cmd.up, 0.0), cmd.yaw_rate)
                errors["brake"] = True
        cmd = self._view_climb(cmd, telem, t, errors)
        cmd = self._altitude_guard(cmd, telem).clamped(c.max_xy, c.max_z, c.max_yaw)
        state = "TRACKING" if target is not None else "COASTING"
        return FollowStatus(state, target, cmd, errors)

    def follow_distance(self, frame_w: int, frame_h: int) -> float:
        """Wanted ground distance to the target, metres."""
        c = self.cfg
        if c.follow_distance is not None:
            return c.follow_distance
        return c.camera.focal_px(frame_w) * 1.8 / (c.desired_box_height * frame_h)

    def estimate(self, t: float, state: str = "") -> Optional[dict]:
        """Where we think the target is (world x, y) and how sure: r is the
        radius we'd accept a re-lock in. None = no idea."""
        if self.world.pos is None:
            return None
        cam = self.cfg.camera
        lost = self._last_seen_t is not None and t - self._last_seen_t > self.filter.cfg.coast_s
        modes = None
        if lost and self.belief.active:
            cl = self.belief.clusters(3)
            main = self._main_cluster(cl)
            p = (main.x, main.y)
            r = max(1.0, 2.0 * main.spread)
            modes = [{"x": round(k.x, 2), "y": round(k.y, 2), "w": round(k.w, 3), "mode": k.mode} for k in cl]
        elif lost:
            p = self.world.predict(t, horizon_cap=self.cfg.reacquire_s) or self.world.pos
            r = self.selector.cfg.reacquire_gate_m + 0.5 * (t - self._last_seen_t)
        else:
            p, r = self.world.pos, 0.5
        out = {"x": float(p[0]), "y": float(p[1]), "h": cam.target_bottom_m + cam.target_height_m,
               "r": float(r), "state": state.split(" ")[0]}
        if modes is not None:
            out["modes"] = modes
        if lost:
            ri = self.route_info()
            if ri is not None:
                out["route"] = ri
        return out

    def belief_points(self, m: int = 150):
        """Some particles (x, y, mode) for drawing the cloud, or None."""
        if not self.belief.active:
            return None
        x, y, mode = self.belief.sample(m)
        return np.stack([x, y, mode], -1)

    def _main_cluster(self, cl):
        """The ghost = the heaviest cluster."""
        return cl[0]

    def _detected_people(self, tracks, other_tracks, frame_w, frame_h, telem) -> list:
        if telem is None:
            return []
        out = []
        sc = self.selector.cfg
        for name, trs in [("front", tracks)] + list((other_tracks or {}).items()):
            cam = self.cfg.rig.get(name)
            if cam is None:
                continue
            for tr in trs:
                if tr.class_name != sc.target_class:
                    continue
                p = box_to_world(tr.box, frame_w, frame_h, telem, cam)
                if p is not None:
                    out.append(p)
        return out

    # --- appearance ------------------------------------------------------------
    def observe_appearance(self, image, tracks: list[Track], frame_w: int, frame_h: int, t: float,
                           telem: Optional[Telemetry] = None) -> None:
        """Called by FollowMission with the front image before step()."""
        if self.appearance is None:
            return
        while self._unban and self._unban[0][0] <= t:
            self.selector.banned.discard(self._unban.pop(0)[1])
        lid = self.selector.locked_id
        relocking = lid is None or all(tr.track_id != lid for tr in tracks)
        near = None
        if relocking:
            gate = self._gate(frame_w, frame_h, t, telem)
            if gate is not None:
                near = gate
            elif self.selector.last_box is not None:
                lb = self.selector.last_box
                near = lambda tr: math.hypot(tr.box.cx - lb.cx, tr.box.cy - lb.cy)
        if self.appearance.observe(image, tracks, lid, t, frame_w, frame_h, relocking, near) == "swap":
            # The tracker handed our id to someone else: stop following that id.
            # Ban the id only briefly: trackers often swap the ids back later.
            self.selector.banned.add(lid)
            self._unban.append((t + 1.5, lid))
            self.selector.locked_id = None
            self.swaps_caught += 1

    def _looks_ok(self, track: Track) -> bool:
        v = self.appearance.verdict(track.track_id)
        return v is None or v >= 0.0

    def _looks_gate(self, gate):
        if gate is None or self.appearance is None:
            return gate
        def g(track):
            d = gate(track)
            v = self.appearance.verdict(track.track_id)
            if v is not None and v < 0.0:
                return None                                   # doesn't look like the target
            if d is not None and v is not None and v >= 0.05:
                d = max(0.0, d - self.appearance.cfg.gate_bonus_m)   # clearly them: trust it further out
            return d
        return g

    # --- re-lock confirmation --------------------------------------------------
    def _lost_long(self, t: float) -> bool:
        """True after a real loss (longer than the filter's coasting)."""
        return self._last_seen_t is not None and t - self._last_seen_t > self.filter.cfg.coast_s

    def _confirmed(self, key, t: float) -> bool:
        """After a real loss, a candidate must keep showing up near the predicted
        spot for confirm_s before we lock on: one-frame look-alikes don't win."""
        if not self._lost_long(t):
            self._cand = None
            return True
        if self._cand is None or self._cand[0] != key or t - self._cand[2] > 0.3:
            self._cand = (key, t, t)
        else:
            self._cand = (key, self._cand[1], t)
        if t - self._cand[1] >= self.cfg.confirm_s:
            self._cand = None
            return True
        return False

    # --- climb for a better view ----------------------------------------------
    def _view_climb(self, cmd: VelocityCommand, telem: Optional[Telemetry], t: float, errors: dict) -> VelocityCommand:
        c = self.cfg
        if self.line_of_sight is None or telem is None or self.world.pos is None:
            return cmd
        px, py = telem.position[0], telem.position[1]
        eye = (px, py, telem.position[2])
        # top of what the tracker sees, a little below (the head for YOLO, shoulders for the colour shirt)
        head_h = c.camera.target_bottom_m + c.camera.target_height_m - 0.18
        ahead = self.world.predict(t + c.los_lookahead_s, horizon_cap=c.los_lookahead_s + 0.5) or self.world.pos
        blocked = any(self.line_of_sight(eye, (q[0], q[1], head_h)) for q in (self.world.pos, ahead))
        # Don't climb so high the person drops out of the bottom of the frame.
        dist = math.hypot(self.world.pos[0] - px, self.world.pos[1] - py)
        cam = c.camera
        vfov_half = math.degrees(math.atan(math.tan(math.radians(cam.hfov_deg) / 2) * 0.75))
        steepest = math.radians(min(80.0, cam.pitch_deg + vfov_half - 8.0))
        ceiling = min(c.los_max_altitude, c.max_altitude, head_h + dist * math.tan(steepest))
        if blocked:
            self._los_until = t + c.los_hold_s
            if telem.altitude < ceiling:
                c.vertical.reset()
                errors["view"] = "blocked: climbing"
                return VelocityCommand(cmd.forward, cmd.right, c.los_climb_speed, cmd.yaw_rate)
            errors["view"] = "blocked: at height limit"
        elif t < self._los_until and cmd.up < 0:
            errors["view"] = "holding height"
            return VelocityCommand(cmd.forward, cmd.right, 0.0, cmd.yaw_rate)
        return cmd

    def _gate(self, frame_w, frame_h, t, telem):
        """While the target is hidden, judge candidates by predicted ground
        position instead of image position (the drone has moved since)."""
        if telem is None or self._last_front_t is None or self.world.pos is None:
            return None
        if t - self._last_front_t < self.filter.cfg.coast_s:
            return None          # brief dropout: the image-space check is better
        return self._world_gate(frame_w, frame_h, t, telem, self.cfg.camera)

    def _world_gate(self, frame_w, frame_h, t, telem, cam: CameraGeometry):
        if self.belief.active:
            return self._belief_gate(frame_w, frame_h, telem, cam)
        pred = self.world.predict(t, horizon_cap=self.cfg.reacquire_s)
        gate_slack = 0.5 * (t - self._last_seen_t)   # uncertainty grows while hidden
        def gate(track):
            p = box_to_world(track.box, frame_w, frame_h, telem, cam)
            return None if p is None else max(0.0, math.hypot(p[0] - pred[0], p[1] - pred[1]) - gate_slack)
        return gate

    def _belief_gate(self, frame_w, frame_h, telem, cam: CameraGeometry):
        """Re-lock gate from the particle cloud: 0 m where the belief is dense,
        growing past the selector's limit where it is empty. A stopped person
        where they vanished passes even when most of the cloud walked on."""
        c = self.cfg
        gate_m = self.selector.cfg.reacquire_gate_m
        cl = self.belief.clusters(3)
        def gate(track):
            p = box_to_world(track.box, frame_w, frame_h, telem, cam)
            if p is None:
                return None
            dens = self.belief.density(p[0], p[1], c.belief_gate_r)
            if dens >= c.belief_gate_mass:
                d = 0.0
            else:
                d = gate_m * (1.0 + math.log(c.belief_gate_mass / max(dens, 1e-9)) / math.log(10.0))
            near = min((math.hypot(p[0] - k.x, p[1] - k.y) - (1.0 + k.spread) for k in cl), default=1e9)
            return max(0.0, min(d, near))
        return gate

    def _follow_in_other_camera(self, other_tracks, frame_w, frame_h, t, telem) -> Optional[FollowStatus]:
        """Target not in the front camera but visible in a side/rear one: turn
        to face it while holding the follow distance (TURNING)."""
        c = self.cfg
        sc = self.selector.cfg
        best = None   # (score, track, cam)
        have_history = self._last_seen_t is not None and self.world.pos is not None and telem is not None
        for name, tracks in other_tracks.items():
            cam = c.rig.get(name)
            if cam is None:
                continue
            cands = [tr for tr in tracks if tr.class_name == sc.target_class and tr.confidence >= sc.min_confidence]
            if not cands:
                continue
            if have_history:
                gate = self._world_gate(frame_w, frame_h, t, telem, cam)
                for tr in cands:
                    d = gate(tr)
                    if d is not None and d <= sc.reacquire_gate_m and (best is None or d < best[0]):
                        best = (d, tr, cam)
            elif self._last_seen_t is None:
                # never had a target: take the biggest person in any camera
                for tr in cands:
                    score = -tr.box.area
                    if best is None or score < best[0]:
                        best = (score, tr, cam)
        if best is None:
            return None
        _, tr, cam = best
        if have_history and not self._confirmed((tr.camera, tr.track_id), t):
            return FollowStatus("CONFIRMING", None, VelocityCommand.hover())
        self._last_seen_t = t
        self._lost_alt = None
        self._lost_pos = None
        self._orbit = None
        self.belief.reset()
        self.planner.reset()
        self._clear_scene()
        for p in (c.yaw, c.vertical, c.distance):
            p.reset()
        rel = cam.bearing(tr.box, frame_w)
        rel = (rel + math.pi) % (2 * math.pi) - math.pi
        self._last_seen_side = 1.0 if rel >= 0 else -1.0
        fwd = right = 0.0
        if telem is not None:
            p = box_to_world(tr.box, frame_w, frame_h, telem, cam)
            if p is not None:
                self.world.update(p, t)
                dist = math.hypot(p[0] - telem.position[0], p[1] - telem.position[1])
                want = self.follow_distance(frame_w, frame_h)
                v = max(-c.max_xy, min(c.max_xy, 0.8 * (dist - want)))
                fwd, right = v * math.cos(rel), v * math.sin(rel)
        cmd = VelocityCommand(fwd, right, 0.0, max(-c.max_yaw, min(c.max_yaw, c.turn_kp * rel)))
        cmd = cmd.clamped(c.max_xy, c.max_z, c.max_yaw)
        return FollowStatus(f"TURNING ({tr.camera})", tr, cmd, {"bearing_deg": math.degrees(rel)})

    def _handle_lost(self, t: float, telem: Optional[Telemetry] = None) -> FollowStatus:
        c = self.cfg
        for p in (c.yaw, c.vertical, c.distance):
            p.reset()
        if self._last_seen_t is None:
            return FollowStatus("IDLE", None, VelocityCommand.hover())
        lost_for = t - self._last_seen_t

        pred = self.world.predict(t, horizon_cap=c.reacquire_s)
        if self._lost_pos is None and self.world.pos is not None:
            self._lost_pos = self.world.pos
            self._lost_vel = self.world.vel
        if telem is not None and pred is not None and c.use_belief:
            self._update_belief(t, telem)
            # With a map: fly wherever sees the most of the belief. Without one
            # we can't tell what a viewpoint would really see (pillars, walls),
            # so keep v1's flight and use the belief only for the ghost and the
            # re-lock gate (a person who stopped still gets picked up).
            if self.obstacle_map is not None and self.belief.clusters(1):
                if c.use_scenes and lost_for < c.reacquire_s + c.orbit_s:
                    st = self._scan_route(telem, t, lost_for)
                    if st is not None:
                        return st
                if lost_for < c.reacquire_s:
                    return self._search_viewpoint(telem, t, lost_for, "REACQUIRING")
                if lost_for < c.reacquire_s + c.orbit_s:
                    return self._search_viewpoint(telem, t, lost_for, "SEARCHING")
        if telem is not None and pred is not None:
            if lost_for < c.reacquire_s:
                return self._reacquire(pred, telem, lost_for)
            if lost_for < c.reacquire_s + c.orbit_s:
                return self._orbit_search(pred, telem, t, lost_for)
            if lost_for < c.reacquire_s + c.orbit_s + c.loiter_s and self._lost_pos is not None:
                cmd = self._goto(self._lost_pos, telem, standoff=0.0, face=self._lost_pos)
                return FollowStatus("LOITER", None, cmd, {"lost_s": lost_for})
            # Give up. The supervisor sees GAVE_UP and flies home.
            self._forget()
            return FollowStatus("GAVE_UP", None, VelocityCommand.hover())

        # No position estimate (no telemetry): hover, then spin to look.
        search_start = c.reacquire_s if pred is not None and telem is not None else c.lost_hover_s
        if lost_for < c.lost_hover_s and search_start == c.lost_hover_s:
            return FollowStatus("LOST", None, VelocityCommand.hover())
        if lost_for < search_start + c.search_s:
            cmd = VelocityCommand(yaw_rate=self._last_seen_side * c.search_yaw_rate)
            return FollowStatus("SEARCHING", None, cmd)
        # Give up: forget the old target so any new person can be acquired.
        self._forget()
        return FollowStatus("IDLE", None, VelocityCommand.hover())

    # --- v2: belief + viewpoint search --------------------------------------------
    def _cams(self) -> list:
        return [self.cfg.rig[n] for n in sorted(self._cams_seen) if n in self.cfg.rig]

    def _update_belief(self, t: float, telem: Telemetry) -> None:
        b = self.belief
        if not b.active:
            if self.world.pos is None:
                return
            b.seed(self.world.pos, self.world.vel, self.world.t if self.world.t is not None else t)
        b.set_map(self.obstacle_map, t)
        b.predict(t)
        if self.obstacle_map is not None:
            # Negative information: what the cameras can clearly see and is
            # empty of the target. Needs the map: without it we can't tell
            # "looked and nobody there" from "looked at the back of a pillar".
            b.observe_empty(tuple(telem.position), telem.yaw, self._cams(), self.obstacle_map,
                            self._people_xy, getattr(self, "_dt", 0.05))

    def _search_viewpoint(self, telem: Telemetry, t: float, lost_for: float, state: str) -> FollowStatus:
        """Fly to the spot that sees the most of the belief, nose at its cluster."""
        c = self.cfg
        base = self._search_alt(telem)
        vp = self.planner.plan(t, tuple(telem.position), telem.yaw, base, self.belief, self._cams(),
                               self.obstacle_map)
        if vp is None:
            main = self.belief.clusters(1)[0]
            return self._reacquire((main.x, main.y), telem, lost_for)
        dx, dy = vp.x - telem.position[0], vp.y - telem.position[1]
        dist = math.hypot(dx, dy)
        v = min(c.reacquire_speed, 0.8 * dist)
        vx, vy = (v * dx / dist, v * dy / dist) if dist > 1e-6 else (0.0, 0.0)
        cmd = self._world_cmd(vx, vy, vp.face[0], vp.face[1], telem, alt=vp.z)
        main = self.belief.clusters(1)[0]
        return FollowStatus(state, None, cmd, {"lost_s": lost_for, "vp_score": round(vp.score, 3),
                                               "vp_z": round(vp.z, 1), "ghost_mode": main.mode,
                                               "ghost_w": round(main.w, 2)})

    # --- v3: scene + scan route ---------------------------------------------------
    def _clear_scene(self) -> None:
        self.scene = None
        self.route_kind = None
        self.runner.clear()
        self._scene_t = -1e9
        self._lost_vel = None
        self._route_check_t = -1e9
        self._route_overruled = False

    def _classify(self, telem: Telemetry, t: float) -> None:
        c = self.cfg
        if t - self._scene_t < c.scenes.every_s or self._lost_pos is None:
            return
        self._scene_t = t
        shares = self.belief.mode_shares() if hasattr(self.belief, "mode_shares") else {}
        main = self.belief.clusters(1)
        bxy = (main[0].x, main[0].y) if main else None
        vel = self._lost_vel
        heading = vel if vel is not None and math.hypot(vel[0], vel[1]) > 0.2 else (
            self._lost_pos[0] - telem.position[0], self._lost_pos[1] - telem.position[1])
        self.scene = self.classifier.classify(self.obstacle_map, self._lost_pos, heading,
                                              inside_share=float(shares.get("inside", 0.0)), belief_xy=bxy,
                                              drone_xy=tuple(telem.position[:2]),
                                              max_alt=c.viewpoints.max_alt)
        sc = self.scene
        if sc.kind is not None and sc.cert >= c.scenes.commit and sc.kind != self.route_kind:
            # sure enough (first time, or the map now says it's something else): fly its route
            route = build_route(sc, self._search_alt(telem), c.routes)
            if route:
                self.route_kind = sc.kind
                self.runner.start(route, loop=sc.kind == "room")
        elif self.route_kind is not None and sc.probs.get(self.route_kind, 0.0) < 0.4:
            # the map no longer backs that scene: back to the v2 search
            self.route_kind = None
            self.runner.clear()

    def _scan_route(self, telem: Telemetry, t: float, lost_for: float) -> Optional[FollowStatus]:
        """Fly the committed scene's scan route; None = no route (use v2's search)."""
        self._classify(telem, t)
        if self.route_kind is None:
            return None
        w = self.runner.current(tuple(telem.position), t)
        if w is None:
            return None                     # route finished: the v2 search takes over
        c = self.cfg
        # The route is the plan, but the v2 search may overrule it when it can
        # see clearly more of where they could be (route_bias x the score of the
        # route's next stop), e.g. the person is already coming back out.
        if t - self._route_check_t >= c.viewpoints.every_s:
            self._route_check_t = t
            vp = self.planner.plan(t, tuple(telem.position), telem.yaw, self._search_alt(telem), self.belief,
                                   self._cams(), self.obstacle_map, force=True)
            ws = self.planner.score_point(tuple(telem.position), w.x, w.y, w.z, (w.lx, w.ly), self.belief,
                                          self._cams(), self.obstacle_map)
            # A room's route is checking for other doors, which the belief doesn't
            # know about (it only waits at the door they used): never overrule it.
            bias = c.route_bias if self.route_kind != "room" else float("inf")
            self._route_overruled = vp is not None and vp.score > bias * ws
        if self._route_overruled:
            st = self._search_viewpoint(telem, t, lost_for, "REACQUIRING" if lost_for < c.reacquire_s else "SEARCHING")
            st.errors.update({"scene": self.route_kind, "wp": self.runner.i, "wp_label": "overruled"})
            return st
        dx, dy = w.x - telem.position[0], w.y - telem.position[1]
        dist = math.hypot(dx, dy)
        v = min(c.reacquire_speed, 0.8 * dist)
        vx, vy = (v * dx / dist, v * dy / dist) if dist > 1e-6 else (0.0, 0.0)
        cmd = self._world_cmd(vx, vy, w.lx, w.ly, telem, alt=w.z)
        state = "REACQUIRING" if lost_for < c.reacquire_s else "SEARCHING"
        return FollowStatus(state, None, cmd, {"lost_s": lost_for, "scene": self.route_kind,
                                               "cert": self.scene.cert if self.scene else 0.0,
                                               "wp": self.runner.i, "wp_label": w.label})

    def route_info(self) -> Optional[dict]:
        """The scene and route being flown, for Unity / the HUD. None = none."""
        sc = self.scene
        if sc is None:
            return None
        out = {"kind": sc.kind or "", "cert": round(float(sc.cert), 2), "active": int(self.route_kind is not None)}
        if self.route_kind is not None:
            out["kind"] = self.route_kind
            out["i"] = int(self.runner.i)
            out["pts"] = [[round(w.x, 2), round(w.y, 2), round(w.z, 2)] for w in self.runner.route]
        return out

    def _forget(self) -> None:
        self.belief.reset()
        self.planner.reset()
        self._clear_scene()
        if self.appearance is not None:
            self.appearance.reset()
        self.selector.reset()
        self.world.reset()
        self._last_seen_t = None
        self._last_front_t = None
        self._lost_alt = None
        self._lost_pos = None
        self._orbit = None
        self._cand = None

    def _search_alt(self, telem: Telemetry) -> float:
        if self._lost_alt is None:
            self._lost_alt = telem.altitude
        return min(self.cfg.max_altitude, self._lost_alt + self.cfg.reacquire_climb)

    def _goto(self, p, telem: Telemetry, standoff: float, face=None, speed: Optional[float] = None) -> VelocityCommand:
        """Fly to ground point p (world), nose toward `face` (default p)."""
        c = self.cfg
        dx, dy = p[0] - telem.position[0], p[1] - telem.position[1]
        dist = math.hypot(dx, dy)
        # negative = back away: from right above it the cameras can't see it
        v = max(-c.backoff_speed, min(speed or c.reacquire_speed, 0.8 * (dist - standoff)))
        vx, vy = (v * dx / dist, v * dy / dist) if dist > 1e-6 else (0.0, 0.0)
        fx, fy = face if face is not None else p
        return self._world_cmd(vx, vy, fx, fy, telem)

    def _world_cmd(self, vx: float, vy: float, fx: float, fy: float, telem: Telemetry,
                   alt: Optional[float] = None) -> VelocityCommand:
        """World-frame velocity + 'look at (fx, fy)' -> body command at search height."""
        c = self.cfg
        s_, c_ = math.sin(telem.yaw), math.cos(telem.yaw)
        fwd = vx * s_ + vy * c_
        right = vx * c_ - vy * s_
        look = math.atan2(fx - telem.position[0], fy - telem.position[1])
        yaw_err = (look - telem.yaw + math.pi) % (2 * math.pi) - math.pi
        goal = self._search_alt(telem) if alt is None else min(c.max_altitude, max(c.min_altitude, alt))
        up = max(-c.max_z, min(c.max_z, goal - telem.altitude))
        return VelocityCommand(fwd, right, up, max(-c.max_yaw, min(c.max_yaw, 1.5 * yaw_err))).clamped(
            c.max_xy, c.max_z, c.max_yaw)

    def _orbit_search(self, pred, telem: Telemetry, t: float, lost_for: float) -> FollowStatus:
        """Circle where the person should be by now, widening every lap, nose to
        the centre. With cameras all round, moving to new viewpoints is what
        finds someone hidden behind something, not spinning in place."""
        c = self.cfg
        px, py = telem.position[0], telem.position[1]
        if self._orbit is None:
            self._orbit = {"c": pred, "a": math.atan2(px - pred[0], py - pred[1]), "t": t, "turned": 0.0}
        o = self._orbit
        dt = max(0.0, t - o["t"])
        o["t"] = t
        r = c.orbit_radius + c.orbit_growth * (o["turned"] / (2 * math.pi))
        o["a"] += c.orbit_speed / r * dt
        o["turned"] += c.orbit_speed / r * dt
        cx, cy = o["c"]
        gx, gy = cx + r * math.sin(o["a"]), cy + r * math.cos(o["a"])
        # tangent (direction of travel) + pull onto the circle
        tx, ty = math.cos(o["a"]), -math.sin(o["a"])
        vx = c.orbit_speed * tx + 0.8 * (gx - px)
        vy = c.orbit_speed * ty + 0.8 * (gy - py)
        s = math.hypot(vx, vy)
        if s > c.max_xy:
            vx, vy = vx * c.max_xy / s, vy * c.max_xy / s
        cmd = self._world_cmd(vx, vy, cx, cy, telem)
        return FollowStatus("SEARCHING", None, cmd, {"lost_s": lost_for, "orbit_r": r,
                                                     "laps": o["turned"] / (2 * math.pi)})

    def _reacquire(self, pred, telem: Telemetry, lost_for: float) -> FollowStatus:
        """Target hidden (e.g. behind a wall): fly toward where it should be now,
        nose pointed at that spot, climbing a bit to see over obstacles. The
        avoidance filter steers us around the wall on the way."""
        c = self.cfg
        if self._lost_alt is None:
            self._lost_alt = telem.altitude
        dx, dy = pred[0] - telem.position[0], pred[1] - telem.position[1]
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dx, dy)
        yaw_err = (bearing - telem.yaw + math.pi) % (2 * math.pi) - math.pi
        self._last_seen_side = 1.0 if yaw_err >= 0 else -1.0
        speed = max(0.0, min(c.reacquire_speed, 0.8 * (dist - c.reacquire_standoff)))
        speed *= max(0.0, math.cos(yaw_err))          # turn toward it before charging
        alt_goal = min(c.max_altitude, self._lost_alt + c.reacquire_climb)
        cmd = VelocityCommand(
            forward=speed,
            up=max(-c.max_z, min(c.max_z, alt_goal - telem.altitude)),
            yaw_rate=max(-c.max_yaw, min(c.max_yaw, 1.5 * yaw_err)),
        )
        return FollowStatus("REACQUIRING", None, cmd,
                            {"pred_dist": dist, "lost_s": lost_for})

    def _altitude_guard(self, cmd: VelocityCommand, telem: Optional[Telemetry]) -> VelocityCommand:
        if telem is None:
            return cmd
        up = cmd.up
        if telem.altitude <= self.cfg.min_altitude and up < 0:
            up = 0.0
        if telem.altitude >= self.cfg.max_altitude and up > 0:
            up = 0.0
        return VelocityCommand(cmd.forward, cmd.right, up, cmd.yaw_rate)
