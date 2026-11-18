from __future__ import annotations

import dataclasses
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Union

import numpy as np

from drone_core.domain.models import FollowStatus, Frame, MultiFrame, Telemetry, Track
from drone_core.ports import (CameraSource, Clock, DroneCommander, MultiCameraSource, ObjectTracker,
                              TelemetrySource)
from drone_core.usecases.avoidance import AvoidanceFilter
from drone_core.usecases.flight_supervisor import FlightSupervisor
from drone_core.usecases.follow_target import FollowTarget


class SystemClock:
    def now(self) -> float:
        return time.monotonic()


@dataclass
class TickResult:
    frame: Frame
    tracks: list[Track]
    status: FollowStatus
    telemetry: Optional[Telemetry] = None
    multi: Optional[MultiFrame] = None                       # all cameras, if a rig is used
    other_tracks: dict = field(default_factory=dict)         # camera name -> tracks (not front)


class FollowMission:
    """The main loop: camera -> tracker -> follow -> supervisor -> avoidance -> commander.

    Depends only on ports, so the same object runs against Unity, the mock
    simulator, or the real drone.
    """

    def __init__(self, camera: Union[CameraSource, MultiCameraSource],
                 tracker: Union[ObjectTracker, dict], commander: DroneCommander,
                 follow: FollowTarget, avoidance: AvoidanceFilter,
                 telemetry: Optional[TelemetrySource] = None, clock: Optional[Clock] = None,
                 on_tick: Optional[Callable[[TickResult], None]] = None,
                 frame_timeout_s: float = 0.5, supervisor: Optional[FlightSupervisor] = None,
                 surround_map=None, depth_estimator=None, depth_every: int = 3,
                 voxel_map=None):
        self.camera = camera
        self.tracker = tracker
        self.commander = commander
        self.follow = follow
        self.avoidance = avoidance
        self.telemetry = telemetry
        self.clock = clock or SystemClock()
        self.on_tick = on_tick
        self.frame_timeout_s = frame_timeout_s
        self.supervisor = supervisor
        # tracker: one ObjectTracker (front camera only) or {camera name: tracker}.
        # Trackers keep per-stream state (ByteTrack ids), so each camera needs its own.
        self.trackers = tracker if isinstance(tracker, dict) else {"front": tracker}
        # Optional obstacle map from the cameras' depth. Its distances are merged
        # into the telemetry ranges, so AvoidanceFilter uses them automatically.
        self.surround_map = surround_map
        # Optional 3-D map from the sonar array (+ depth). Adds front_up /
        # front_down / up / below so avoidance can go over or under branches.
        self.voxel_map = voxel_map
        if voxel_map is not None and getattr(follow, "line_of_sight", "x") is None:
            follow.line_of_sight = voxel_map.line_blocked     # climb when the view is blocked
        self.depth_estimator = depth_estimator    # real drone: depth from images
        self.depth_every = depth_every            # run the (slow) estimator every n frames
        self._ticks = 0
        self.map_send_every_s = 0.5
        self._last_map_sent = -1e9
        self._last_frame_t: Optional[float] = None
        self._last_frame_id: Optional[int] = None

    def tick(self) -> Optional[TickResult]:
        now = self.clock.now()
        multi = self._read()
        frame = multi.primary if multi is not None else None
        if frame is None or multi.frame_id == self._last_frame_id:
            # No new image. Failsafe: hover if the feed has stalled.
            if self._last_frame_t is not None and now - self._last_frame_t > self.frame_timeout_s:
                self.commander.hover()
            return None
        self._last_frame_t = now
        self._last_frame_id = multi.frame_id

        telem = self.telemetry.latest() if self.telemetry else None
        tracks: list[Track] = []
        other: dict = {}
        for name, f in multi.frames.items():
            tr = self.trackers.get(name)
            if tr is None:
                continue
            found = [dataclasses.replace(t, camera=name) for t in tr.update(f)]
            if name == "front":
                tracks = found
            else:
                other[name] = found
        if (self.surround_map is not None or self.voxel_map is not None) and telem is not None:
            telem = self._update_map(multi, telem)
        if getattr(self.follow, "appearance", None) is not None:
            self.follow.observe_appearance(frame.image, tracks, frame.width, frame.height, now, telem)
        status = self.follow.step(tracks, frame.width, frame.height, now, telem, other)
        if self.supervisor:
            status = self.supervisor.step(status, telem, now)   # may override with HOLD / RTH
        status.command = self.avoidance.apply(status.command, telem)
        if hasattr(self.commander, "show_estimate"):        # Unity draws a ghost there
            self.commander.show_estimate(self.follow.estimate(now, status.state))
        if (self.voxel_map is not None and hasattr(self.commander, "show_map")
                and now - self._last_map_sent >= self.map_send_every_s):
            self._last_map_sent = now                       # Unity draws the 3-D map (2 Hz)
            pts = self.voxel_map.occupied_points()
            if telem is not None and len(pts):
                d = np.linalg.norm(pts[:, :2] - np.array(telem.position[:2]), axis=1)
                pts = pts[np.argsort(d)]                      # nearest first if it has to be cut
            self.commander.show_map(pts, self.voxel_map.cfg.resolution)
        self.commander.send_velocity(status.command)

        result = TickResult(frame, tracks, status, telem, multi, other)
        if self.on_tick:
            self.on_tick(result)
        return result

    def _update_map(self, multi: MultiFrame, telem: Telemetry) -> Telemetry:
        self._ticks += 1
        if not multi.depth and self.depth_estimator is not None and self._ticks % self.depth_every == 0:
            depth = {n: self.depth_estimator.estimate(f) for n, f in multi.frames.items()}
            multi = dataclasses.replace(multi, depth=depth)
        ranges = dict(telem.ranges)
        maps = []
        if self.surround_map is not None:
            self.surround_map.update(multi, telem, ignore_xy=self.follow.world.pos)
            maps.append(self.surround_map)
        if self.voxel_map is not None:
            self.voxel_map.update(telem, multi, ignore_xy=self.follow.world.pos)
            maps.append(self.voxel_map)
        for m in maps:
            for k, v in m.ranges(telem).items():
                ranges[k] = min(v, ranges.get(k, float("inf")))
        return dataclasses.replace(telem, ranges=ranges)

    def _read(self) -> Optional[MultiFrame]:
        if hasattr(self.camera, "read_all"):
            return self.camera.read_all()
        f = self.camera.read()
        return None if f is None else MultiFrame({"front": f}, f.timestamp, f.frame_id)

    def run(self, should_stop: Callable[[], bool] = lambda: False, idle_sleep: float = 0.002) -> None:
        try:
            while not should_stop():
                if self.tick() is None:
                    time.sleep(idle_sleep)
        finally:
            self.commander.hover()
