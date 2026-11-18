"""Ports: the interfaces the use cases depend on.

Swapping simulation for real hardware = providing different implementations
of these protocols. Nothing in domain/ or usecases/ imports an adapter.
"""
from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

import numpy as np

from drone_core.domain.models import BoundingBox, Frame, MultiFrame, Telemetry, Track, VelocityCommand


@runtime_checkable
class CameraSource(Protocol):
    def read(self) -> Optional[Frame]:
        """Return the newest frame, or None if none is available yet."""
        ...

    def close(self) -> None: ...


@runtime_checkable
class MultiCameraSource(Protocol):
    """A rig of several cameras (e.g. front/right/back/left)."""

    def read_all(self) -> Optional[MultiFrame]:
        """Newest synchronised set of frames, or None."""
        ...

    def close(self) -> None: ...


@runtime_checkable
class DepthEstimator(Protocol):
    """Image -> metric depth map (metres). Real drone: a monocular depth
    network. Not needed in the sim, which sends ground-truth depth."""

    def estimate(self, frame: Frame) -> np.ndarray: ...


@runtime_checkable
class TelemetrySource(Protocol):
    def latest(self) -> Optional[Telemetry]: ...


@runtime_checkable
class DroneCommander(Protocol):
    def send_velocity(self, cmd: VelocityCommand) -> None: ...

    def hover(self) -> None: ...

    def close(self) -> None: ...


@runtime_checkable
class ObjectTracker(Protocol):
    """Detection + multi-object tracking (e.g. YOLO + ByteTrack)."""

    def update(self, frame: Frame) -> list[Track]: ...


@runtime_checkable
class Clock(Protocol):
    def now(self) -> float: ...


@runtime_checkable
class AppearanceEncoder(Protocol):
    """Person crop -> appearance vector ("fingerprint"), L2-normalised, so the
    same person gives similar vectors from different views (re-ID).
    Sim: colour histograms. Real: a re-ID network (yolo26n-reid, OSNet)."""

    def embed(self, image: np.ndarray, boxes: list[BoundingBox]) -> list[Optional[np.ndarray]]: ...
