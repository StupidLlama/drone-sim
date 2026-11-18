from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

from drone_core.domain.models import BoundingBox, Track


@dataclass
class TargetSelectorConfig:
    target_class: str = "person"
    min_confidence: float = 0.35
    # When the locked id disappears, accept a new track as "the same target" if
    # its centre is within this fraction of the frame diagonal of the last box.
    reacquire_radius_frac: float = 0.25
    # When re-locking by predicted ground position: max distance (m) between
    # a candidate and where we expected the hidden target to be.
    reacquire_gate_m: float = 4.0


class TargetSelector:
    """Locks onto one track id and keeps it. ByteTrack ids can change after an
    occlusion, so if the locked id vanishes we re-lock on the best candidate
    near the last known position (or the largest one if we have no history)."""

    def __init__(self, cfg: TargetSelectorConfig | None = None):
        self.cfg = cfg or TargetSelectorConfig()
        self.locked_id: Optional[int] = None
        self.last_box: Optional[BoundingBox] = None
        self.banned: set = set()      # track ids caught being someone else (id swap), for a short while

    def reset(self) -> None:
        self.locked_id = None
        self.last_box = None
        self.banned = set()

    def lock(self, track_id: int) -> None:
        """Manual lock, e.g. operator clicked a box."""
        self.locked_id = track_id

    def select(self, tracks: list[Track], frame_w: int, frame_h: int,
               gate: Optional[Callable[[Track], Optional[float]]] = None,
               accept: Optional[Callable[[Track], bool]] = None) -> Optional[Track]:
        """gate(track) -> distance in metres from where we expect the target,
        or None if it can't be judged. When given, it replaces the image-space
        check, so the lock survives the drone moving while the target was hidden.
        accept(track) -> False rules a candidate out when re-locking (e.g. it
        doesn't look like the target)."""
        candidates = [t for t in tracks
                      if t.class_name == self.cfg.target_class
                      and t.confidence >= self.cfg.min_confidence
                      and t.track_id not in self.banned]
        if not candidates:
            return None

        if self.locked_id is not None:
            for t in candidates:
                if t.track_id == self.locked_id:
                    self.last_box = t.box
                    return t
        if accept is not None:
            candidates = [t for t in candidates if accept(t)]
            if not candidates:
                return None

        chosen: Optional[Track] = None
        if gate is not None:
            scored = [(d, t) for t in candidates if (d := gate(t)) is not None]
            if scored:
                d, best = min(scored, key=lambda x: x[0])
                chosen = best if d <= self.cfg.reacquire_gate_m else None
        elif self.last_box is not None:
            diag = math.hypot(frame_w, frame_h)
            def dist(t: Track) -> float:
                return math.hypot(t.box.cx - self.last_box.cx, t.box.cy - self.last_box.cy)
            nearest = min(candidates, key=dist)
            if dist(nearest) <= self.cfg.reacquire_radius_frac * diag:
                chosen = nearest
        else:
            # No history yet: take the largest (closest) candidate.
            chosen = max(candidates, key=lambda t: t.box.area)
        # History but nothing nearby -> None. We don't jump to a stranger; the
        # follow use case calls reset() after a long loss to allow that.

        if chosen is not None:
            self.locked_id = chosen.track_id
            self.last_box = chosen.box
        return chosen
