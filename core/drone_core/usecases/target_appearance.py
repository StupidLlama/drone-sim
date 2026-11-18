"""Remember what the target looks like, so re-locking after a loss (and
noticing an id swap) uses appearance, not just "who is near where it should be".

How it works (after RPF-ReID, Ye et al. 2023, simplified):
  * While locked and the target is clearly visible (confident, not overlapping
    anyone, not cut off), save its appearance vector every sample_every_s into a
    gallery. Only views that still match the gallery are added, so a silent id
    swap can't teach it the wrong person.
  * While the target is visible, everyone else in the frame is certainly NOT the
    target: save them as negatives.
  * A candidate is accepted when it looks like the gallery (score >= match) and
    more like the target than like any negative (by margin).
  * If the locked track stops looking like the target for switch_checks checks
    in a row (while not overlapping anyone), the tracker swapped ids: drop it.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from drone_core.domain.models import BoundingBox, Track
from drone_core.ports import AppearanceEncoder


@dataclass
class AppearanceConfig:
    match: Optional[float] = None      # similarity needed; None = encoder.match_threshold or 0.8
    margin: float = 0.03               # must look more like the target than any negative by this
    sample_every_s: float = 0.3
    max_samples: int = 60
    negatives_every_s: float = 1.0
    max_negatives: int = 80
    min_samples: int = 3               # no opinion before this many target samples
    check_every_s: float = 0.25        # how often to re-check the locked track
    switch_checks: int = 4             # this many failed checks in a row = id swap
    min_confidence: float = 0.4
    max_overlap: float = 0.05          # fraction of the box covered by another box
    edge_px: float = 3.0               # box touching the image edge = cut off, don't sample
    max_candidates: int = 6            # embed at most this many when re-locking (nearest first)
    gate_bonus_m: float = 3.0          # a strong look-alike may be this much further from the prediction


def _overlap_frac(b: BoundingBox, others: list[BoundingBox]) -> float:
    best = 0.0
    for o in others:
        w = min(b.x2, o.x2) - max(b.x1, o.x1)
        h = min(b.y2, o.y2) - max(b.y1, o.y1)
        if w > 0 and h > 0:
            best = max(best, w * h / max(b.area, 1e-6))
    return best


class TargetAppearance:
    def __init__(self, encoder: AppearanceEncoder, cfg: AppearanceConfig | None = None):
        self.encoder = encoder
        self.cfg = cfg or AppearanceConfig()
        self.reset()

    @property
    def match(self) -> float:
        if self.cfg.match is not None:
            return self.cfg.match
        return float(getattr(self.encoder, "match_threshold", 0.8))

    def reset(self) -> None:
        self.gallery: list[np.ndarray] = []
        self.negatives: list[np.ndarray] = []
        self.current: dict[int, Optional[np.ndarray]] = {}   # this frame's embeddings by track id
        self._last_sample = -1e9
        self._last_neg = -1e9
        self._last_check = -1e9
        self._bad = 0
        self.last_score: Optional[float] = None
        self.embeds = 0                                       # encoder calls (crops), for profiling

    @property
    def ready(self) -> bool:
        return len(self.gallery) >= self.cfg.min_samples

    # ------------------------------------------------------------- observing
    def observe(self, image: np.ndarray, tracks: list[Track], locked_id: Optional[int], t: float,
                frame_w: int, frame_h: int, relocking: bool, near=None) -> Optional[str]:
        """Update the memory from one front-camera frame. Returns "swap" when
        the locked track no longer looks like the target."""
        c = self.cfg
        self.current = {}
        people = [tr for tr in tracks if tr.confidence >= c.min_confidence]
        locked = next((tr for tr in people if tr.track_id == locked_id), None)
        want: list[Track] = []
        clean_locked = False
        if locked is not None and not relocking:
            others = [tr.box for tr in people if tr is not locked]
            clean_locked = self._clean(locked.box, others, frame_w, frame_h)
            due = t - self._last_sample >= c.sample_every_s or (self.ready and t - self._last_check >= c.check_every_s)
            if clean_locked and due:
                want.append(locked)
            if clean_locked and self.ready and t - self._last_neg >= c.negatives_every_s:
                if locked not in want:
                    want.append(locked)
                for tr in people:
                    if tr is not locked and self._clean(tr.box, [o.box for o in people if o is not tr],
                                                        frame_w, frame_h):
                        want.append(tr)
        elif self.ready:
            cands = [tr for tr in people if near is None or near(tr) is not None]
            if near is not None:
                cands.sort(key=lambda tr: near(tr))
            want = cands[:c.max_candidates]
        if not want:
            return None
        vecs = self.encoder.embed(image, [tr.box for tr in want])
        self.embeds += len(want)
        for tr, v in zip(want, vecs):
            self.current[tr.track_id] = v
        event = None
        if locked is not None and not relocking:
            v = self.current.get(locked.track_id)
            if v is not None:
                s = self.similarity(v)
                self.last_score = s
                if self.ready:
                    self._last_check = t
                    if s < self.match:
                        self._bad += 1
                        if self._bad >= c.switch_checks:
                            self._bad = 0
                            event = "swap"
                    else:
                        self._bad = 0
                if (not self.ready or s >= self.match) and t - self._last_sample >= c.sample_every_s:
                    self._add(self.gallery, v, c.max_samples)
                    self._last_sample = t
            # Others are "not the target" only if we're sure the locked track IS the target
            # (right after an unnoticed id swap, the real target would be filed as a negative).
            sure = v is not None and self.ready and self.similarity(v) >= self.match
            negs = [v2 for tid, v2 in self.current.items() if tid != locked.track_id and v2 is not None]
            if negs and sure:
                for v2 in negs:
                    self._add(self.negatives, v2, c.max_negatives)
                self._last_neg = t
        return event

    def _clean(self, b: BoundingBox, others: list[BoundingBox], frame_w: int, frame_h: int) -> bool:
        e = self.cfg.edge_px
        if b.x1 <= e or b.y1 <= e or b.x2 >= frame_w - 1 - e or b.y2 >= frame_h - 1 - e:
            return False
        return _overlap_frac(b, others) <= self.cfg.max_overlap

    @staticmethod
    def _add(store: list, v: np.ndarray, cap: int) -> None:
        store.append(v)
        if len(store) > cap:
            # keep the first few (the original look) and the newest ones
            del store[3]

    # --------------------------------------------------------------- scoring
    def similarity(self, v: np.ndarray) -> float:
        if not self.gallery:
            return 1.0
        sims = np.stack(self.gallery) @ v
        k = min(3, len(sims))
        return float(np.mean(np.sort(sims)[-k:]))

    def negative_similarity(self, v: np.ndarray) -> float:
        if not self.negatives:
            return -1.0
        return float(np.max(np.stack(self.negatives) @ v))

    def verdict(self, track_id: int) -> Optional[float]:
        """None = no opinion (not ready, or no embedding for this track).
        Otherwise a score; >= 0 accepts, < 0 rejects; bigger = more sure."""
        if not self.ready:
            return None
        v = self.current.get(track_id)
        if v is None:
            return None
        s = self.similarity(v)
        neg = self.negative_similarity(v)
        return min(s - self.match, s - neg - self.cfg.margin)
