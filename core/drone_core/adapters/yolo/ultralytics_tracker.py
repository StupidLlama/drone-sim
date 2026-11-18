"""ObjectTracker backed by Ultralytics YOLO + ByteTrack (or BoT-SORT)."""
from __future__ import annotations

from typing import Optional, Sequence

from drone_core.domain.models import BoundingBox, Frame, Track


class UltralyticsTracker:
    def __init__(self, model: str = "yolo11n.pt", tracker: str = "bytetrack.yaml",
                 conf: float = 0.3, imgsz: int = 640, device: Optional[str] = None,
                 classes: Optional[Sequence[str]] = ("person",)):
        from ultralytics import YOLO  # imported lazily so the core never needs it
        self.model = YOLO(model)
        self.tracker = tracker
        self.conf = conf
        self.imgsz = imgsz
        self.device = device
        names = self.model.names  # {id: name}
        self.class_ids = None if not classes else [i for i, n in names.items() if n in set(classes)]

    def update(self, frame: Frame) -> list[Track]:
        results = self.model.track(frame.image, persist=True, tracker=self.tracker,
                                   conf=self.conf, imgsz=self.imgsz, device=self.device,
                                   classes=self.class_ids, verbose=False)
        r = results[0]
        if r.boxes is None or r.boxes.id is None:
            return []
        out = []
        xyxy = r.boxes.xyxy.cpu().numpy()
        ids = r.boxes.id.int().cpu().tolist()
        clss = r.boxes.cls.int().cpu().tolist()
        confs = r.boxes.conf.cpu().tolist()
        for (x1, y1, x2, y2), tid, c, cf in zip(xyxy, ids, clss, confs):
            out.append(Track(int(tid), BoundingBox(float(x1), float(y1), float(x2), float(y2)),
                             r.names[c], float(cf)))
        return out
