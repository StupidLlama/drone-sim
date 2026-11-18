"""Flight log: one CSV row per control tick. Plot with tools/plot_log.py."""
from __future__ import annotations

import csv

from drone_core.usecases.follow_mission import TickResult

FIELDS = ["t", "state", "ex", "ey", "ed", "forward", "right", "up", "yaw_rate",
          "x", "y", "alt", "yaw", "battery", "front", "box_cx", "box_cy", "box_h", "n_tracks"]


class CsvRecorder:
    def __init__(self, path: str):
        self._f = open(path, "w", newline="")
        self._w = csv.DictWriter(self._f, fieldnames=FIELDS)
        self._w.writeheader()

    def __call__(self, r: TickResult) -> None:
        s, c, tm = r.status, r.status.command, r.telemetry
        row = {"t": f"{r.frame.timestamp:.3f}", "state": s.state, "n_tracks": len(r.tracks),
               "forward": f"{c.forward:.3f}", "right": f"{c.right:.3f}", "up": f"{c.up:.3f}",
               "yaw_rate": f"{c.yaw_rate:.3f}"}
        for k in ("ex", "ey", "ed"):
            if k in s.errors:
                row[k] = f"{s.errors[k]:.4f}"
        if tm is not None:
            row.update(x=f"{tm.position[0]:.2f}", y=f"{tm.position[1]:.2f}", alt=f"{tm.altitude:.2f}",
                       yaw=f"{tm.yaw:.3f}", battery=f"{tm.battery:.3f}")
            if "front" in tm.ranges:
                row["front"] = f"{tm.ranges['front']:.2f}"
        if s.target is not None:
            b = s.target.box
            row.update(box_cx=f"{b.cx / r.frame.width:.4f}", box_cy=f"{b.cy / r.frame.height:.4f}",
                       box_h=f"{b.h / r.frame.height:.4f}")
        self._w.writerow(row)

    def close(self) -> None:
        self._f.close()
