"""Wire protocol shared with Unity's PythonBridge.cs. Keep the two in sync.

Every message, both directions:
    uint32 big-endian  length of (type + payload)
    uint8              message type
    bytes              payload

Unity -> Python
    MSG_FRAME (1):  uint32 BE header_len | header JSON (utf-8) | blobs
                    blobs = the JPEG of each camera in header["cams"] order, then
                    the depth image of each entry in header["depth"] (float16 LE,
                    metres, row 0 = top, inf = nothing hit).
                    header["cams"]  = [{"name","yaw_deg","len"}, ...]
                    header["depth"] = [{"name","w","h","len"}, ...]   (optional)
                    No "cams" key: the blob is a single front-camera JPEG.
                    header = {"frame_id", "t", "pos":[x,y,z], "yaw_deg",
                              "vel_body":[fwd,right,up], "alt", "battery", "ranges":{...}}
                    pos is Unity world (x right, y up, z forward).
                    header["sonar"] (optional) = {"fov","min","max",
                              "beams":[[yaw_deg, pitch_deg, range], ...]}
                              range < 0 = no echo.
Python -> Unity
    MSG_COMMAND (2): JSON {"forward","right","up" (m/s), "yaw_rate" (rad/s, + = turn right),
                           optional "est": {"has":1,"x","y","h","r","state"} for the ghost target}
    MSG_RESET   (5): empty — respawn drone & target
    MSG_MAP     (6): Py->Unity, the 3-D obstacle map to draw: uint32 LE count | float32 LE voxel size (m) |
                     count x (int16 LE x, y, z in cm, core world frame: x east, y north, z up)
"""
from __future__ import annotations

import json
import math
import socket
import struct
from typing import Optional

import cv2
import numpy as np

from drone_core.domain.models import Frame, MultiFrame, SonarBeam, SonarScan, Telemetry, VelocityCommand

MSG_FRAME = 1
MSG_COMMAND = 2
MSG_RESET = 5
MSG_MAP = 6

_HDR = struct.Struct(">IB")
_U32 = struct.Struct(">I")
MAX_MSG = 32 * 1024 * 1024


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    return _HDR.pack(len(payload) + 1, msg_type) + payload


def encode_command(cmd: VelocityCommand, est: dict | None = None) -> bytes:
    """est (optional, for the ghost target in Unity): where the core thinks the
    target is. {"x","y"} core world metres (= Unity x, z), "h" height of what the
    tracker boxes (m), "r" re-lock radius (m), "state" follow state."""
    d = {"forward": cmd.forward, "right": cmd.right, "up": cmd.up, "yaw_rate": cmd.yaw_rate}
    if est is not None:
        d["est"] = {"has": 1, "x": round(est["x"], 3), "y": round(est["y"], 3), "h": round(est.get("h", 1.8), 2),
                    "r": round(est.get("r", 0.5), 2), "state": est.get("state", "")}
    return encode(MSG_COMMAND, json.dumps(d).encode())


MAP_MAX_VOXELS = 40000


def encode_map(points: np.ndarray, resolution: float) -> bytes:
    """Occupied voxel centres (n, 3) metres -> MSG_MAP. Capped at MAP_MAX_VOXELS
    (nearest kept by caller's ordering) so a message stays well under 1 MB."""
    pts = np.asarray(points, np.float64).reshape(-1, 3)[:MAP_MAX_VOXELS]
    cm = np.clip(np.round(pts * 100.0), -32768, 32767).astype("<i2")
    return encode(MSG_MAP, struct.pack("<If", len(cm), float(resolution)) + cm.tobytes())


def encode_frame(header: dict, jpeg: bytes) -> bytes:
    """Single-camera frame. Used by tests / fake servers."""
    h = json.dumps(header).encode()
    return encode(MSG_FRAME, _U32.pack(len(h)) + h + jpeg)


def encode_multi_frame(header: dict, jpegs: dict, yaws: dict, depth: dict | None = None) -> bytes:
    """Multi-camera frame, same layout PythonBridge.cs produces."""
    header = dict(header)
    header["cams"] = [{"name": n, "yaw_deg": yaws[n], "len": len(b)} for n, b in jpegs.items()]
    blobs = list(jpegs.values())
    if depth:
        header["depth"] = []
        for n, d in depth.items():
            raw = np.ascontiguousarray(d, dtype="<f2").tobytes()
            header["depth"].append({"name": n, "w": int(d.shape[1]), "h": int(d.shape[0]), "len": len(raw)})
            blobs.append(raw)
    h = json.dumps(header).encode()
    return encode(MSG_FRAME, _U32.pack(len(h)) + h + b"".join(blobs))


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf.extend(chunk)
    return bytes(buf)


def read_message(sock: socket.socket) -> tuple[int, bytes]:
    length, msg_type = _HDR.unpack(_recv_exact(sock, _HDR.size))
    if length < 1 or length > MAX_MSG:
        raise ValueError(f"bad message length {length}")
    return msg_type, _recv_exact(sock, length - 1)


def _jpeg(buf: bytes) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("could not decode JPEG")
    return image


def decode_frame(payload: bytes) -> tuple[MultiFrame, Telemetry]:
    (hlen,) = _U32.unpack_from(payload, 0)
    header = json.loads(payload[4:4 + hlen].decode())
    body = memoryview(payload)[4 + hlen:]
    t = float(header.get("t", 0.0))
    fid = int(header.get("frame_id", 0))
    frames, depth = {}, {}
    if "cams" not in header:
        frames["front"] = Frame(_jpeg(bytes(body)), t, fid, "front")
    else:
        off = 0
        for c in header["cams"]:
            n = int(c["len"])
            frames[c["name"]] = Frame(_jpeg(bytes(body[off:off + n])), t, fid, c["name"])
            off += n
        for d in header.get("depth", []):
            n = int(d["len"])
            arr = np.frombuffer(bytes(body[off:off + n]), dtype="<f2").astype(np.float32)
            depth[d["name"]] = arr.reshape(int(d["h"]), int(d["w"]))
            off += n
    frame = MultiFrame(frames, t, fid, depth)
    ux, uy, uz = header.get("pos", [0.0, 0.0, 0.0])
    telem = Telemetry(
        timestamp=t,
        position=(ux, uz, uy),                       # Unity (x, y-up, z-fwd) -> core (x, y, z-up)
        velocity_body=tuple(header.get("vel_body", [0.0, 0.0, 0.0])),
        yaw=math.radians(float(header.get("yaw_deg", 0.0))),
        altitude=float(header.get("alt", uy)),
        battery=float(header.get("battery", 1.0)),
        ranges=dict(header.get("ranges", {})),
        sonar=decode_sonar(header.get("sonar")),
    )
    return frame, telem


def decode_sonar(s: Optional[dict]) -> Optional[SonarScan]:
    if not s:
        return None
    beams = tuple(SonarBeam(float(y), float(p), float(r) if r is not None and r >= 0 else None)
                  for y, p, r in s.get("beams", []))
    return SonarScan(beams, float(s.get("fov", 30.0)), float(s.get("min", 0.2)), float(s.get("max", 7.0)))


def encode_sonar(scan: SonarScan) -> dict:
    """Header dict in the format SonarArray.cs sends (used by tests / fakes)."""
    return {"fov": scan.fov_deg, "min": scan.min_range, "max": scan.max_range,
            "beams": [[b.yaw_deg, b.pitch_deg, -1.0 if b.range is None else b.range] for b in scan.beams]}
