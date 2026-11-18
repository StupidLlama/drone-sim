"""Stand-in for the Unity scene: speaks the exact same TCP protocol as
PythonBridge.cs but renders the mock world. Use it to check the Python side
(including the Unity adapter) before Unity is set up:

    terminal 1:  python tools/fake_unity.py
    terminal 2:  python main.py --mode unity --tracker color
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cv2  # noqa: E402

from drone_core.adapters.mock.sim import MockWorld  # noqa: E402
from drone_core.adapters.unity import protocol  # noqa: E402
from drone_core.domain.models import FOUR_CAMERA_RIG, VelocityCommand  # noqa: E402


def serve_client(conn: socket.socket, world: MockWorld, fps: float, stop: threading.Event) -> None:
    def reader():
        try:
            while not stop.is_set():
                t, payload = protocol.read_message(conn)
                if t == protocol.MSG_COMMAND:
                    d = json.loads(payload)
                    world.cmd = VelocityCommand(d["forward"], d["right"], d["up"], d["yaw_rate"])
                elif t == protocol.MSG_RESET:
                    world.reset()
        except (ConnectionError, OSError):
            pass
    threading.Thread(target=reader, daemon=True).start()

    dt = 1.0 / fps
    try:
        while not stop.is_set():
            t0 = time.monotonic()
            world.step(dt)
            world.frame_id += 1
            jpgs, yaws, depth = {}, {}, {}
            for m in FOUR_CAMERA_RIG:
                ok, jpg = cv2.imencode(".jpg", world.render(m.yaw_deg), [cv2.IMWRITE_JPEG_QUALITY, 80])
                jpgs[m.name], yaws[m.name] = jpg.tobytes(), m.yaw_deg
                depth[m.name] = world.depth(m.yaw_deg, 32, 24)
            x, y, z = world.pos  # core world (x, y, z-up) -> Unity (x, y-up, z)
            header = {"frame_id": world.frame_id, "t": world.t, "pos": [x, z, y],
                      "yaw_deg": math.degrees(world.yaw), "alt": z,
                      "vel_body": list(world.telemetry().velocity_body), "battery": world.battery, "ranges": world.ranges()}
            conn.sendall(protocol.encode_multi_frame(header, jpgs, yaws, depth))
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))
    except OSError:
        pass


def run_server(host: str, port: int, fps: float, stop: threading.Event, world: MockWorld | None = None,
               ready: threading.Event | None = None) -> None:
    world = world or MockWorld()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    srv.settimeout(0.5)
    if ready:
        ready.set()
    print(f"fake Unity listening on {host}:{port}")
    while not stop.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        print("client connected", addr)
        serve_client(conn, world, fps, stop)
        conn.close()
    srv.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--fps", type=float, default=20)
    a = ap.parse_args()
    try:
        run_server(a.host, a.port, a.fps, threading.Event())
    except KeyboardInterrupt:
        pass
