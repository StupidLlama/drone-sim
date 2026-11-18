"""TCP client that talks to Unity's PythonBridge.cs, plus the three port
adapters (camera, telemetry, commander) that share one connection."""
from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Optional

from drone_core.adapters.unity import protocol
from drone_core.domain.models import Frame, MultiFrame, Telemetry, VelocityCommand

log = logging.getLogger(__name__)


class UnityBridge:
    def __init__(self, host: str = "127.0.0.1", port: int = 5555, reconnect_s: float = 1.0):
        self.host, self.port, self.reconnect_s = host, port, reconnect_s
        self._sock: Optional[socket.socket] = None
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._frame: Optional[MultiFrame] = None
        self._telem: Optional[Telemetry] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="unity-bridge", daemon=True)
        self.connected = threading.Event()

    def start(self) -> "UnityBridge":
        self._thread.start()
        return self

    def wait_connected(self, timeout: Optional[float] = None) -> bool:
        return self.connected.wait(timeout)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=2.0)
                sock.settimeout(None)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._sock = sock
                self.connected.set()
                log.info("connected to Unity at %s:%d", self.host, self.port)
                while not self._stop.is_set():
                    msg_type, payload = protocol.read_message(sock)
                    if msg_type == protocol.MSG_FRAME:
                        frame, telem = protocol.decode_frame(payload)
                        with self._state_lock:
                            self._frame, self._telem = frame, telem
            except (OSError, ConnectionError, ValueError) as e:
                if not self._stop.is_set():
                    log.warning("Unity link: %s — retrying in %.1fs", e, self.reconnect_s)
            finally:
                self.connected.clear()
                if self._sock:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                    self._sock = None
            self._stop.wait(self.reconnect_s)

    def latest_multi(self) -> Optional[MultiFrame]:
        with self._state_lock:
            return self._frame

    def latest_frame(self) -> Optional[Frame]:
        m = self.latest_multi()
        return None if m is None else m.primary

    def latest_telemetry(self) -> Optional[Telemetry]:
        with self._state_lock:
            return self._telem

    def send(self, data: bytes) -> None:
        sock = self._sock
        if sock is None:
            return
        with self._send_lock:
            try:
                sock.sendall(data)
            except OSError as e:
                log.warning("send failed: %s", e)

    def close(self) -> None:
        self._stop.set()
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self._thread.join(timeout=2.0)


class UnityCamera:
    def __init__(self, bridge: UnityBridge):
        self.bridge = bridge

    def read(self) -> Optional[Frame]:
        return self.bridge.latest_frame()

    def close(self) -> None:
        pass


class UnityMultiCamera:
    """All cameras the Unity drone streams (front/right/back/left), plus
    ground-truth depth when the scene sends it."""

    def __init__(self, bridge: UnityBridge):
        self.bridge = bridge

    def read_all(self) -> Optional[MultiFrame]:
        return self.bridge.latest_multi()

    def close(self) -> None:
        pass


class UnityTelemetry:
    def __init__(self, bridge: UnityBridge):
        self.bridge = bridge

    def latest(self) -> Optional[Telemetry]:
        return self.bridge.latest_telemetry()


class UnityCommander:
    def __init__(self, bridge: UnityBridge):
        self.bridge = bridge
        self._est = None

    def show_map(self, points, resolution: float) -> None:
        """Occupied voxels of the 3-D map: drawn as blocks in Unity (X toggles)."""
        self.bridge.send(protocol.encode_map(points, resolution))

    def show_estimate(self, est) -> None:
        """Where the core thinks the target is: drawn as a ghost in Unity."""
        self._est = est

    def send_velocity(self, cmd: VelocityCommand) -> None:
        self.bridge.send(protocol.encode_command(cmd, self._est))

    def hover(self) -> None:
        self.send_velocity(VelocityCommand.hover())

    def reset_sim(self) -> None:
        self.bridge.send(protocol.encode(protocol.MSG_RESET))

    def close(self) -> None:
        self.hover()
