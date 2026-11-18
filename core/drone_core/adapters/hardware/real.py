"""Real-hardware adapters. Same ports as the Unity/mock ones.

STATUS: written against the documented OpenCV and MAVSDK-Python APIs but not
flight-tested. Test with props OFF, then in SITL (PX4 + Gazebo), before flying.
"""
from __future__ import annotations

import asyncio
import math
import threading
import time
from typing import Optional

import cv2

from drone_core.domain.models import Frame, Telemetry, VelocityCommand


class OpenCVCamera:
    """USB / CSI / RTSP camera via cv2.VideoCapture. Reads on a thread so the
    core always gets the newest frame instead of a stale buffered one."""

    def __init__(self, source: int | str = 0, width: int = 640, height: int = 480):
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._frame: Optional[Frame] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._id = 0
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            ok, img = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            self._id += 1
            with self._lock:
                self._frame = Frame(img, time.monotonic(), self._id)

    def read(self) -> Optional[Frame]:
        with self._lock:
            return self._frame

    def close(self) -> None:
        self._stop.set()
        self.cap.release()


class MultiOpenCVCamera:
    """Several cv2 cameras read as one rig (front/right/back/left). Frames are
    the newest from each camera, so they're within a frame period of each other."""

    def __init__(self, sources, names, width: int = 640, height: int = 480):
        from drone_core.domain.models import MultiFrame  # noqa: F401
        self.cams = {n: OpenCVCamera(s, width, height) for n, s in zip(names, sources)}
        self._id = 0

    def read_all(self):
        from drone_core.domain.models import Frame, MultiFrame
        frames = {}
        for n, cam in self.cams.items():
            f = cam.read()
            if f is None:
                return None
            frames[n] = Frame(f.image, f.timestamp, f.frame_id, n)
        self._id += 1
        return MultiFrame(frames, max(f.timestamp for f in frames.values()), self._id)

    def close(self) -> None:
        for c in self.cams.values():
            c.close()


class MavsdkDrone:
    """DroneCommander + TelemetrySource over MAVSDK (PX4 / ArduPilot offboard).

    MAVSDK is async; this runs its own event loop on a background thread so
    the synchronous core can call send_velocity() from its loop.
    Usage: d = MavsdkDrone("udp://:14540"); d.connect_and_arm(takeoff_alt=3)
    """

    def __init__(self, address: str = "udp://:14540"):
        from mavsdk import System  # lazy import
        self.address = address
        self.system = System()
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        self._telem = Telemetry(timestamp=0.0)
        self._lock = threading.Lock()

    def _run(self, coro, timeout: float = 30.0):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    def connect_and_arm(self, takeoff_alt: float = 3.0) -> None:
        self._run(self._connect_and_arm(takeoff_alt), timeout=120)

    async def _connect_and_arm(self, alt: float) -> None:
        from mavsdk.offboard import VelocityBodyYawspeed
        await self.system.connect(system_address=self.address)
        async for state in self.system.core.connection_state():
            if state.is_connected:
                break
        async for health in self.system.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break
        await self.system.action.set_takeoff_altitude(alt)
        await self.system.action.arm()
        await self.system.action.takeoff()
        await asyncio.sleep(8)
        await self.system.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        await self.system.offboard.start()
        asyncio.ensure_future(self._telemetry_task())

    async def _telemetry_task(self) -> None:
        async def pos():
            async for p in self.system.telemetry.position():
                with self._lock:
                    self._telem = Telemetry(time.monotonic(), self._telem.position, self._telem.velocity_body,
                                            self._telem.yaw, p.relative_altitude_m, self._telem.battery)
        async def att():
            async for a in self.system.telemetry.attitude_euler():
                with self._lock:
                    t = self._telem
                    self._telem = Telemetry(t.timestamp, t.position, t.velocity_body,
                                            math.radians(a.yaw_deg), t.altitude, t.battery)
        await asyncio.gather(pos(), att())

    # --- DroneCommander ---
    def send_velocity(self, cmd: VelocityCommand) -> None:
        from mavsdk.offboard import VelocityBodyYawspeed
        # core: up+, yaw rad/s  ->  MAVSDK: down+, yaw deg/s (both + = clockwise)
        v = VelocityBodyYawspeed(cmd.forward, cmd.right, -cmd.up, math.degrees(cmd.yaw_rate))
        asyncio.run_coroutine_threadsafe(self.system.offboard.set_velocity_body(v), self.loop)

    def hover(self) -> None:
        self.send_velocity(VelocityCommand.hover())

    def close(self) -> None:
        self.hover()

    # --- TelemetrySource ---
    def latest(self) -> Optional[Telemetry]:
        with self._lock:
            return self._telem

    def land(self) -> None:
        self._run(self.system.action.land())
