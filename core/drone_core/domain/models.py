"""Pure domain models. No I/O, no frameworks — only numpy for image buffers.

Coordinate convention used everywhere in the core (body frame, like MAVSDK's
VelocityBodyYawspeed but with *up* instead of *down*):

    forward  (+ = nose direction)        m/s
    right    (+ = drone's right)         m/s
    up       (+ = climb)                 m/s
    yaw_rate (+ = turn right / clockwise seen from above)   rad/s

Adapters translate to/from whatever their backend uses.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class Frame:
    image: np.ndarray          # HxWx3 BGR uint8 (OpenCV convention)
    timestamp: float           # seconds, source clock
    frame_id: int = 0
    camera: str = "front"      # which camera of the rig took it

    @property
    def width(self) -> int:
        return int(self.image.shape[1])

    @property
    def height(self) -> int:
        return int(self.image.shape[0])


@dataclass(frozen=True)
class BoundingBox:
    """Pixel-space box, (x1, y1) top-left, (x2, y2) bottom-right."""
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2.0

    @property
    def w(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def h(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.w * self.h


@dataclass(frozen=True)
class Track:
    """A detection that the tracker has given a persistent id."""
    track_id: int
    box: BoundingBox
    class_name: str
    confidence: float
    camera: str = "front"      # which camera saw it


@dataclass(frozen=True)
class CameraMount:
    """Where a camera sits on the drone. yaw is relative to the nose,
    + = to the right (90 = right camera, 180 = rear)."""
    name: str
    yaw_deg: float = 0.0
    pitch_deg: float = 20.0     # tilted down
    hfov_deg: float = 90.0


# The rig on the real drone: four 90-degree cameras covering 360 degrees.
FOUR_CAMERA_RIG = (
    CameraMount("front", 0.0),
    CameraMount("right", 90.0),
    CameraMount("back", 180.0),
    CameraMount("left", -90.0),
)


@dataclass(frozen=True)
class MultiFrame:
    """One synchronised capture from every camera on the rig."""
    frames: dict                 # camera name -> Frame
    timestamp: float
    frame_id: int
    # Optional per-camera depth in metres (HxW float32, any resolution, NaN/inf =
    # nothing hit). The sim sends ground truth; on the real drone a depth
    # model fills it in.
    depth: dict = field(default_factory=dict)

    @property
    def primary(self) -> "Frame":
        return self.frames.get("front") or next(iter(self.frames.values()))


@dataclass(frozen=True)
class SonarBeam:
    """One ultrasonic transducer. yaw relative to the nose (+ = right),
    pitch + = up. range = distance to the first echo, None = no echo."""
    yaw_deg: float
    pitch_deg: float
    range: Optional[float] = None


@dataclass(frozen=True)
class SonarScan:
    """One reading of every transducer on the drone. Each beam is a cone:
    the echo comes from *somewhere* in it, at the given distance."""
    beams: tuple = ()
    fov_deg: float = 30.0        # full cone angle
    min_range: float = 0.2
    max_range: float = 7.0


def sonar_layout() -> list[tuple[float, float]]:
    """(yaw, pitch) of the default 18-transducer array: 8 level around the
    drone, 4 tilted up 35 deg, 4 tilted down 35 deg, one straight up, one down."""
    out = [(float(a), 0.0) for a in (0, 45, 90, 135, 180, -135, -90, -45)]
    out += [(float(a), 35.0) for a in (0, 90, 180, -90)]
    out += [(float(a), -35.0) for a in (0, 90, 180, -90)]
    out += [(0.0, 90.0), (0.0, -90.0)]
    return out


@dataclass(frozen=True)
class Telemetry:
    timestamp: float
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)   # world, metres (x, y, z-up)
    velocity_body: tuple[float, float, float] = (0.0, 0.0, 0.0)  # forward, right, up
    yaw: float = 0.0           # rad, world heading
    altitude: float = 0.0      # metres above ground
    battery: float = 1.0       # 0..1
    armed: bool = True
    # Range-sensor readings in metres keyed by direction ("front", "left",
    # "right", "back", "down"). Sim: raycasts. Real: ToF/lidar. Missing = clear.
    ranges: dict = field(default_factory=dict)
    sonar: Optional[SonarScan] = None   # ultrasonic array, if the drone has one


@dataclass(frozen=True)
class VelocityCommand:
    forward: float = 0.0
    right: float = 0.0
    up: float = 0.0
    yaw_rate: float = 0.0

    def clamped(self, max_xy: float, max_z: float, max_yaw: float) -> "VelocityCommand":
        c = lambda v, m: float(max(-m, min(m, v)))
        return VelocityCommand(c(self.forward, max_xy), c(self.right, max_xy),
                               c(self.up, max_z), c(self.yaw_rate, max_yaw))

    @staticmethod
    def hover() -> "VelocityCommand":
        return VelocityCommand()


@dataclass
class FollowStatus:
    """What the follow use case decided this tick (for logging / HUD)."""
    state: str                      # "TRACKING" | "LOST" | "SEARCHING" | "IDLE"
    target: Optional[Track] = None
    command: VelocityCommand = field(default_factory=VelocityCommand)
    errors: dict = field(default_factory=dict)
