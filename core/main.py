"""Composition root: the ONLY place that knows which adapters are used.

    python main.py --mode unity                 # Unity sim + YOLO/ByteTrack
    python main.py --mode unity --tracker color # Unity sim, red-target colour tracker
    python main.py --mode mock --tracker color  # pure-Python sim, no Unity, no YOLO
    python main.py --mode real --camera 0,1,2,3 --mavlink udp://:14540   # untested!

Multi-camera: Unity and the mock stream front/right/back/left; --cams front
uses only the front one. The obstacle map is built when depth is available
(always in the sim; on the real drone with --depth-model).
"""
from __future__ import annotations

import argparse
import os
import logging
import time

from drone_core.usecases.avoidance import AvoidanceConfig, AvoidanceFilter
from drone_core.usecases.flight_supervisor import FOLLOW, HOLD, RTH, FlightSupervisor, SupervisorConfig
from drone_core.usecases.follow_mission import FollowMission
from drone_core.usecases.follow_target import FollowConfig, FollowTarget
from drone_core.domain.models import FOUR_CAMERA_RIG
from drone_core.usecases.surround_map import SurroundMap
from drone_core.usecases.target_geo import rig_geometry
from drone_core.usecases.voxel_map import VoxelMap
from drone_core.usecases.target_filter import TargetFilter, TargetFilterConfig
from drone_core.usecases.target_selection import TargetSelector, TargetSelectorConfig


def build_trackers(args, names):
    """One tracker per camera: ByteTrack keeps per-stream state."""
    return {n: build_tracker(args) for n in names}


def build_tracker(args):
    if args.tracker == "color":
        from drone_core.adapters.mock.sim import ColorBlobTracker
        return ColorBlobTracker(class_name=args.target_class)
    from drone_core.adapters.yolo.ultralytics_tracker import UltralyticsTracker
    return UltralyticsTracker(model=args.model, tracker=args.tracker_cfg,
                              conf=args.conf, classes=[args.target_class], device=args.device)


def build_appearance(args):
    """Appearance memory of the target (re-lock on someone who LOOKS like them)."""
    log = logging.getLogger("main")
    kind = args.reid
    if kind == "auto":
        kind = "net" if args.tracker == "yolo" else "color"
    if kind == "off":
        return None
    from drone_core.usecases.target_appearance import TargetAppearance
    if kind == "net":
        try:
            from drone_core.adapters.appearance.onnx_reid import OnnxReidEncoder
            enc = OnnxReidEncoder(args.reid_model)
            log.info("appearance memory: %s", args.reid_model)
            return TargetAppearance(enc)
        except Exception as e:   # no onnxruntime / no download: fall back, don't refuse to fly
            log.warning("re-ID network unavailable (%s); using colour histograms", e)
    from drone_core.adapters.appearance.color_hist import HistogramEncoder
    log.info("appearance memory: colour histograms")
    return TargetAppearance(HistogramEncoder())


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["unity", "mock", "real"], default="unity")
    p.add_argument("--tracker", choices=["yolo", "color"], default="yolo")
    p.add_argument("--model", default="yolo11n.pt")
    p.add_argument("--tracker-cfg", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                         "trackers", "botsort_reid.yaml"),
                   help="Ultralytics tracker config (default: BoT-SORT with appearance re-ID; or bytetrack.yaml)")
    p.add_argument("--reid", choices=["auto", "off", "color", "net"], default="auto",
                   help="target appearance memory for re-locking: color = colour histogram, net = yolo26n-reid "
                        "network (needs onnxruntime), auto = net with --tracker yolo, color with --tracker color")
    p.add_argument("--reid-model", default="yolo26n-reid.onnx", help="ONNX re-ID model (downloaded if missing)")
    p.add_argument("--target-class", default="person")
    p.add_argument("--conf", type=float, default=0.3)
    p.add_argument("--device", default=None, help="e.g. cpu, 0, mps")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5555)
    p.add_argument("--camera", default="0", help="real mode: cv2 source(s), comma-separated in rig order "
                   "front,right,back,left, e.g. 0,1,2,3")
    p.add_argument("--cams", default="all", help="'all' = every camera the source has, or 'front'")
    p.add_argument("--no-map", action="store_true", help="don't build the obstacle map")
    p.add_argument("--depth-model", default=None,
                   help="real mode: Hugging Face depth model for the obstacle map, e.g. "
                        "depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf")
    p.add_argument("--mavlink", default="udp://:14540")
    p.add_argument("--box-height", type=float, default=0.30, help="desired target box height fraction (distance)")
    p.add_argument("--distance", type=float, default=None,
                   help="follow distance in metres (default: what --box-height gives for a 1.8 m person, ~4.4 m)")
    p.add_argument("--target-height", type=float, default=1.8, help="height of the person being followed (m)")
    p.add_argument("--orbit", type=float, default=0.0, help="lateral m/s while following")
    p.add_argument("--no-hud", action="store_true")
    p.add_argument("--hud-map", action="store_true", help="also draw the 3-D map in the Python window (Unity shows it: X)")
    p.add_argument("--log", default=None, help="write a CSV flight log (plot with tools/plot_log.py)")
    p.add_argument("--latency", type=float, default=0.08, help="camera-to-command delay to predict across (s)")
    p.add_argument("--geofence", type=float, default=80.0, help="auto return-home beyond this many metres (0 = off)")
    p.add_argument("--low-battery", type=float, default=0.20)
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    log = logging.getLogger("main")

    clock = None
    pace = None  # mock mode: sleep to run in real time so you can watch
    if args.mode == "unity":
        from drone_core.adapters.unity.bridge import UnityBridge, UnityCommander, UnityMultiCamera, UnityTelemetry
        bridge = UnityBridge(args.host, args.port).start()
        log.info("waiting for Unity on %s:%d (press Play in Unity)...", args.host, args.port)
        bridge.wait_connected()
        camera, commander, telemetry = UnityMultiCamera(bridge), UnityCommander(bridge), UnityTelemetry(bridge)
    elif args.mode == "mock":
        from drone_core.adapters.mock.sim import (MockClock, MockCommander, MockMultiCamera, MockTelemetry,
                                                  MockWorld, MockWorldConfig)
        world = MockWorld(MockWorldConfig(obstacles=((3, 8, 1.0), (-4, 3, 1.2), (6, 14, 1.5), (-2, 16, 0.8)),
                                          branches=(((-3, 11, 2.8), (2, 12, 3.0), 0.04),), sonar=True))
        camera = MockMultiCamera(world, with_depth=True)
        commander, telemetry = MockCommander(world), MockTelemetry(world)
        clock = MockClock(world)
        pace = 1.0 / world.cfg.fps
    else:
        from drone_core.adapters.hardware.real import MavsdkDrone, MultiOpenCVCamera
        srcs = [int(x) if x.isdigit() else x for x in args.camera.split(",")]
        camera = MultiOpenCVCamera(srcs, [m.name for m in FOUR_CAMERA_RIG][:len(srcs)])
        drone = MavsdkDrone(args.mavlink)
        drone.connect_and_arm(takeoff_alt=3.0)
        commander = telemetry = drone

    supervisor = FlightSupervisor(SupervisorConfig(geofence_radius=args.geofence, low_battery=args.low_battery))

    def on_key(ch: str) -> None:
        mode = {"f": FOLLOW, "h": RTH, " ": HOLD}.get(ch)
        if mode and not supervisor.request(mode):
            log.warning("FOLLOW refused: failsafe active")

    # Which cameras to track on: ask the source for one frame set.
    first = None
    for _ in range(200):
        first = camera.read_all()
        if first is not None:
            break
        time.sleep(0.05)
    names = list(first.frames) if first is not None else ["front"]
    if args.cams == "front":
        names = ["front"]
    log.info("cameras: %s", ", ".join(names))

    # What the tracker draws its box around: YOLO boxes the whole person (feet
    # to head, 1.8 m). The colour tracker in Unity boxes only the red shirt and
    # arms, 0.88-1.53 m above the ground. Getting this wrong makes the drone
    # think the person is ~3x further away than they are, so it flies on top.
    if args.tracker == "color" and args.mode == "unity":
        span = (0.65, 0.88)
    else:
        span = (args.target_height, 0.0)
    fcfg = FollowConfig(desired_box_height=args.box_height, lateral_speed=args.orbit,
                        follow_distance=args.distance, rig=rig_geometry(FOUR_CAMERA_RIG, *span))
    follow = FollowTarget(TargetSelector(TargetSelectorConfig(target_class=args.target_class)), fcfg,
                          TargetFilter(TargetFilterConfig(latency_s=args.latency)))
    follow.appearance = build_appearance(args)
    depth_estimator = None
    if args.mode == "real" and args.depth_model and not args.no_map:
        from drone_core.adapters.depth.depth_anything import DepthAnythingEstimator
        depth_estimator = DepthAnythingEstimator(args.depth_model, device=args.device)
    smap = None
    if not args.no_map and (args.mode != "real" or depth_estimator is not None):
        smap = SurroundMap(follow.cfg.rig)
    # 3-D map from the sonar array (and depth): lets avoidance go over/under branches.
    vmap = VoxelMap(follow.cfg.rig) if not args.no_map and args.mode != "real" else None

    hud = None
    if not args.no_hud:
        from drone_core.adapters.hud import HudWindow
        hud = HudWindow(on_key=on_key, surround_map=smap, follow=follow, voxel_map=vmap, map3d=args.hud_map)
    recorder = None
    if args.log:
        from drone_core.adapters.recorder import CsvRecorder
        recorder = CsvRecorder(args.log)

    last_log = [0.0]
    def on_tick(r):
        if hud:
            hud(r)
        if recorder:
            recorder(r)
        now = time.monotonic()
        if now - last_log[0] > 1.0:
            last_log[0] = now
            c = r.status.command
            log.info("%-9s tracks=%d cmd fwd=%+.2f right=%+.2f up=%+.2f yaw=%+.2f",
                     r.status.state, len(r.tracks), c.forward, c.right, c.up, c.yaw_rate)
        if pace:
            time.sleep(pace)

    mission = FollowMission(camera, build_trackers(args, names), commander, follow,
                            AvoidanceFilter(AvoidanceConfig(vertical_dodge=vmap is not None)), telemetry, clock,
                            on_tick, supervisor=supervisor, surround_map=smap, depth_estimator=depth_estimator,
                            voxel_map=vmap)
    try:
        mission.run(should_stop=lambda: bool(hud and hud.quit))
    except KeyboardInterrupt:
        pass
    finally:
        commander.close()
        camera.close()
        if recorder:
            recorder.close()


if __name__ == "__main__":
    main()
