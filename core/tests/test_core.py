import math
import os
import sys
import threading
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from drone_core.adapters.mock.sim import (ColorBlobTracker, MockCamera, MockClock, MockCommander,  # noqa: E402
                                          MockTelemetry, MockWorld, MockWorldConfig, circle_path,
                                          straight_path)
from drone_core.domain.models import BoundingBox, Telemetry, Track, VelocityCommand  # noqa: E402
from drone_core.ports import CameraSource, DroneCommander, ObjectTracker, TelemetrySource  # noqa: E402
from drone_core.usecases.avoidance import AvoidanceFilter  # noqa: E402
from drone_core.usecases.follow_mission import FollowMission  # noqa: E402
from drone_core.usecases.follow_target import FollowConfig, FollowTarget  # noqa: E402
from drone_core.usecases.pid import PID  # noqa: E402
from drone_core.usecases.target_selection import TargetSelector  # noqa: E402

W, H = 640, 480


def track(cx, cy, h, tid=1, cls="person"):
    return Track(tid, BoundingBox(cx - h / 4, cy - h / 2, cx + h / 4, cy + h / 2), cls, 0.9)


# ---------------------------------------------------------------- unit tests
def test_pid_clamps_and_converges_sign():
    pid = PID(kp=2.0, ki=1.0, out_limit=1.0)
    assert pid.update(10.0, 0.1) == 1.0
    assert pid.update(-10.0, 0.1) == -1.0


def test_follow_signs():
    """Target right/above/far -> yaw right, climb, go forward."""
    f = FollowTarget(TargetSelector(), FollowConfig(desired_box_height=0.4))
    st = f.step([track(500, 100, 60)], W, H, 0.0)
    assert st.state == "TRACKING"
    assert st.command.yaw_rate > 0 and st.command.up > 0 and st.command.forward > 0


def test_follow_backs_off_when_too_close():
    f = FollowTarget(TargetSelector(), FollowConfig(desired_box_height=0.3))
    st = f.step([track(320, 240, 400)], W, H, 0.0)
    assert st.command.forward < 0


def test_lost_then_search_then_idle():
    cfg = FollowConfig(lost_hover_s=1.0, search_s=2.0)
    f = FollowTarget(TargetSelector(), cfg)
    f.step([track(600, 240, 100)], W, H, 0.0)
    assert f.step([], W, H, 0.5).state == "COASTING"   # Kalman prediction bridges the gap
    assert f.step([], W, H, 0.8).state == "LOST"
    s = f.step([], W, H, 1.5)
    assert s.state == "SEARCHING" and s.command.yaw_rate > 0  # searches toward last-seen side (right)
    assert f.step([], W, H, 3.5).state == "IDLE"


def test_selector_keeps_lock_and_ignores_strangers():
    sel = TargetSelector()
    assert sel.select([track(100, 240, 200, tid=1), track(500, 240, 100, tid=2)], W, H).track_id == 1
    # id 1 disappears, a new id appears near the old position -> re-lock on it
    assert sel.select([track(110, 240, 200, tid=7), track(500, 240, 100, tid=2)], W, H).track_id == 7
    # only a far-away person -> don't jump to them
    assert sel.select([track(600, 240, 100, tid=2)], W, H) is None


def test_selector_filters_class():
    sel = TargetSelector()
    assert sel.select([track(320, 240, 200, cls="car")], W, H) is None


def test_avoidance_blocks_forward_near_wall():
    av = AvoidanceFilter()
    t = Telemetry(0, altitude=3, ranges={"front": 1.0, "down": 3})
    out = av.apply(VelocityCommand(forward=2.0, yaw_rate=0.3), t)
    assert out.forward == 0.0 and out.yaw_rate == 0.3 and abs(out.right) > 1.0   # stops and sidesteps
    t = Telemetry(0, altitude=3, ranges={"front": 2.75})
    assert AvoidanceFilter().apply(VelocityCommand(forward=2.0), t).forward == pytest.approx(1.0)
    # steers toward the open side
    t = Telemetry(0, altitude=3, ranges={"front": 2.0, "front_left": 2.5})
    assert AvoidanceFilter().apply(VelocityCommand(forward=2.0), t).right > 0
    t = Telemetry(0, altitude=3, ranges={"front": 2.0, "front_right": 2.5})
    assert AvoidanceFilter().apply(VelocityCommand(forward=2.0), t).right < 0
    t = Telemetry(0, altitude=0.8, ranges={"left": 0.6})
    out = av.apply(VelocityCommand(up=-1.0), t)
    assert out.up == 0.0 and out.right > 0


def test_mock_adapters_satisfy_ports():
    w = MockWorld()
    assert isinstance(MockCamera(w), CameraSource)
    assert isinstance(MockCommander(w), DroneCommander)
    assert isinstance(MockTelemetry(w), TelemetrySource)
    assert isinstance(ColorBlobTracker(), ObjectTracker)


def test_color_tracker_matches_ground_truth():
    w = MockWorld(MockWorldConfig(noise=0))
    frame = MockCamera(w).read()
    [t] = ColorBlobTracker().update(frame)
    gt = w.target_box()
    assert abs(t.box.cx - gt.cx) < 3 and abs(t.box.h - gt.h) < 3


# --------------------------------------------------------- closed-loop tests
def run_closed_loop(world, seconds, cfg=None, tracker=None, supervisor=None, mission_out=None):
    follow = FollowTarget(TargetSelector(), cfg or FollowConfig(desired_box_height=0.30))
    m = FollowMission(MockCamera(world), tracker or ColorBlobTracker(), MockCommander(world), follow,
                      AvoidanceFilter(), MockTelemetry(world), MockClock(world), supervisor=supervisor)
    if mission_out is not None:
        mission_out.append(m)
    log = []
    for _ in range(int(seconds * world.cfg.fps)):
        r = m.tick()
        log.append((world.t, r.status.state, world.distance_to_target(), r.status.errors.get("ex", 0.0)))
    return log


def steady(log, from_t):
    tail = [e for e in log if e[0] >= from_t]
    return tail, np.mean([d for _, _, d, _ in tail]), np.max([abs(ex) for *_, ex in tail])


@pytest.mark.parametrize("wind", [(0, 0, 0), (0.6, 0.0, 0.0), (0.0, -0.5, -0.2)])
def test_follows_circling_target(wind):
    world = MockWorld(MockWorldConfig(wind=wind, target_path=circle_path(radius=6, speed=1.2)))
    log = run_closed_loop(world, 40)
    tail, mean_d, max_ex = steady(log, 15)
    assert all(s == "TRACKING" for _, s, _, _ in tail)
    # desired_box_height 0.30 at hfov 90 -> ~4 m slant range to a 1.8 m person
    assert 2.5 < mean_d < 5.5, mean_d
    assert max_ex < 0.2, max_ex


def test_follows_straight_running_target():
    world = MockWorld(MockWorldConfig(target_path=straight_path(speed=2.5)))
    log = run_closed_loop(world, 30)
    tail, mean_d, _ = steady(log, 15)
    assert all(s == "TRACKING" for _, s, _, _ in tail)
    assert 2.5 < mean_d < 5.5, mean_d


def test_hovers_when_target_hidden():
    world = MockWorld()
    run_closed_loop(world, 5)
    world.target_visible_override = False
    log = run_closed_loop(world, 0.5)  # fresh follow instance -> IDLE immediately
    assert log[-1][1] == "IDLE" and world.cmd == VelocityCommand()


# ------------------------------------------------- harder, realistic conditions
def test_filter_smooths_and_coasts():
    from drone_core.usecases.target_filter import TargetFilter
    rng = np.random.default_rng(0)
    f = TargetFilter()
    raw, smooth = [], []
    for i in range(200):
        t = i * 0.05
        true_cx = 320 + 100 * math.sin(0.5 * t)
        z = true_cx + rng.normal(0, 12)
        est = f.update(BoundingBox(z - 30, 180, z + 30, 300), W, H, t)
        if i > 20:
            raw.append(abs(z - true_cx)); smooth.append(abs(est.cx * W - true_cx))
    assert np.mean(smooth) < 0.75 * np.mean(raw)
    # no detections: keeps predicting for coast_s, then gives up
    assert f.update(None, W, H, 200 * 0.05 + 0.3) is not None
    assert f.update(None, W, H, 200 * 0.05 + 1.0) is None


def test_follows_with_noisy_detector_dropouts_and_latency():
    """Jittery boxes, 20% missed frames in bursts of 3, id switches, 100 ms camera lag, wind."""
    from drone_core.adapters.mock.sim import NoisyTracker
    world = MockWorld(MockWorldConfig(latency_frames=2, wind=(0.4, 0.2, 0.0),
                                      target_path=circle_path(radius=6, speed=1.5)))
    tracker = NoisyTracker(ColorBlobTracker(), jitter_frac=0.05, drop_prob=0.07, burst_len=3,
                           id_switch_prob=0.02)
    log = run_closed_loop(world, 60, tracker=tracker)
    tail, mean_d, max_ex = steady(log, 15)
    states = [s for _, s, _, _ in tail]
    assert set(states) <= {"TRACKING", "COASTING"}, set(states)   # never drops to LOST
    assert 2.5 < mean_d < 5.5, mean_d
    assert max_ex < 0.25, max_ex
    d = [x for _, _, x, _ in tail]
    assert np.std(d) < 0.6, np.std(d)                             # holds distance steadily


def test_goes_around_pillar_between_drone_and_target():
    # target walks straight north; a 1 m-radius pillar sits on the drone's path
    world = MockWorld(MockWorldConfig(target_path=straight_path(speed=1.5, heading_deg=0.0),
                                      obstacles=((0.4, 14.0, 1.0),)))
    log = run_closed_loop(world, 40)
    assert world.pos[1] > 30, world.pos                 # got past it
    assert world.min_clearance > 0.3, world.min_clearance   # without touching it
    assert log[-1][1] == "TRACKING" and log[-1][2] < 6.0


def test_operator_rth_returns_home_and_holds():
    from drone_core.usecases.flight_supervisor import RTH, FlightSupervisor
    world = MockWorld(MockWorldConfig(target_path=straight_path(speed=2.0)))
    sup = FlightSupervisor()
    run_closed_loop(world, 15, supervisor=sup)
    assert math.hypot(world.pos[0], world.pos[1]) > 15       # followed the target away
    assert sup.request(RTH)
    log = run_closed_loop(world, 40, supervisor=sup)
    assert math.hypot(world.pos[0], world.pos[1]) < 1.5, world.pos
    assert log[-1][1].startswith("HOLD") and world.cmd == VelocityCommand()


def test_low_battery_forces_rth_and_refuses_follow():
    from drone_core.usecases.flight_supervisor import FOLLOW, FlightSupervisor
    world = MockWorld(MockWorldConfig(target_path=straight_path(speed=2.0), battery_drain_per_s=0.08))
    sup = FlightSupervisor()
    log = run_closed_loop(world, 12, supervisor=sup)        # battery hits 20% at t = 10 s
    assert log[-1][1].startswith("RTH (low battery)"), log[-1][1]
    assert sup.request(FOLLOW) is False


def test_geofence_forces_rth():
    from drone_core.usecases.flight_supervisor import FlightSupervisor, SupervisorConfig
    world = MockWorld(MockWorldConfig(target_path=straight_path(speed=2.5)))
    sup = FlightSupervisor(SupervisorConfig(geofence_radius=25.0))
    log = run_closed_loop(world, 40, supervisor=sup)
    assert any(s.startswith("RTH (geofence)") for _, s, _, _ in log)
    assert math.hypot(world.pos[0], world.pos[1]) < 26.0


def test_ground_position_estimate_is_accurate():
    from drone_core.usecases.target_geo import CameraGeometry, box_to_world
    world = MockWorld(MockWorldConfig(noise=0, target_path=circle_path(radius=6, speed=1.2)))
    run_closed_loop(world, 5)
    frame = MockCamera(world).read()
    [t] = ColorBlobTracker().update(frame)
    p = box_to_world(t.box, W, H, world.telemetry(), CameraGeometry())
    tp = world.target_pos()
    assert math.hypot(p[0] - tp[0], p[1] - tp[1]) < 0.8


def test_relock_by_predicted_position_ignores_decoy():
    """After an occlusion the tracker gives the target a new id. The selector
    must pick the candidate near the predicted ground spot, not the bigger decoy."""
    sel = TargetSelector()
    sel.select([track(320, 240, 150, tid=1)], W, H)
    gate = lambda t: {7: 12.0, 9: 1.0}[t.track_id]   # metres from prediction
    chosen = sel.select([track(500, 240, 300, tid=7), track(100, 240, 120, tid=9)], W, H, gate)
    assert chosen.track_id == 9
    assert sel.select([track(500, 240, 300, tid=7)], W, H, lambda t: 12.0) is None


@pytest.mark.parametrize("start,pillar", [((-8, 12), (-1, 9, 2.5)), ((-6, 12), (-1, 8, 2.0))])
def test_reacquires_after_target_walks_behind_pillar(start, pillar):
    """Target walks east behind a 4-5 m wide pillar. The old behaviour (hover,
    then spin in place) never saw it again; now the drone flies toward where it
    should be, around the pillar, and picks it back up."""
    world = MockWorld(MockWorldConfig(target_path=lambda t: (start[0] + 1.5 * t, start[1]),
                                      obstacles=(pillar,)))
    world.yaw = math.atan2(start[0], start[1])
    log = run_closed_loop(world, 35)
    states = [s for _, s, _, _ in log]
    assert "REACQUIRING" in states                         # it really was hidden
    hidden = sum(s not in ("TRACKING", "COASTING") for s in states) / world.cfg.fps
    assert hidden < 10.0, hidden
    assert states[-1] == "TRACKING" and log[-1][2] < 5.0, (states[-1], log[-1][2])
    assert world.min_clearance > 0.3                       # never touched the pillar


# ---------------------------------------------- unity adapter over real TCP
def test_unity_adapter_end_to_end_with_fake_server():
    from drone_core.adapters.unity.bridge import UnityBridge, UnityCamera, UnityCommander, UnityTelemetry
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
    from fake_unity import run_server

    stop, ready = threading.Event(), threading.Event()
    world = MockWorld()
    port = 55871
    threading.Thread(target=run_server, args=("127.0.0.1", port, 30, stop, world, ready), daemon=True).start()
    ready.wait(5)
    bridge = UnityBridge("127.0.0.1", port, reconnect_s=0.2).start()
    try:
        assert bridge.wait_connected(5)
        cam, cmd, tel = UnityCamera(bridge), UnityCommander(bridge), UnityTelemetry(bridge)
        deadline = time.time() + 5
        while cam.read() is None and time.time() < deadline:
            time.sleep(0.02)
        f = cam.read()
        assert f is not None and f.image.shape == (480, 640, 3)
        assert tel.latest().altitude == pytest.approx(world.pos[2], abs=0.2)
        assert len(ColorBlobTracker().update(f)) == 1
        from drone_core.adapters.unity.bridge import UnityMultiCamera
        mf = UnityMultiCamera(bridge).read_all()
        assert set(mf.frames) == {"front", "right", "back", "left"}
        assert set(mf.depth) == set(mf.frames) and mf.depth["front"].ndim == 2
        cmd.send_velocity(VelocityCommand(0.0, 0.0, 0.5, 0.2))
        time.sleep(0.3)
        assert world.cmd == VelocityCommand(0.0, 0.0, 0.5, 0.2)
    finally:
        bridge.close()
        stop.set()


# ------------------------------------------------ 4-camera rig + surround map
def run_rig_loop(world, seconds, cams=("front", "right", "back", "left"), smap=None, depth=False):
    from drone_core.adapters.mock.sim import MockMultiCamera
    from drone_core.domain.models import FOUR_CAMERA_RIG
    mounts = [m for m in FOUR_CAMERA_RIG if m.name in cams]
    follow = FollowTarget(TargetSelector(), FollowConfig(desired_box_height=0.30))
    m = FollowMission(MockMultiCamera(world, mounts=mounts, with_depth=depth),
                      {n: ColorBlobTracker() for n in cams}, MockCommander(world), follow,
                      AvoidanceFilter(), MockTelemetry(world), MockClock(world), surround_map=smap)
    log = []
    for _ in range(int(seconds * world.cfg.fps)):
        r = m.tick()
        log.append((world.t, r.status.state, world.distance_to_target()))
    return log


def test_turns_to_target_seen_only_by_back_camera():
    """Target starts behind the drone and walks away. Front-only never sees it;
    with the rig the drone turns round and follows."""
    path = lambda t: (0.0, -6.0 - 1.0 * t)          # south, behind a north-facing drone
    w4 = MockWorld(MockWorldConfig(target_path=path))
    log4 = run_rig_loop(w4, 25)
    states = [s for _, s, _ in log4]
    assert any(s.startswith("TURNING") for s in states)
    assert states[-1] == "TRACKING" and log4[-1][2] < 6.0, (states[-1], log4[-1][2])
    w1 = MockWorld(MockWorldConfig(target_path=path))
    log1 = run_rig_loop(w1, 10, cams=("front",))
    assert log1[-1][1] == "IDLE"


def test_surround_map_sees_pillars_and_feeds_avoidance():
    """No rangefinders at all: the only obstacle info is the map built from the
    cameras' depth. The drone must still go round the pillar."""
    from drone_core.domain.models import FOUR_CAMERA_RIG
    from drone_core.usecases.surround_map import SurroundMap
    world = MockWorld(MockWorldConfig(target_path=straight_path(speed=1.5, heading_deg=0.0),
                                      obstacles=((0.4, 14.0, 1.0),), range_sensors=False))
    smap = SurroundMap(FollowConfig().rig)
    log = run_rig_loop(world, 35, smap=smap, depth=True)
    assert smap.occupied().any()
    assert world.pos[1] > 25, world.pos
    assert world.min_clearance > 0.3, world.min_clearance
    assert log[-1][1] == "TRACKING"


def test_protocol_multi_frame_round_trip():
    import cv2
    from drone_core.adapters.unity.protocol import MSG_FRAME, decode_frame, encode_multi_frame, read_message
    imgs = {n: np.full((48, 64, 3), i * 60, np.uint8) for i, n in enumerate(["front", "right", "back", "left"])}
    jpegs = {n: cv2.imencode(".jpg", im)[1].tobytes() for n, im in imgs.items()}
    depth = {"front": np.array([[1.5, np.inf], [3.0, 4.0]], np.float32)}
    msg = encode_multi_frame({"t": 1.0, "frame": 3, "pos": [1, 2, 3], "yaw": 0.5, "alt": 3.0},
                             jpegs, {"front": 0, "right": 90, "back": 180, "left": -90}, depth)
    import socket
    a, b = socket.socketpair()
    a.sendall(msg)
    typ, payload = read_message(b)
    a.close(); b.close()
    mf, tel = decode_frame(payload)
    assert typ == MSG_FRAME and list(mf.frames) == ["front", "right", "back", "left"]
    assert mf.frames["back"].camera == "back" and mf.frames["back"].image.shape == (48, 64, 3)
    d = mf.depth["front"]
    assert d[0, 0] == pytest.approx(1.5, abs=0.01) and np.isinf(d[0, 1])


# ---------------------------------------------------------------- sonar + 3-D map
def _sonar_loop(world, seconds, sonar=True):
    from drone_core.usecases.avoidance import AvoidanceConfig
    from drone_core.usecases.voxel_map import VoxelMap
    follow = FollowTarget(TargetSelector(), FollowConfig(desired_box_height=0.30))
    vm = VoxelMap() if sonar else None
    m = FollowMission(MockCamera(world), ColorBlobTracker(), MockCommander(world), follow,
                      AvoidanceFilter(AvoidanceConfig(vertical_dodge=sonar)), MockTelemetry(world),
                      MockClock(world), voxel_map=vm)
    states = [m.tick().status.state for _ in range(int(seconds * world.cfg.fps))]
    return states, vm


def test_sonar_hears_branches_but_misses_twigs():
    def front(radius):
        w = MockWorld(MockWorldConfig(sonar=True, branches=(((-6, 3, 3), (6, 3, 3), radius),)))
        return next(b.range for b in w.sonar().beams if b.yaw_deg == 0 and b.pitch_deg == 0)
    r = front(0.04)                     # 8 cm branch, 3 m ahead
    assert r is not None and abs(r - 2.96) < 0.15, r
    assert front(0.004) is None         # 8 mm twig: too small an echo, like real ultrasonic


def test_voxel_map_has_no_phantoms_and_finds_branch():
    from drone_core.usecases.voxel_map import VoxelMap
    w = MockWorld(MockWorldConfig(sonar=True))
    vm = VoxelMap()
    for _ in range(100):                # wander over open ground: the ground must not become obstacles
        w.cmd = VelocityCommand(forward=1.0, yaw_rate=0.3)
        w.step(0.05)
        vm.update(w.telemetry())
    assert vm.occupied().sum() == 0

    w = MockWorld(MockWorldConfig(sonar=True, branches=(((-4, 6, 3), (4, 6, 3), 0.05),)))
    vm = VoxelMap()
    for _ in range(60):                 # fly toward the branch, stop short of it
        w.cmd = VelocityCommand(forward=1.0 if w.pos[1] < 3.5 else 0.0)
        w.step(0.05)
        vm.update(w.telemetry())
    pts = vm.occupied_points()
    assert len(pts) > 5
    # A sonar cone only says "something at this distance, somewhere in the cone",
    # so a hovering drone smears the branch vertically; it must stay at the
    # right distance and centred on the branch.
    assert np.all(np.abs(pts[:, 1] - 6) < 0.6), pts[:, 1].min()
    assert np.all(np.abs(pts[:, 2] - 3) < 1.6) and abs(np.median(pts[:, 2]) - 3) < 0.5
    r = vm.ranges(w.telemetry())
    assert r.get("front", 99) < 3.5 and "front_up" not in r     # blocked ahead, open above


def test_avoidance_never_climbs_into_canopy_or_sinks_onto_branch():
    from drone_core.usecases.avoidance import AvoidanceConfig
    av = AvoidanceFilter(AvoidanceConfig(vertical_dodge=True))
    tm = Telemetry(0.0, altitude=4.0, ranges={"up": 0.6})
    assert av.apply(VelocityCommand(up=1.0), tm).up == 0.0
    tm = Telemetry(0.0, altitude=4.0, ranges={"below": 0.6})
    assert av.apply(VelocityCommand(up=-1.0), tm).up == 0.0
    # branch ahead, open above -> climb (not sidestep)
    tm = Telemetry(0.0, altitude=3.0, ranges={"front": 2.0, "front_left": 2.8, "front_right": 2.8})
    out = av.apply(VelocityCommand(forward=2.0), tm)
    assert out.up > 0.4 and abs(out.right) < 1e-9 and out.forward < 1.0


def test_climbs_over_branch_that_camera_depth_misses():
    """A branch across the path at flight height. The cameras' depth doesn't
    see it (too thin); without sonar the drone flies into it, with sonar it
    climbs over and keeps following."""
    cfg = dict(target_path=straight_path(speed=1.5, heading_deg=0.0), range_sensors=False,
               branches=(((-10, 14, 2.8), (10, 14, 2.8), 0.04),))
    blind = MockWorld(MockWorldConfig(**cfg))
    _sonar_loop(blind, 15, sonar=False)
    assert blind.min_branch_clearance < 0.1
    w = MockWorld(MockWorldConfig(sonar=True, **cfg))
    states, vm = _sonar_loop(w, 25)
    assert w.min_branch_clearance > 0.4, w.min_branch_clearance
    assert w.pos[1] > 30 and states[-1] == "TRACKING", (w.pos, states[-1])


def test_sonar_drone_still_goes_around_tall_pillar():
    """Vertical dodging must not try to climb over something tall."""
    w = MockWorld(MockWorldConfig(target_path=straight_path(speed=1.5, heading_deg=0.0), range_sensors=False,
                                  obstacles=((0.4, 14.0, 1.0),), sonar=True))
    states, _ = _sonar_loop(w, 35)
    assert w.pos[1] > 25 and w.min_clearance > 0.3, (w.pos, w.min_clearance)
    assert w.pos[2] < 7.0 and states[-1] == "TRACKING"


def test_protocol_sonar_round_trip():
    from drone_core.adapters.unity.protocol import decode_sonar, encode_sonar
    from drone_core.domain.models import SonarBeam, SonarScan
    scan = SonarScan((SonarBeam(0.0, 0.0, 2.5), SonarBeam(90.0, 35.0, None)), 30.0, 0.2, 7.0)
    assert decode_sonar(encode_sonar(scan)) == scan


# ---------------------------------------------------------------- losing the target
def test_relock_after_loss_needs_confirmation():
    """After a real loss, a person at the predicted spot must be seen for
    confirm_s before the drone locks on again (a flash of a look-alike doesn't)."""
    f = FollowTarget(TargetSelector(), FollowConfig())
    tm = Telemetry(0.0, position=(0.0, 0.0, 3.0), altitude=3.0)
    t = 0.0
    for _ in range(20):                                   # 1 s of normal tracking
        t += 0.05
        assert f.step([track(320, 260, 140)], W, H, t, tm).state in ("TRACKING", "COASTING")
    for _ in range(30):                                   # 1.5 s hidden
        t += 0.05
        f.step([], W, H, t, tm)
    states = []
    for _ in range(16):                                   # reappears where expected, new id
        t += 0.05
        states.append(f.step([track(320, 260, 140, tid=5)], W, H, t, tm).state)
    assert "TRACKING" not in states[:8], states           # not before ~0.4 s
    assert states[-1] == "TRACKING", states


def test_climbs_for_a_better_view_when_line_of_sight_blocked():
    cfg = FollowConfig(los_max_altitude=8.0)
    f = FollowTarget(TargetSelector(), cfg)
    blocked = [True]
    f.line_of_sight = lambda a, b: blocked[0]
    t = 0.0
    def step(alt):
        nonlocal t
        t += 0.05
        return f.step([track(320, 300, 120)], W, H, t, Telemetry(t, position=(0.0, 0.0, alt), altitude=alt))
    out = step(3.0)
    out = step(3.0)
    assert out.command.up == pytest.approx(cfg.los_climb_speed) and "climbing" in out.errors["view"]
    out = step(cfg.los_max_altitude + 0.1)                # at the limit: stop climbing
    assert out.command.up <= cfg.max_z and out.errors["view"] == "blocked: at height limit"
    assert out.command.up < cfg.los_climb_speed or out.command.up <= 0.0
    blocked[0] = False                                    # view clear: hold height a while
    for _ in range(10):
        out = step(6.0)
        assert out.command.up >= 0.0


def test_voxel_line_of_sight():
    from drone_core.usecases.voxel_map import VoxelMap
    w = MockWorld(MockWorldConfig(sonar=True, branches=(((-4, 6, 2.5), (4, 6, 2.5), 0.06),)))
    vm = VoxelMap()
    for _ in range(60):
        w.cmd = VelocityCommand(forward=1.0 if w.pos[1] < 3.5 else 0.0)
        w.step(0.05)
        vm.update(w.telemetry())
    eye = tuple(w.pos)
    assert vm.line_blocked(eye, (0.0, 9.0, 2.05))         # passes through the branch
    assert not vm.line_blocked((0.0, w.pos[1], 6.0), (0.0, 9.0, 6.0))   # well above it


def test_lost_target_circle_search_then_wait_then_home():
    from drone_core.usecases.flight_supervisor import FlightSupervisor
    cfg = FollowConfig(desired_box_height=0.30, reacquire_s=3.0, orbit_s=14.0, loiter_s=6.0)
    world = MockWorld(MockWorldConfig(target_path=straight_path(speed=1.0, heading_deg=0.0)))
    follow = FollowTarget(TargetSelector(), cfg)
    m = FollowMission(MockCamera(world), ColorBlobTracker(), MockCommander(world), follow,
                      AvoidanceFilter(), MockTelemetry(world), MockClock(world), supervisor=FlightSupervisor())
    log = []
    for i in range(int(45 * world.cfg.fps)):
        if i == int(6 * world.cfg.fps):
            world.target_visible_override = False        # the person vanishes
        r = m.tick()
        log.append((world.t, r.status.state, world.pos.copy(), dict(r.status.errors)))
    states = [s for _, s, _, _ in log]
    assert "SEARCHING" in states and "LOITER" in states, sorted(set(states))
    # the circle really goes round: bearings from its centre cover most of 360 deg
    orbit = [(p, e) for _, s, p, e in log if s == "SEARCHING"]
    assert orbit[-1][1]["laps"] > 0.9, orbit[-1][1]
    # it waits at the last-seen spot, then the supervisor brings it home
    loiter_pos = [p for _, s, p, _ in log if s == "LOITER"][-1]
    lost_at = [p for _, s, p, _ in log if s == "TRACKING"][-1]
    assert math.hypot(loiter_pos[0] - lost_at[0], loiter_pos[1] - (lost_at[1] + 4.0)) < 3.0, (loiter_pos, lost_at)
    assert any(s.startswith("RTH (target lost)") for s in states), sorted(set(states))
    assert states[-1] == "HOLD (arrived home)" and math.hypot(*world.pos[:2]) < 1.5


# ------------------------------------------------- don't fly over the target
def test_ground_range_is_accurate_from_any_height():
    from drone_core.usecases.target_geo import CameraGeometry, ground_range
    for alt in (2.0, 4.0, 7.0):
        for dist in (2.5, 4.0, 7.0):
            for base, h in ((0.0, 1.8), (0.88, 0.65)):
                w = MockWorld(MockWorldConfig(noise=0, drone_start=(0, 0, alt), target_base=base,
                                              target_size=(0.5, 0.4, h), target_path=lambda t, d=dist: (0.0, d)))
                box = w.target_box()
                if box is None:
                    continue
                r, _ = ground_range(box, W, H, w.telemetry(), CameraGeometry(target_height_m=h, target_bottom_m=base))
                assert abs(r - dist) < 0.25 * dist + 0.2, (alt, dist, base, r)


def _min_gap(world, seconds=15, cfg=None):
    log = run_closed_loop(world, seconds, cfg=cfg)
    tail = [e for e in log if e[0] > 3]
    return min(e[2] for e in tail), {s for _, s, _, _ in tail}


def test_high_drone_does_not_fly_over_standing_target():
    # Box height alone shrinks when looking down (foreshortening): the old
    # controller read that as "too far" and ended up right above the person.
    gap, states = _min_gap(MockWorld(MockWorldConfig(noise=0, drone_start=(0, 0, 7), target_path=lambda t: (0.0, 6.0))))
    assert gap > 3.0 and states == {"TRACKING"}, (gap, states)


def test_backs_off_when_target_walks_at_drone():
    gap, states = _min_gap(MockWorld(MockWorldConfig(
        noise=0, drone_start=(0, 0, 6), target_path=straight_path(speed=1.2, start=(0, 10), heading_deg=180))))
    assert gap > 1.8 and states == {"TRACKING"}, (gap, states)


def test_shirt_only_tracker_keeps_distance():
    # Unity's colour tracker boxes only the red shirt (0.88-1.53 m). Told so,
    # the drone keeps its normal distance instead of flying on top.
    from drone_core.domain.models import FOUR_CAMERA_RIG
    from drone_core.usecases.target_geo import rig_geometry
    cfg = FollowConfig(desired_box_height=0.30, rig=rig_geometry(FOUR_CAMERA_RIG, 0.65, 0.88))
    world = MockWorld(MockWorldConfig(noise=0, target_base=0.88, target_size=(0.45, 0.25, 0.65),
                                      target_path=circle_path(radius=6, speed=1.2)))
    log = run_closed_loop(world, 30, cfg=cfg)
    tail = [e for e in log if e[0] > 10]
    assert {s for _, s, _, _ in tail} == {"TRACKING"}
    assert 3.0 < np.mean([e[2] for e in tail]) < 6.0 and min(e[2] for e in tail) > 2.5


def test_view_climb_and_distance_hold_work_together():
    # Line of sight blocked for 6 s: the drone climbs for a view, and while
    # higher up it still holds its distance instead of drifting over the person.
    world = MockWorld(MockWorldConfig(noise=0, target_path=circle_path(radius=6, speed=1.0)))
    out = []
    follow = FollowTarget(TargetSelector(), FollowConfig(desired_box_height=0.30, los_max_altitude=8.0))
    m = FollowMission(MockCamera(world), ColorBlobTracker(), MockCommander(world), follow,
                      AvoidanceFilter(), MockTelemetry(world), MockClock(world))
    follow.line_of_sight = lambda a, b: 5.0 < world.t < 11.0
    for _ in range(int(25 * world.cfg.fps)):
        r = m.tick()
        out.append((world.t, r.status.state, world.distance_to_target(), world.pos[2]))
    tail = [e for e in out if e[0] > 4]
    assert {s for _, s, _, _ in tail} == {"TRACKING"}
    assert max(e[3] for e in tail) > 5.0                 # climbed
    assert min(e[2] for e in tail) > 2.5                 # never over the top of the person


def test_hud_draws_labels_top_down_and_3d_map():
    from drone_core.adapters.hud import draw_map3d, draw_rig
    from drone_core.adapters.mock.sim import MockMultiCamera
    from drone_core.domain.models import FOUR_CAMERA_RIG
    from drone_core.usecases.voxel_map import VoxelMap
    world = MockWorld(MockWorldConfig(sonar=True, obstacles=((2.0, 9.0, 0.5),)))
    follow = FollowTarget(TargetSelector(), FollowConfig(desired_box_height=0.30))
    vm = VoxelMap(follow.cfg.rig)
    m = FollowMission(MockMultiCamera(world, mounts=list(FOUR_CAMERA_RIG)),
                      {c.name: ColorBlobTracker() for c in FOUR_CAMERA_RIG}, MockCommander(world), follow,
                      AvoidanceFilter(), MockTelemetry(world), MockClock(world), voxel_map=vm)
    for _ in range(40):
        r = m.tick()
    img = draw_rig(r, None, follow, vm, view_yaw_deg=90)
    assert img.shape[1] == 640 + 2 * img.shape[0]          # cameras + top-down + 3D
    assert draw_map3d(r, None, follow).shape == (480, 480, 3)


# ------------------------------------------------ appearance memory (re-ID)
def _people_run(name, reid):
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
    import benchmark as B
    sc = next(s for s in B.SCENARIOS if s.name == name)
    return B.run_scenario(sc, reid)


@pytest.mark.parametrize("name", ["decoy_at_exit", "crowd_walk"])
def test_appearance_memory_keeps_the_right_person(name):
    """Look-alike decoys and tracker id swaps: by position alone the drone ends
    up following someone else; with the appearance memory it doesn't (for long)."""
    without = _people_run(name, "none")
    with_ = _people_run(name, "hist")
    assert without["wrong_pct"] > 8.0, without            # the scenario really is a trap
    assert with_["wrong_pct"] < 5.0, with_
    assert with_["on_target_pct"] > without["on_target_pct"], (with_, without)


def test_histogram_encoder_tells_trousers_apart():
    from drone_core.adapters.appearance.color_hist import HistogramEncoder
    from drone_core.adapters.mock.sim import MockPerson
    RED, BLUE, GREEN = (30, 30, 220), (160, 60, 30), (40, 140, 40)
    w = MockWorld(MockWorldConfig(noise=2, target_colors=(RED, BLUE), target_path=lambda t: (-1.0, 6.0),
                                  people=(MockPerson(lambda t: (1.5, 7.0), RED, GREEN),)))
    vis = {i: b for i, b, _ in w.visible_people()}
    e = HistogramEncoder().embed(w.render(), [vis[0], vis[1]])
    w2 = MockWorld(MockWorldConfig(noise=2, target_colors=(RED, BLUE), target_path=lambda t: (1.0, 9.0),
                                   drone_start=(0.5, 0, 4)))
    [(_, b2, _)] = w2.visible_people()
    [e2] = HistogramEncoder().embed(w2.render(), [b2])
    assert e2 @ e[0] > 0.9 > e2 @ e[1]


@pytest.mark.skipif(not os.path.exists(os.path.join(os.path.dirname(__file__), "..", "yolo26n-reid.onnx")),
                    reason="yolo26n-reid.onnx not downloaded")
def test_reid_network_encoder_runs():
    pytest.importorskip("onnxruntime")
    from drone_core.adapters.appearance.onnx_reid import OnnxReidEncoder
    enc = OnnxReidEncoder(os.path.join(os.path.dirname(__file__), "..", "yolo26n-reid.onnx"))
    w = MockWorld(MockWorldConfig(noise=0))
    [v] = enc.embed(w.render(), [w.target_box()])
    assert v.shape == (512,) and abs(np.linalg.norm(v) - 1) < 1e-3


def test_command_carries_ghost_estimate():
    import json
    from drone_core.adapters.unity import protocol
    msg = protocol.encode_command(VelocityCommand(1, 0, 0, 0), {"x": 1.5, "y": -2.0, "h": 1.8, "r": 4.2,
                                                                "state": "REACQUIRING"})
    body = json.loads(msg[5:])
    assert body["est"] == {"has": 1, "x": 1.5, "y": -2.0, "h": 1.8, "r": 4.2, "state": "REACQUIRING"}
    assert "est" not in json.loads(protocol.encode_command(VelocityCommand())[5:])
    f = FollowTarget(TargetSelector(), FollowConfig())
    tm = Telemetry(0.0, position=(0.0, 0.0, 3.0), altitude=3.0)
    t = 0.0
    for _ in range(10):
        t += 0.05
        f.step([track(320, 260, 140)], W, H, t, tm)
    e = f.estimate(t, "TRACKING")
    assert e["state"] == "TRACKING" and e["r"] == 0.5 and 2.0 < e["y"] < 8.0
    for _ in range(40):
        t += 0.05
        f.step([], W, H, t, tm)
    assert f.estimate(t)["r"] > 4.0                        # less sure while hidden


def test_map_is_streamed_to_unity():
    import struct
    from drone_core.adapters.unity import protocol
    pts = np.array([[1.0, 2.0, 3.0], [-4.5, 0.2, 0.1]])
    msg = protocol.encode_map(pts, 0.2)
    assert msg[4] == protocol.MSG_MAP
    n, res = struct.unpack("<If", msg[5:13])
    back = np.frombuffer(msg[13:], "<i2").reshape(-1, 3) / 100.0
    assert n == 2 and abs(res - 0.2) < 1e-6 and np.allclose(back, pts)

    from drone_core.usecases.voxel_map import VoxelMap
    world = MockWorld(MockWorldConfig(sonar=True, obstacles=((2.0, 8.0, 0.6),)))
    sent = []
    class Cmd(MockCommander):
        def show_map(self, points, resolution):
            sent.append((world.t, len(points)))
    m = FollowMission(MockCamera(world), ColorBlobTracker(), Cmd(world), FollowTarget(TargetSelector()),
                      AvoidanceFilter(), MockTelemetry(world), MockClock(world), voxel_map=VoxelMap())
    for _ in range(60):                                   # 3 s
        m.tick()
    assert 5 <= len(sent) <= 7 and sent[-1][1] > 0        # ~2 Hz, with blocks in it
