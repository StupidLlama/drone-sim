"""Performance benchmark: fly fixed scenarios in the mock sim and score them.

    python tools/benchmark.py --version v1              # run, save, compare
    python tools/benchmark.py --version v1 --reid none  # without appearance memory
    python tools/benchmark.py --only decoy_at_exit,crossing_swap

Results go to benchmarks/<version>.json and benchmarks/REPORT.md (all saved
versions side by side). Same seeds every run, so differences come from code.

Metrics (after a 3 s warm-up):
  on_target %   time locked on the REAL target (green)
  wrong %       time locked on someone else (red: the bad one)
  lost %        time with no target at all
  dist m        mean / min ground distance to the real target
  close s       seconds closer than 2 m
  relock s      first loss -> back on the real target (blank = never lost, "never" = didn't)
  clear m       closest approach to a pillar / branch surface
  tick ms       mean / 95th percentile time of one tick, INCLUDING the mock sim's own
                rendering (~15 ms of it); compare versions, not absolute values

Suite 2 (v2+, "occlusion suite"): walls, buildings and doors, a front depth
camera feeding the 3-D map, and each scenario in 2 variants (left/right,
stops/keeps walking...). Extra metrics while the target is lost:
  ghost m       mean distance from the ghost (main belief cluster) to the real person
  @5s / @10s    the same, 5 and 10 s into the first loss
  top3 %        share of lost time the person is within 2 m of one of the 3 heaviest clusters
  refind s      from the first moment the person is visible to the front camera again
                to back on them (blank = never lost, "never" = didn't)
Suite 1 (the original 11) keeps its old score so 92.7 stays comparable.

    python tools/benchmark.py --version v2 --suite all
    python tools/benchmark.py --version v1-suite2 --suite 2 --legacy   # v1 behaviour (no belief)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import platform
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable, Optional

# One BLAS thread per process. With --jobs every worker would otherwise reserve
# buffers for one thread per CPU core, and on Windows OpenBLAS runs out of memory
# ("OpenBLAS error: Memory allocation still failed"). Must be set before numpy loads.
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from drone_core.adapters.mock.sim import (ColorBlobTracker, MockCamera, MockClock, MockCommander,  # noqa: E402
                                          MockMultiCamera, MockPerson, MockPersonTracker, MockTelemetry, MockWorld,
                                          MockWorldConfig, NoisyTracker, circle_path, straight_path)
from drone_core.domain.models import FOUR_CAMERA_RIG  # noqa: E402
from drone_core.usecases.avoidance import AvoidanceConfig, AvoidanceFilter  # noqa: E402
from drone_core.usecases.follow_mission import FollowMission  # noqa: E402
from drone_core.usecases.follow_target import FollowConfig, FollowTarget  # noqa: E402
from drone_core.usecases.target_geo import rig_geometry  # noqa: E402
from drone_core.usecases.target_selection import TargetSelector  # noqa: E402

BENCH_DIR = os.path.join(HERE, "..", "..", "benchmarks")
RED, BLUE, GREEN, GREY, YELLOW = (30, 30, 220), (160, 60, 30), (40, 140, 40), (120, 120, 120), (40, 200, 220)


def _east_then_north(start, speed, turn_x, y, north_speed):
    """East along y to turn_x (behind the pillar), then away north."""
    t_turn = (turn_x - start) / speed
    return lambda t: (start + speed * t, y) if t < t_turn else (turn_x, y + north_speed * (t - t_turn))


def _walk_wait_walk(start, speed, stop_x, wait_s, y):
    """East along y, stop at stop_x for wait_s, then carry on."""
    t_stop = (stop_x - start) / speed
    def p(t):
        if t < t_stop:
            return (start + speed * t, y)
        if t < t_stop + wait_s:
            return (stop_x, y)
        return (stop_x + speed * (t - t_stop - wait_s), y)
    return p


def _route(start, *legs):
    """Piecewise path: legs are (x, y, speed) = walk there, or ("wait", s)."""
    pts = [(0.0, float(start[0]), float(start[1]))]
    t, x, y = 0.0, float(start[0]), float(start[1])
    for leg in legs:
        if leg[0] == "wait":
            t += leg[1]
        else:
            nx, ny, sp = leg
            t += math.hypot(nx - x, ny - y) / sp
            x, y = float(nx), float(ny)
        pts.append((t, x, y))
    def p(tt):
        if tt <= 0:
            return pts[0][1], pts[0][2]
        for (t0, x0, y0), (t1, x1, y1) in zip(pts, pts[1:]):
            if tt <= t1:
                f = 0.0 if t1 - t0 < 1e-9 else (tt - t0) / (t1 - t0)
                return x0 + f * (x1 - x0), y0 + f * (y1 - y0)
        return pts[-1][1], pts[-1][2]
    return p


def _wall(x0, y0, x1, y1, h=3.0, th=0.3):
    """A straight wall from (x0, y0) to (x1, y1), axis-aligned, as a box."""
    if abs(y1 - y0) < abs(x1 - x0):
        return (min(x0, x1), y0 - th / 2, max(x0, x1), y0 + th / 2, h)
    return (x0 - th / 2, min(y0, y1), x0 + th / 2, max(y0, y1), h)


@dataclass
class Scenario:
    name: str
    about: str
    seconds: float
    world: Callable                    # () -> MockWorldConfig, or (variant) -> ... for suite 2
    tracker: str = "people"            # people | color | noisy
    swap_prob: float = 0.0
    sonar: bool = False
    shirt_only: bool = False
    start_yaw_to_target: bool = False
    suite: int = 1
    variants: tuple = ()               # labels; world(i) builds variant i


SCENARIOS = [
    Scenario("circle_walk", "walks a 6 m circle at 1.2 m/s", 40,
             lambda: MockWorldConfig(target_colors=(RED, BLUE), target_path=circle_path(radius=6, speed=1.2))),
    Scenario("run_straight", "runs straight at 2.5 m/s", 30,
             lambda: MockWorldConfig(target_colors=(RED, BLUE), target_path=straight_path(speed=2.5))),
    Scenario("walks_at_drone", "walks straight at the drone (drone 6 m up)", 15,
             lambda: MockWorldConfig(target_colors=(RED, BLUE), drone_start=(0, 0, 6),
                                     target_path=straight_path(speed=1.2, start=(0, 10), heading_deg=180))),
    Scenario("high_start", "drone starts 7 m up, person stands still", 15,
             lambda: MockWorldConfig(target_colors=(RED, BLUE), drone_start=(0, 0, 7), target_path=lambda t: (0.0, 6.0))),
    Scenario("pillar_occlusion", "walks behind a 5 m wide pillar", 35,
             lambda: MockWorldConfig(target_colors=(RED, BLUE), obstacles=((-1, 9, 2.5),),
                                     target_path=lambda t: (-8 + 1.5 * t, 12)), start_yaw_to_target=True),
    Scenario("decoy_at_exit", "steps into a shop for 8 s; meanwhile someone in the SAME red shirt "
             "(green trousers) walks on exactly where the target was heading", 40,
             lambda: MockWorldConfig(target_colors=(RED, BLUE), target_hidden=((6.0, 14.0),),
                                     target_path=_walk_wait_walk(-6, 1.2, -6 + 1.2 * 6.0, 8.0, 10.0),
                                     people=(MockPerson(lambda t: (-6 + 1.2 * t, 10.3) if t >= 7.0 else (-6 + 1.2 * 7.0, 30.0),
                                                        RED, GREEN),))),
    Scenario("crossing_swap", "a look-alike (same shirt) crosses right in front; tracker swaps their ids", 30,
             lambda: MockWorldConfig(target_colors=(RED, BLUE), target_path=lambda t: (-3 + 0.5 * t, 8.0),
                                     people=(MockPerson(lambda t: (5 - 0.9 * t, 7.6), RED, GREEN),)),
             swap_prob=1.0),
    Scenario("crowd_walk", "circle walk with 3 other people wandering nearby (different clothes)", 40,
             lambda: MockWorldConfig(target_colors=(RED, BLUE), target_path=circle_path(radius=6, speed=1.2),
                                     people=(MockPerson(circle_path(radius=4, speed=0.8, center=(3, 9)), BLUE, GREY),
                                             MockPerson(circle_path(radius=5, speed=1.0, center=(-3, 12)), YELLOW, BLUE),
                                             MockPerson(straight_path(speed=0.7, start=(-6, 18), heading_deg=100),
                                                        RED, GREEN))), swap_prob=0.5),
    Scenario("noisy_detector", "missed frames, jitter, 100 ms lag, wind", 40,
             lambda: MockWorldConfig(target_colors=(RED, BLUE), latency_frames=2, wind=(0.4, 0.2, 0.0),
                                     target_path=circle_path(radius=6, speed=1.5)), tracker="noisy"),
    Scenario("branch_climb", "4 cm branch across the path at flight height (sonar)", 25,
             lambda: MockWorldConfig(target_colors=(RED, BLUE), target_path=straight_path(speed=1.5, heading_deg=0.0),
                                     range_sensors=False, sonar=True,
                                     branches=(((-10, 14, 2.8), (10, 14, 2.8), 0.04),)), sonar=True),
    Scenario("shirt_only", "colour tracker that only sees the red shirt (like Unity)", 30,
             lambda: MockWorldConfig(target_base=0.88, target_size=(0.45, 0.25, 0.65),
                                     target_path=circle_path(radius=6, speed=1.2)), tracker="color", shirt_only=True),
]


# --- suite 2: walls, buildings, doors (v2) -------------------------------------------
SUITE2 = [
    Scenario("stop_behind_pillar", "walks behind a 5 m wide pillar and stops in its shadow (10 s / 5 s), then walks on", 40,
             lambda v: MockWorldConfig(target_colors=(RED, BLUE), obstacles=((-1, 9, 2.5),),
                                       target_path=_route((-8, 12), (-0.5, 12, 1.5), ("wait", (10.0, 5.0)[v]),
                                                          (20, 12, 1.5))),
             start_yaw_to_target=True, suite=2, variants=("stop 10 s", "stop 5 s")),
    Scenario("turn_at_corner", "walks up the side of an 8x8 m building and turns round its far corner "
             "(keeps going / stops 8 s behind it)", 45,
             lambda v: MockWorldConfig(target_colors=(RED, BLUE), boxes=((0, 12, 8, 20, 4.0),),
                                       target_path=_route((-0.4, 6), (-0.4, 20.4, 1.5), (3.0 + 2.0 * v, 20.4, 1.5),
                                                          ("wait", (0.0, 8.0)[v]), (20, 20.4, 1.5), (20, 30, 1.5))),
             start_yaw_to_target=True, suite=2, variants=("keeps going", "stops behind it")),
    Scenario("u_turn_hidden", "goes behind a 12 m wall, turns back after 4 / 6.5 m, comes out where it went in", 45,
             lambda v: MockWorldConfig(target_colors=(RED, BLUE), boxes=(_wall(0, 13, 12, 13, 3.0),),
                                       target_path=_route((-0.4, 5), (-0.4, 13.45, 1.5), ((4.0, 6.5)[v], 13.45, 1.5),
                                                          (-0.4, 13.45, 1.5), (-0.4, 26, 1.5))),
             start_yaw_to_target=True, suite=2, variants=("turns at 4 m", "turns at 6.5 m")),
    Scenario("enters_building", "walks into a 10x6 m shop, stays 15 s, leaves by the same door / a second door", 50,
             lambda v: MockWorldConfig(target_colors=(RED, BLUE), boxes=((2, 14, 12, 20, 3.5),),
                                       target_path=(_route((-6, 12), (4, 12, 1.3), (4, 16, 1.0), ("wait", 15.0),
                                                           (4, 12, 1.0), (-6, 8, 1.3)) if v == 0 else
                                                    _route((-6, 12), (4, 12, 1.3), (4, 16, 1.0), ("wait", 13.0),
                                                           (13, 17, 1.0), (16, 12, 1.3), (16, 4, 1.3)))),
             start_yaw_to_target=True, suite=2, variants=("same door", "other door")),
    Scenario("decoy_at_exit_geo", "steps into a real shop for 8 / 12 s; a same-shirt decoy walks on where it was heading", 45,
             lambda v: MockWorldConfig(target_colors=(RED, BLUE), boxes=((0.2, 11.0, 4.2, 15.0, 3.0),),
                                       target_path=_route((-6, 10), (1.2, 10, 1.2), (1.2, 12.0, 1.0),
                                                          ("wait", (8.0, 12.0)[v]), (1.2, 10, 1.0), (14, 10, 1.2)),
                                       people=(MockPerson(lambda t: (-6 + 1.2 * t, 9.7) if t >= 7.0
                                                          else (-6 + 1.2 * 7.0, 30.0), RED, GREEN),)),
             start_yaw_to_target=True, suite=2, variants=("8 s inside", "12 s inside")),
    Scenario("low_wall_see_over", "walks round the end of a 2.2 m wall and along behind it, stops 10 s / keeps "
             "walking; the drone can see over by climbing", 40,
             lambda v: MockWorldConfig(target_colors=(RED, BLUE), boxes=(_wall(0, 13, 14, 13, 2.2),),
                                       target_path=_route((-0.4, 5), (-0.4, 13.5, 1.5), (4, 13.5, 1.5),
                                                          ("wait", (10.0, 0.0)[v]), (16, 13.5, 1.5), (16, 4, 1.5))),
             start_yaw_to_target=True, suite=2, variants=("stops 10 s", "keeps walking")),
    Scenario("into_hall", "turns into a 3 m wide hall between two 14 m walls (3.5 m high); walks through it / "
             "stops 8 s half way", 45,
             lambda v: MockWorldConfig(target_colors=(RED, BLUE),
                                       boxes=(_wall(2, 12, 16, 12, 3.5), _wall(2, 15, 16, 15, 3.5)),
                                       target_path=_route((-0.5, 4), (-0.5, 13.5, 1.5), (8.0, 13.5, 1.5),
                                                          ("wait", (0.0, 8.0)[v]), (18.0, 13.5, 1.5),
                                                          (18.0, 26.0, 1.5))),
             start_yaw_to_target=True, suite=2, variants=("walks through", "stops 8 s inside")),
    Scenario("round_wall_end", "goes round the end of a 12 m, 4 m high wall (two corners) and back along "
             "the far side, walking / running (Evan's Unity run, 2026-11-19)", 45,
             lambda v: MockWorldConfig(target_colors=(RED, BLUE), boxes=(_wall(0, 13, 12, 13, 4.0),),
                                       target_path=_route((-6, 12.4), (12.6, 12.4, (1.5, 3.0)[v]),
                                                          (12.6, 13.6, (1.5, 3.0)[v]), (0.0, 13.6, (1.5, 3.0)[v]),
                                                          (-6.0, 20.0, (1.5, 3.0)[v]))),
             start_yaw_to_target=True, suite=2, variants=("walking", "running")),
]
ALL_SCENARIOS = SCENARIOS + SUITE2


def build_encoder(reid: str):
    if reid == "none":
        return None
    if reid == "hist":
        from drone_core.adapters.appearance.color_hist import HistogramEncoder
        return HistogramEncoder()
    if reid == "onnx":
        from drone_core.adapters.appearance.onnx_reid import OnnxReidEncoder
        return OnnxReidEncoder(os.environ.get("REID_MODEL", "yolo26n-reid.onnx"))
    raise ValueError(reid)


def start_on_target(follow, tracker) -> None:
    """Like pressing F with the right person selected: start locked on the real
    target, so every version is scored from the same start."""
    tid = next((t for t, i in tracker.gt.items() if i == 0), None)
    if tid is not None and follow.selector.locked_id != tid:
        follow.selector.locked_id = tid
        if follow.appearance is not None:
            follow.appearance.reset()


def run_scenario(sc: Scenario, reid: str = "hist", variant: Optional[int] = None, legacy: bool = False) -> dict:
    world = MockWorld(sc.world() if variant is None else sc.world(variant))
    if sc.start_yaw_to_target:
        tp = world.target_pos()
        world.yaw = math.atan2(tp[0] - world.pos[0], tp[1] - world.pos[1])
    if sc.tracker == "people":
        tracker = MockPersonTracker(world, swap_prob=sc.swap_prob)
        gt = lambda tid: tracker.gt.get(tid)
    elif sc.tracker == "noisy":
        inner = MockPersonTracker(world)
        tracker = NoisyTracker(inner, jitter_frac=0.05, drop_prob=0.07, burst_len=3)
        gt = lambda tid: inner.gt.get(tid)
    else:
        tracker = ColorBlobTracker()
        gt = lambda tid: 0
    rig = rig_geometry(FOUR_CAMERA_RIG, 0.65, 0.88) if sc.shirt_only else rig_geometry(FOUR_CAMERA_RIG)
    follow = FollowTarget(TargetSelector(), FollowConfig(desired_box_height=0.30, rig=rig, use_belief=not legacy))
    enc = build_encoder(reid)
    if enc is not None:
        from drone_core.usecases.target_appearance import TargetAppearance
        follow.appearance = TargetAppearance(enc)
    vmap = None
    camera = MockCamera(world)
    if sc.sonar or sc.suite == 2:
        from drone_core.usecases.voxel_map import VoxelMap
        if sc.suite == 2:
            # front camera with depth (like Unity) so the 3-D map sees walls and buildings
            front = tuple(m for m in FOUR_CAMERA_RIG if m.name == "front")
            camera = MockMultiCamera(world, mounts=front, with_depth=True)
            vmap = VoxelMap(rig=rig_geometry(front))
        else:
            vmap = VoxelMap()
    m = FollowMission(camera, tracker, MockCommander(world), follow,
                      AvoidanceFilter(AvoidanceConfig(vertical_dodge=sc.sonar)), MockTelemetry(world),
                      MockClock(world), voxel_map=vmap)
    fps = world.cfg.fps
    warm = 3.0
    n_on = n_wrong = n_lost = n = 0
    dists, close, ticks = [], 0, []
    lost_at = relock = None
    states = {}
    hidden = 0
    ghost_err, top3 = [], []
    ghost_at = {}
    visible_again = None
    wrong_relocks, prev_who = 0, 0
    for k in range(int(sc.seconds * fps)):
        t0 = time.perf_counter()
        r = m.tick()
        ticks.append((time.perf_counter() - t0) * 1000)
        if k == 0 and sc.tracker != "color":
            start_on_target(follow, tracker if sc.tracker == "people" else inner)
        if world.t < warm:
            continue
        n += 1
        st = r.status
        states[st.state.split(" ")[0]] = states.get(st.state.split(" ")[0], 0) + 1
        tgt_id = st.target.track_id if st.target is not None else None
        if tgt_id is None and st.state.startswith("COASTING"):
            tgt_id = follow.selector.locked_id      # a missed detection: still locked on the same id
        who = gt(tgt_id) if tgt_id is not None else None
        locked = tgt_id is not None and st.state.startswith(("TRACKING", "COASTING", "TURNING"))
        if locked:
            if who == 0:
                n_on += 1
                if lost_at is not None and relock is None:
                    relock = world.t - lost_at
            else:
                n_wrong += 1
                if prev_who == 0 or prev_who is None:
                    wrong_relocks += 1
        else:
            n_lost += 1
        prev_who = who if locked else None
        if who != 0 and lost_at is None:
            lost_at = world.t
        seen = world.target_box(0.0) is not None
        hidden += not seen
        if lost_at is not None and relock is None and visible_again is None and seen and not (locked and who == 0):
            visible_again = world.t
        if not locked and lost_at is not None:
            est = follow.estimate(world.t, st.state)
            if est is not None:
                truth = world.target_pos()
                e = math.hypot(est["x"] - truth[0], est["y"] - truth[1])
                ghost_err.append(e)
                pts = est.get("modes") or [est]
                top3.append(min(math.hypot(q["x"] - truth[0], q["y"] - truth[1]) for q in pts) < 2.0)
                if relock is None:
                    for s_ in (5, 10):
                        if s_ not in ghost_at and world.t - lost_at >= s_:
                            ghost_at[s_] = e
        d = world.distance_to_target()
        dists.append(d)
        close += d < 2.0
    clear = min(world.min_clearance, world.min_branch_clearance)
    t_arr = np.array(ticks[int(fps):]) if len(ticks) > fps else np.array(ticks)
    refind = None
    if lost_at is not None:
        if visible_again is None:
            refind = round(relock, 1) if relock is not None else "never"
        else:
            refind = round(lost_at + relock - visible_again, 1) if relock is not None else "never"
    out = {
        "on_target_pct": round(100 * n_on / n, 1),
        "wrong_pct": round(100 * n_wrong / n, 1),
        "lost_pct": round(100 * n_lost / n, 1),
        "dist_mean": round(float(np.mean(dists)), 2),
        "dist_min": round(float(np.min(dists)), 2),
        "too_close_s": round(close / fps, 1),
        "relock_s": None if lost_at is None else (round(relock, 1) if relock is not None else "never"),
        "clearance_m": None if not math.isfinite(clear) else round(float(clear), 2),
        "tick_ms_mean": round(float(t_arr.mean()), 2),
        "tick_ms_p95": round(float(np.percentile(t_arr, 95)), 2),
        "swaps_caught": getattr(follow, "swaps_caught", 0),
        "states": {k: round(v / fps, 1) for k, v in sorted(states.items())},
        "hidden_s": round(hidden / fps, 1),
        "ghost_err_m": round(float(np.mean(ghost_err)), 2) if ghost_err else None,
        "ghost_err_5s": None if 5 not in ghost_at else round(ghost_at[5], 2),
        "ghost_err_10s": None if 10 not in ghost_at else round(ghost_at[10], 2),
        "top3_pct": round(100 * float(np.mean(top3)), 1) if top3 else None,
        "refind_s": refind,
        "wrong_relocks": wrong_relocks,
    }
    return out


def _run_by_name(name, reid, variant, legacy):
    """For --jobs: worker processes look the scenario up by name (lambdas don't pickle)."""
    return run_scenario(next(s for s in ALL_SCENARIOS if s.name == name), reid, variant, legacy)


def _one(r: dict) -> float:
    v = r["on_target_pct"] - 2.0 * r["wrong_pct"] - 5.0 * r["too_close_s"]
    if r["clearance_m"] is not None and r["clearance_m"] < 0.3:
        v -= 50
    return max(0.0, min(100.0, v))


def score(res: dict, suite: int = 1) -> Optional[float]:
    """One number per suite, 0-100: mostly time on the right person, with
    penalties for following the wrong one and for getting too close (and -50
    for touching an obstacle). Suite 1 = the original 11 scenarios."""
    names = {s.name for s in ALL_SCENARIOS if s.suite == suite}
    s = [_one(r) for k, r in res.items() if k.split(".")[0] in names]
    return round(float(np.mean(s)), 1) if s else None


def _cell(v):
    return "" if v is None else v


def write_report() -> str:
    runs = []
    for f in sorted(os.listdir(BENCH_DIR)):
        if f.endswith(".json"):
            with open(os.path.join(BENCH_DIR, f)) as fh:
                runs.append(json.load(fh))
    runs.sort(key=lambda r: r["created"])
    L = ["# Benchmark report", "",
         "Mock-sim scenarios, same seeds every run (`python tools/benchmark.py --version vN --suite all`).",
         "on = % of time locked on the real target, wrong = % locked on someone else, "
         "lost = % with no target, dist = mean / min metres, close = s under 2 m, "
         "relock = s from first loss back to the real target.",
         "Suite 2 (occlusion suite, v2+): ghost = mean metres from the ghost to the real person while lost, "
         "top3 = % of lost time the person is within 2 m of one of the 3 heaviest guesses, "
         "refind = s from the person being visible again to back on them.", ""]
    L.append("| version | suite 1 score (11 original) | suite 2 score (occlusion) | notes | created |")
    L.append("|---|---|---|---|---|")
    for r in runs:
        s1 = r.get("score_suite1", r.get("score"))
        L.append(f"| **{r['version']}** | {_cell(s1)} | {_cell(r.get('score_suite2'))} | {r['notes']} | {r['created'][:16]} |")
    L.append("")
    L += ["## Suite 1", ""]
    for sc in SCENARIOS:
        L += [f"### {sc.name}", f"_{sc.about}_", "",
              "| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |",
              "|---|---|---|---|---|---|---|---|---|"]
        for r in runs:
            x = r["results"].get(sc.name)
            if x is None:
                continue
            L.append(f"| {r['version']} | {x['on_target_pct']} | {x['wrong_pct']} | {x['lost_pct']} | "
                     f"{x['dist_mean']} / {x['dist_min']} | {x['too_close_s']} | {_cell(x['relock_s'])} | "
                     f"{_cell(x['clearance_m'])} | {x['tick_ms_mean']} / {x['tick_ms_p95']} |")
        L.append("")
    L += ["## Suite 2 (occlusion)", ""]
    for sc in SUITE2:
        L += [f"### {sc.name}", f"_{sc.about}_", "",
              "| version | variant | on % | wrong % | close s | relock s | refind s | ghost m | @5s / @10s | top3 % | clearance m | tick ms mean/p95 |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in runs:
            for i, lab in enumerate(sc.variants):
                x = r["results"].get(f"{sc.name}.{i}")
                if x is None:
                    continue
                L.append(f"| {r['version']} | {lab} | {x['on_target_pct']} | {x['wrong_pct']} | {x['too_close_s']} | "
                         f"{_cell(x['relock_s'])} | {_cell(x.get('refind_s'))} | {_cell(x.get('ghost_err_m'))} | "
                         f"{_cell(x.get('ghost_err_5s'))} / {_cell(x.get('ghost_err_10s'))} | {_cell(x.get('top3_pct'))} | "
                         f"{_cell(x['clearance_m'])} | {x['tick_ms_mean']} / {x['tick_ms_p95']} |")
        L.append("")
    text = "\n".join(L)
    with open(os.path.join(BENCH_DIR, "REPORT.md"), "w") as fh:
        fh.write(text)
    return text


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--version", required=True, help="label, e.g. v2")
    p.add_argument("--notes", default="")
    p.add_argument("--reid", choices=["none", "hist", "onnx"], default="hist",
                   help="appearance memory: none, colour histogram, or the yolo26n-reid network")
    p.add_argument("--suite", choices=["1", "2", "all"], default="all")
    p.add_argument("--legacy", action="store_true", help="v1 behaviour: no particle belief / viewpoint search")
    p.add_argument("--only", default=None, help="comma-separated scenario names (suite 2: name or name.variant)")
    p.add_argument("--jobs", type=int, default=1,
                   help="run scenarios in parallel (faster; tick ms then is not comparable)")
    p.add_argument("--no-save", action="store_true")
    a = p.parse_args()
    os.makedirs(BENCH_DIR, exist_ok=True)
    only = None if a.only is None else set(a.only.split(","))
    todo = []
    for sc in ALL_SCENARIOS:
        if a.suite != "all" and str(sc.suite) != a.suite:
            continue
        for v in (range(len(sc.variants)) if sc.variants else (None,)):
            key = sc.name if v is None else f"{sc.name}.{v}"
            if only is None or sc.name in only or key in only:
                todo.append((key, sc, v))
    results = {}
    pool = None
    if a.jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        pool = ProcessPoolExecutor(a.jobs)
        futs = [pool.submit(_run_by_name, sc.name, a.reid, v, a.legacy) for _, sc, v in todo]
    for i, (key, sc, v) in enumerate(todo):
        t0 = time.time()
        results[key] = r = futs[i].result() if pool else run_scenario(sc, a.reid, v, a.legacy)
        extra = ""
        if sc.suite == 2:
            extra = f"  refind {r['refind_s']}  ghost {r['ghost_err_m']}  top3 {r['top3_pct']}"
        print(f"{key:22s} on {r['on_target_pct']:5.1f}%  wrong {r['wrong_pct']:5.1f}%  "
              f"lost {r['lost_pct']:5.1f}%  close {r['too_close_s']}  relock {r['relock_s']}{extra}  "
              f"tick {r['tick_ms_mean']:.1f} ms  ({time.time() - t0:.0f} s)", flush=True)
    rec = {"version": a.version, "notes": a.notes or f"reid={a.reid}", "reid": a.reid, "legacy": a.legacy,
           "created": datetime.now().isoformat(timespec="seconds"),
           "machine": f"{platform.system()} {platform.machine()} py{platform.python_version()}",
           "score_suite1": score(results, 1), "score_suite2": score(results, 2), "results": results}
    rec["score"] = rec["score_suite1"]
    print(f"\nsuite 1 score {rec['score_suite1']} / 100   suite 2 score {rec['score_suite2']} / 100")
    if not a.no_save:
        with open(os.path.join(BENCH_DIR, f"{a.version}.json"), "w") as fh:
            json.dump(rec, fh, indent=1)
        write_report()
        print(f"saved benchmarks/{a.version}.json and benchmarks/REPORT.md")


if __name__ == "__main__":
    main()
