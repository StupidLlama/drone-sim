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

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from drone_core.adapters.mock.sim import (ColorBlobTracker, MockCamera, MockClock, MockCommander,  # noqa: E402
                                          MockPerson, MockPersonTracker, MockTelemetry, MockWorld,
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


@dataclass
class Scenario:
    name: str
    about: str
    seconds: float
    world: Callable[[], MockWorldConfig]
    tracker: str = "people"            # people | color | noisy
    swap_prob: float = 0.0
    sonar: bool = False
    shirt_only: bool = False
    start_yaw_to_target: bool = False


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


def run_scenario(sc: Scenario, reid: str = "hist") -> dict:
    world = MockWorld(sc.world())
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
    follow = FollowTarget(TargetSelector(), FollowConfig(desired_box_height=0.30, rig=rig))
    enc = build_encoder(reid)
    if enc is not None:
        from drone_core.usecases.target_appearance import TargetAppearance
        follow.appearance = TargetAppearance(enc)
    vmap = None
    if sc.sonar:
        from drone_core.usecases.voxel_map import VoxelMap
        vmap = VoxelMap()
    m = FollowMission(MockCamera(world), tracker, MockCommander(world), follow,
                      AvoidanceFilter(AvoidanceConfig(vertical_dodge=sc.sonar)), MockTelemetry(world),
                      MockClock(world), voxel_map=vmap)
    fps = world.cfg.fps
    warm = 3.0
    n_on = n_wrong = n_lost = n = 0
    dists, close, ticks = [], 0, []
    lost_at = relock = None
    states = {}
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
        if tgt_id is not None and st.state.startswith(("TRACKING", "COASTING", "TURNING")):
            if who == 0:
                n_on += 1
                if lost_at is not None and relock is None:
                    relock = world.t - lost_at
            else:
                n_wrong += 1
        else:
            n_lost += 1
        if who != 0 and lost_at is None:
            lost_at = world.t
        d = world.distance_to_target()
        dists.append(d)
        close += d < 2.0
    clear = min(world.min_clearance, world.min_branch_clearance)
    t_arr = np.array(ticks[int(fps):]) if len(ticks) > fps else np.array(ticks)
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
    }
    return out


def score(res: dict) -> float:
    """One number per run, 0-100: mostly time on the right person, with
    penalties for following the wrong one and for getting too close."""
    s = []
    for r in res.values():
        v = r["on_target_pct"] - 2.0 * r["wrong_pct"] - 5.0 * r["too_close_s"]
        if r["clearance_m"] is not None and r["clearance_m"] < 0.3:
            v -= 50
        s.append(max(0.0, min(100.0, v)))
    return round(float(np.mean(s)), 1)


def write_report() -> str:
    runs = []
    for f in sorted(os.listdir(BENCH_DIR)):
        if f.endswith(".json"):
            with open(os.path.join(BENCH_DIR, f)) as fh:
                runs.append(json.load(fh))
    runs.sort(key=lambda r: r["created"])
    names = [s.name for s in SCENARIOS]
    L = ["# Benchmark report", "",
         "Mock-sim scenarios, same seeds every run (`python tools/benchmark.py --version vN`).",
         "on = % of time locked on the real target, wrong = % locked on someone else, "
         "lost = % with no target, dist = mean / min metres, close = s under 2 m, "
         "relock = s from first loss back to the real target.", ""]
    L.append("| version | score | notes | created |")
    L.append("|---|---|---|---|")
    for r in runs:
        L.append(f"| **{r['version']}** | {r['score']} | {r['notes']} | {r['created'][:16]} |")
    L.append("")
    for name in names:
        about = next(s.about for s in SCENARIOS if s.name == name)
        L += [f"### {name}", f"_{about}_", "",
              "| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |",
              "|---|---|---|---|---|---|---|---|---|"]
        for r in runs:
            x = r["results"].get(name)
            if x is None:
                continue
            rl = "" if x["relock_s"] is None else x["relock_s"]
            cl = "" if x["clearance_m"] is None else x["clearance_m"]
            L.append(f"| {r['version']} | {x['on_target_pct']} | {x['wrong_pct']} | {x['lost_pct']} | "
                     f"{x['dist_mean']} / {x['dist_min']} | {x['too_close_s']} | {rl} | {cl} | "
                     f"{x['tick_ms_mean']} / {x['tick_ms_p95']} |")
        L.append("")
    text = "\n".join(L)
    with open(os.path.join(BENCH_DIR, "REPORT.md"), "w") as fh:
        fh.write(text)
    return text


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--version", required=True, help="label, e.g. v1")
    p.add_argument("--notes", default="")
    p.add_argument("--reid", choices=["none", "hist", "onnx"], default="hist",
                   help="appearance memory: none, colour histogram, or the yolo26n-reid network")
    p.add_argument("--only", default=None, help="comma-separated scenario names")
    p.add_argument("--no-save", action="store_true")
    a = p.parse_args()
    os.makedirs(BENCH_DIR, exist_ok=True)
    todo = [s for s in SCENARIOS if a.only is None or s.name in a.only.split(",")]
    results = {}
    for sc in todo:
        t0 = time.time()
        results[sc.name] = run_scenario(sc, a.reid)
        r = results[sc.name]
        print(f"{sc.name:18s} on {r['on_target_pct']:5.1f}%  wrong {r['wrong_pct']:5.1f}%  "
              f"lost {r['lost_pct']:5.1f}%  dist {r['dist_mean']:.1f}/{r['dist_min']:.1f}  "
              f"relock {r['relock_s']}  tick {r['tick_ms_mean']:.1f} ms  ({time.time() - t0:.0f} s)", flush=True)
    rec = {"version": a.version, "notes": a.notes or f"reid={a.reid}", "reid": a.reid,
           "created": datetime.now().isoformat(timespec="seconds"),
           "machine": f"{platform.system()} {platform.machine()} py{platform.python_version()}",
           "score": score(results), "results": results}
    print(f"\nscore {rec['score']} / 100")
    if not a.no_save:
        with open(os.path.join(BENCH_DIR, f"{a.version}.json"), "w") as fh:
            json.dump(rec, fh, indent=1)
        write_report()
        print(f"saved benchmarks/{a.version}.json and benchmarks/REPORT.md")


if __name__ == "__main__":
    main()
