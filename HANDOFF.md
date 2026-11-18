# Drone follow sim: handoff (2026-11-18)

Paste this file (or point the new thread at it) to pick up where the last thread stopped.

## Where everything is
- GitHub repo: https://github.com/StupidLlama/drone-sim (master copy since 2026-11-18; v1 on `main`,
  v2 on branch `v2-lost-target-estimation` / its PR). Work on a branch, open a PR, then also copy to the
  cloud project files and Evan's PC folders until Evan switches to `git pull`.
- Master copy (cloud project files): `/mnt/project-files/drone-sim/` and `/mnt/project-files/drone-sim.zip`
- Evan's PC:
  - Python: `C:\Users\user\Downloads\drone-sim\drone-python\` (core in `core\`)
  - Unity project: `C:\Users\user\Desktop\drone sim\` (scripts in `Assets\Scripts\`, Unity 6000.6.9f1)
  - Research reports: `C:\Users\user\Desktop\drone sim\Research\` (copies in `/mnt/project-files/research/`)
- All three copies were md5-checked identical on 2026-11-17 after the last change.

## How to run
- Unity: open the project, press Play (Unity is the TCP server on 127.0.0.1:5555).
- Python: double-click `run_unity.bat`, or the VS Code run button (rename `vscode` to `.vscode` first).
- Manual: `cd core` then `python main.py --mode unity --tracker color` (or `--mode mock`).
- Keys in Unity: V ghost target, Z scan route, X 3D map, B battery, Q/E or right-drag chase camera.
- Keys in Python HUD: `[` `]` rotate the 3D map panel.

## Current state: v4 (suite 1 92.7, occlusion suite 2: v1 64.2, v2 84.1, v3 83.6, v4 84.6)
v4 (2026-11-18): `usecases/turn_anticipation.py` while following with the map: wall ahead of the target
(likely turn = more open side) or a wall beside them that ends (corner) -> follow at 65% distance (>= 3 m,
not when they walk toward the drone) and move 2.5 m to the outside of the turn; held 1.5 s
(`FollowConfig.anticipate`). Room route also re-locks on anyone within 4 m of the building (`room_gate_m`).
turn_at_corner now never lost (93.9 -> 100%), u_turn_hidden 92.0 -> 93.2. 60 tests + 1 skip.

v3 (2026-11-18, Evan's idea A): `usecases/obstacle_type.py` classifies what the target vanished behind
(wall / hall / room, certainty, commit at 0.8, re-classified every 0.5 s), `usecases/scan_routes.py`
builds that scene's route (wall: swing wide past the end then along the hidden side; low wall: one look
over first; hall: down the centre line; room: door -> both sides -> door, looped). The v2 viewpoint
search may overrule wall/hall routes when it sees 2x more of the belief (`route_bias`); never a room's.
People are cut out of the depth before mapping (`FollowMission._cut_people`). Unity draws the route
(`RouteView.cs`, Z; `est.route` in the protocol). Python HUD 3-D map off unless `--hud-map`.
New benchmark scenario `into_hall` (v1 and v2 re-run on it and merged into their json).
59 tests + 1 skip (~10 min). Unity C# for v2 and v3 not compiled here.

v2: `usecases/target_belief.py` particle filter (walk/turn/stop/inside, map walls,
negative information), `usecases/viewpoints.py` viewpoint search (with the map), belief re-lock gate,
`VoxelMap.height_at / top_between / free_above / occupied_above / lines_blocked / obstacle_mask`,
mock `boxes` (walls/buildings), benchmark suite 2, Unity faded extra ghosts (`est.modes`).
Without a map (`--no-map`, real mode) the flight is still v1; the belief only drives ghost + gate.

### v1 notes
- Fly-over fix: ground range from altitude + box edge angle, min separation brake/backoff (`--distance`, `--target-height`).
- Appearance re-ID memory (`--reid auto|off|color|net`), BoT-SORT re-ID tracker config, id-swap check.
- Ghost target in Unity showing where the core thinks the lost target is.
- 3D voxel map shown in the Python HUD and in Unity (MSG_MAP = 6, 2 Hz).
- Loss handling: REACQUIRING 8 s -> SEARCHING orbit -> LOITER 60 s -> RTH.
- Tests: `cd core && python -m pytest -q tests` -> 49 passed, 1 skipped (reid net model missing), about 8.5 min.
- Not verified inside Evan's Unity yet: the Unity map view (X). C# compiles against Unity refs.

## Rules for the next thread
1. After every update run `python tools/benchmark.py --version vN --notes "..."` from `core/` and show a before/after table. Next version is **v5**.
2. Sync: after any change, md5-compare every file against both PC folders and push all differences, not just the files you touched (stale files broke Evan's runs twice).
3. Keep the Python protocol (`adapters/unity/protocol.py`) and C# (`PythonBridge.cs`) in sync.
4. New features go in `usecases/` and depend only on `ports/`; `main.py` is the only place things are wired together.

## Next job: v5
- Orbit/spiral mapping while the target is idle, with one camera kept on them (Evan's idea);
  a gimbal would let the drone look straight down over low walls.
- Room route timing is fragile (enters_building.1 swings 15-45% on small changes): use exits found on the map.
- Open items: give-up rule from visible-reachable mass instead of timers, side/back cameras in suite 2
  (front-camera only), suite-1 pillar_occlusion ghost without a map.
- Benchmark: `python tools/benchmark.py --version v5 --suite all --jobs 4 --notes "..."`, compare with v4.
