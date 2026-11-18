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
- Keys in Unity: V ghost target, X 3D map, B battery, Q/E or right-drag chase camera.
- Keys in Python HUD: `[` `]` rotate the 3D map panel.

## Current state: v2 (suite 1 score 92.7, occlusion suite 2: v1 59.2 -> v2 82.2)
v2 (2026-11-18): `usecases/target_belief.py` particle filter (walk/turn/stop/inside, map walls,
negative information), `usecases/viewpoints.py` viewpoint search (with the map), belief re-lock gate,
`VoxelMap.height_at / top_between / free_above / occupied_above / lines_blocked / obstacle_mask`,
mock `boxes` (walls/buildings), benchmark suite 2 (6 scenarios x 2 variants, `--suite`, `--legacy`, `--jobs`),
Unity faded extra ghosts (`est.modes`). 55 tests + 1 skip (~10 min). Unity C# for v2 not compiled.
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
1. After every update run `python tools/benchmark.py --version vN --notes "..."` from `core/` and show a before/after table. Next version is **v2**.
2. Sync: after any change, md5-compare every file against both PC folders and push all differences, not just the files you touched (stale files broke Evan's runs twice).
3. Keep the Python protocol (`adapters/unity/protocol.py`) and C# (`PythonBridge.cs`) in sync.
4. New features go in `usecases/` and depend only on `ports/`; `main.py` is the only place things are wired together.

## Next job: v3
- World-frame follow controller with velocity feed-forward (reuse the v2 belief/world estimate).
- Then orbit/spiral mapping while following; wall-penetrating radar later (hardware).
- Open v2 items (from the research to-do): goal hints (doors/exits), give-up rule from visible-reachable
  mass instead of timers, simulated UWB/GPS tag, speed-ups if tick p95 grows (enters_building p95 71 ms),
  side/back cameras in suite 2 (it is front-camera only), ghost in suite-1 pillar_occlusion got worse
  without a map (0.79 -> 4.19 m: the "stopped" guess becomes the heaviest).
- Benchmark: `python tools/benchmark.py --version v3 --suite all --notes "..."`, compare with v2.
