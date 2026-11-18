# Drone follow sim: handoff (2026-11-18)

Paste this file (or point the new thread at it) to pick up where the last thread stopped.

## Where everything is
- GitHub repo: https://github.com/StupidLlama/drone-sim.git (created by Evan 2026-11-18, may still be empty).
  First job in a new thread: attach this repo, push the full `/mnt/project-files/drone-sim/` contents
  (no `__pycache__`, `.pytest_cache`, `*.onnx`; add a `.gitignore` for those plus Unity `Library/`, `Temp/`, `Logs/`),
  then from then on make the repo the master copy and have Evan `git pull` instead of file copying.
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

## Current state: v1 (benchmark score 92.7, baseline v0 79.6)
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

## Next job: v2 = better lost-target estimation
Read first: `research/Drone v2 lost target estimation.md` (ranked 11-item to-do list, new scenarios, sources).
Problem: the ghost keeps walking at full speed and overshoots (about 9.6 m off in `decoy_at_exit`, re-lock gate is 8 m).
Plan in short:
1. Add real walls/corners to the mock world (needed to test anything below).
2. Replace the constant-velocity guess with a particle filter: walk / turn / stop / hidden modes, map-constrained.
3. Negative information: remove particles in areas the cameras saw empty.
4. Fly to the viewpoint that sees the most remaining probability, not to one point.
5. Obstacle-height query (Evan approved for v2): `VoxelMap.height_at(x, y)` / `top_between(a, b)` returning the
   highest occupied voxel, so the viewpoint search knows if it can see over a wall and how high to climb
   (respect the 10 m ceiling and the <1 m overhead rule). Unit test with a known wall in the mock world.
6. New benchmark scenarios (corner turn, stops behind wall, doubles back, low wall to see over).

Keep v2 to estimation only, so the benchmark shows what helped.
Later: v3 world-frame follow controller with velocity feed-forward (reuse the v2 world estimate);
v3+ orbit/spiral mapping while following (needs v2 recovery first); wall-penetrating radar (hardware, later).
