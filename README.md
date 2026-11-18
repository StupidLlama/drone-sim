# Drone follow sim: Unity world + clean-architecture Python core

Yes, Unity is the right call for this. You drive the target with the keyboard
(or let it walk scripted/random paths), the drone's four cameras (front, right, back, left) stream JPEG frames and depth
to Python over a local TCP socket, and Python sends velocity commands back.
The Python core doesn't know it's talking to Unity: switching to the real
drone means swapping adapters in `main.py`, nothing else.

```
 Unity (C#)                                      Python core
┌──────────────────────────┐   TCP 127.0.0.1:5555  ┌──────────────────────────────────────┐
│ CameraStreamer ─ JPEG + telemetry JSON ─────────►│ UnityCamera / UnityTelemetry  (adapters)
│ DroneController ◄─ {forward,right,up,yaw_rate} ──│ UnityCommander                (adapter)
│ TargetController (WASD / waypoints / wander)     │            │ ports (interfaces)
│ WindField (gusts → drag on the drone)            │            ▼
│ SceneBootstrap builds the whole world on Play    │ FollowMission: tracker → Kalman filter → FollowTarget (PIDs)
└──────────────────────────┘                       │   → FlightSupervisor → AvoidanceFilter → commander
                                                   └──────────────────────────────────────┘
```

## Layout

```
core/
  drone_core/
    domain/models.py          Frame, BoundingBox, Track, Telemetry, VelocityCommand (pure data)
    ports/__init__.py         CameraSource, TelemetrySource, DroneCommander, ObjectTracker, Clock
    usecases/
      pid.py                  PID with anti-windup + filtered D
      target_selection.py     lock onto one track id, re-lock after ByteTrack id switches
      target_filter.py        Kalman filter on the box: smooths jitter, coasts through missed
                              frames, predicts across camera latency
      surround_map.py         top-down obstacle map built from the 4 cameras' depth
      target_geo.py           ground position + velocity of the target from box + drone pose
      target_belief.py        v2: particle cloud of where a hidden target is (walk / turn / stop /
                              inside), kept out of walls, emptied where the cameras see nobody
      viewpoints.py           v2: pick the spot that sees the most of that cloud (climbs over low walls)
      voxel_map.py            3-D map from sonar + depth; line of sight, obstacle-height queries
      follow_target.py        visual servo: yaw←x offset, climb←y offset, forward←log(box height)
                              + COASTING → REACQUIRING → SEARCHING → IDLE when the target is lost
      flight_supervisor.py    FOLLOW / HOLD / RTH modes; low-battery and geofence failsafes
      avoidance.py            safety filter: brakes, then sidesteps around what's ahead
      follow_mission.py       the loop: camera → tracker → follow → avoidance → commander
    adapters/
      unity/                  TCP protocol + bridge (camera, telemetry, commander)
      yolo/                   Ultralytics YOLO + ByteTrack as an ObjectTracker
      mock/sim.py             pure-Python world + colour tracker (tests, no Unity/YOLO)
      hardware/real.py        OpenCV cameras (1 or 4) + MAVSDK offboard (real drone, untested)
      depth/depth_anything.py camera image → metric depth for the map on the real drone (untested)
      hud.py                  OpenCV debug window + mode keys
      recorder.py             CSV flight log
  main.py                     composition root: picks adapters from --mode/--tracker
  tools/fake_unity.py         speaks Unity's protocol using the mock world
  tools/plot_log.py           plots a flight log for PID tuning
  tests/test_core.py
unity/Assets/Scripts/         the C# side (10 scripts)
```

The dependency rule: `domain` ← `ports` ← `usecases` ← `adapters` ← `main.py`.
Nothing in `domain/`, `ports/` or `usecases/` imports OpenCV sockets, Unity,
Ultralytics or MAVSDK.

## 1. Python setup

```bash
cd core
python -m venv .venv && source .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python -m pytest -q                                   # 28 tests, ~6 min
```

Quick check with no Unity at all:

```bash
python main.py --mode mock --tracker color            # OpenCV window, q to quit
```

## 2. Unity setup (once, ~5 minutes)

1. Unity Hub → New project → **Universal 3D** (URP) or **3D (Built-in)**. Unity 6 or 2022.3 LTS.
2. Copy `unity/Assets/Scripts/` into your project's `Assets/` folder.
3. New empty scene (File → New Scene → Basic or Empty). Create an empty GameObject
   (GameObject → Create Empty), name it `Bootstrap`, add the **SceneBootstrap** component.
   If the scene already has a Main Camera and Directional Light that's fine, it reuses them.
4. **Edit → Project Settings → Player → Resolution and Presentation → Run In Background = ON.**
   Without this Unity pauses when you click on the Python window and the stream stops.
5. Press **Play**. You should see the world, the red "blocky person", the drone, a
   picture-in-picture of the drone camera (top-right) and a HUD saying
   "Python: waiting on port 5555".

Input works with either input system (the scripts detect it). If you see an
`InvalidOperationException` about `UnityEngine.Input`, set Project Settings →
Player → Active Input Handling = **Both**.

## 3. Run

Start Unity first (Play), then:

```bash
# colour tracker: works with the red placeholder person, no GPU needed
python main.py --mode unity --tracker color

# YOLO + ByteTrack (downloads yolo11n.pt on first run)
python main.py --mode unity --tracker yolo --device 0      # or cpu / mps
```

### Controls (Unity Game window must have focus)

| Key | Action |
|---|---|
| WASD, Shift | move target (relative to the view), run |
| T | target mode: Manual → Waypoints (figure-8) → Wander (random, with pauses) |
| G | wind + gusts on/off |
| M | manual drone override: I/K forward/back, J/L strafe, U/O yaw, Y/H up/down |
| Q / E, or hold right mouse + drag | orbit the view camera around the target |
| C | camera: behind target → behind drone → overhead |
| B | drop battery to 15% (triggers Python's return-home failsafe); again to refill |
| P | show/hide the 4 drone camera previews |
| N | show/hide the sonar beams (yellow = echo, length = range; blue = clear). The drone cameras never see them |
| V | show/hide the ghost target: where Python thinks the person is (see below) |
| X | show/hide the 3-D obstacle map Python built (blocks coloured by height) |
| R | reset drone and target |
| F1 | hide help |

In the Python window (click it first): `f` follow, `h` return home, `space` hold,
`q` quit (drone hovers).

### The 3-D map in Unity

Twice a second Python sends its 3-D obstacle map (sonar + depth, 20 cm blocks) to Unity,
which draws every occupied block where it is in the world, coloured by height: blue at
the ground, through green and yellow, to red at 6 m and up. That lets you check the map
against the real scene: blocks should sit on trunks, branches and walls, and nowhere
else. X toggles it. Like the ghost and the sonar lines, the drone's own cameras don't
see it. (The same map is also in the Python window's 3D panel.)

### Ghost target: what the drone thinks

A see-through person in Unity marks where the Python core *thinks* the target is,
with a ring on the ground and a label ("drone thinks: TRACKING"):

* **green**: it sees the person. The ghost should sit on them; the gap is its distance error.
* **orange**: the person is hidden. Since v2 the ghost is the core's **best guess** out of
  a cloud of possibilities (stopped behind the wall, walked on, turned the corner, went
  into the shop). One or two **fainter ghosts** show the next-best guesses, more see-through
  the less likely they are. The ring shows how spread out the best guess is.
* **cyan**: TURNING toward a side/back camera. **Grey**: gave up.

The ghost is on the debug layer, so the drone's own cameras never see it. V toggles it.

### Picking the right person again (appearance memory)

Re-locking by position alone fails when someone else is standing where the target
should be. So the core also remembers what the target **looks like** (`usecases/target_appearance.py`):

* While locked on and the person is clearly in view, it saves an appearance
  "fingerprint" every 0.3 s. It only saves views that still match, so it can't slowly
  learn the wrong person.
* Everyone else in view at the same time is saved as "not the target".
* To re-lock, a candidate must look like the target **and** more like the target than
  like anyone in the "not" list, plus be near the predicted spot (a very clear match may
  be up to 3 m further out).
* If the tracker hands the target's id to someone else (two people crossing), the
  fingerprint stops matching. After 4 failed checks (~1 s) the drone drops that id and
  re-locks on whoever actually looks like the target.

Fingerprints: `--reid color` = colour histograms of shirt and trousers (no download; fine
for the sim's blocky people). `--reid net` = the **yolo26n-reid** network (2.8M
parameters, trained on 4,000+ real people; downloaded on first use, needs
`pip install onnxruntime`), much better on real people. The default `--reid auto` uses
the network with `--tracker yolo` and colours with `--tracker color`. `--reid off` disables it.
With `--tracker yolo` the tracker itself (BoT-SORT, `trackers/botsort_reid.yaml`) also
matches by appearance, which keeps ids through short occlusions.

### Performance benchmark

```powershell
python tools\benchmark.py --version v3 --notes "what changed"
python tools\benchmark.py --version test --suite 2 --jobs 4 --no-save   # quick look, ~2 min
```

**Suite 1** flies the 11 original scenarios in the mock sim (circle, sprint, walking at the
drone, pillar occlusion, a look-alike at the exit, id swaps, a crowd, a bad detector, a
branch, the Unity colour tracker...). **Suite 2** (since v2, the occlusion suite) has real
walls, buildings and a shop, a front depth camera feeding the 3-D map, and 6 scenarios in
2 variants each: stops behind a pillar, turns round a building corner, U-turn behind a
wall, goes into a shop and out by the same or another door, a look-alike at the shop door,
a low wall that can be seen over. It scores time on the right person, time on the wrong
person, distance, re-lock time, clearance and CPU time per tick, and for suite 2 also
how far the ghost was from the real person (`ghost m`), how often the person was within
2 m of one of the top 3 guesses (`top3 %`) and how fast it re-found them once visible
(`refind s`). It saves `benchmarks/vN.json` and rewrites `benchmarks/REPORT.md` with every
version side by side. Same seeds every time, so changes in the numbers come from the code.
About 12 minutes for both suites; `--jobs 4` runs scenarios in parallel (then the tick
times aren't comparable). `--legacy` runs the v1 loss handling for comparison.

### When the target disappears

1. **COASTING** (up to 0.6 s): the Kalman filter keeps predicting the box, so a few
   missed detections don't change anything.
2. **Belief (v2).** From then on the core keeps a cloud of 2,000 guesses
   (`usecases/target_belief.py`). Each guess walks on, turns (a corner, a U-turn), stops,
   or goes **inside** something (a guess that walks into a wall on the map either slides
   along it or went through a door), with walking speeds, stop and turn rates from
   pedestrian studies. Guesses can't walk through mapped walls. Every frame, guesses the
   cameras could clearly see (in view, in range, nothing in between on the 3-D map) lose
   weight, because nobody was there: looking at an empty street pushes the cloud behind
   the wall. Places the map hasn't seen don't count as "seen empty". Someone else standing
   there (a decoy) doesn't count either: the target could be right behind them.
3. **REACQUIRING** (up to 8 s) and **SEARCHING** (20 s): with the 3-D map, the drone flies
   to the **viewpoint that would see the most of the cloud soonest** (`usecases/viewpoints.py`,
   re-planned 3 times a second, only switching for a 20% better spot). Candidates sit on
   rings round the top guesses. If a mapped wall is in the way, the candidate is raised
   just enough to see over it (**obstacle-height query**, `VoxelMap.height_at` /
   `top_between`), never above 10 m, never into something overhead (needs 1 m clear), and
   only when the map has seen open air above that wall (a tall pillar it hasn't seen the
   top of isn't "low"). Guesses that went inside count at their door, so the drone ends
   up watching the door. Without a map (`--no-map`, the real drone today) it keeps the
   v1 flight: toward the predicted spot, then circles it 4 m out and 2 m wider every lap.
4. **LOITER** (60 s): flies back to where the person was last seen and hovers there.
5. **GAVE_UP**: forgets the target and the supervisor flies home (**RTH (target lost)**).
   Press `f` to follow again.

Obstacle avoidance and the battery / geofence failsafes stay on the whole time. Tune
`orbit_s`, `orbit_radius`, `orbit_growth`, `loiter_s` in `FollowConfig`.

**Re-locking.** When someone reappears after an occlusion, ByteTrack usually gives them a
new id. Since v2 the core re-locks only on a person where the cloud says the target could
be (enough guesses within 1.5 m, or near one of the top 3 guesses), so a person who
**stopped** where they vanished is accepted even when most guesses walked on. It also waits
until it has seen them there for 0.5 s (**CONFIRMING** when
it's in a side camera), so a decoy elsewhere, or a look-alike that flashes past, isn't
picked up. A look-alike standing right where the target should be can still win: the core
checks position, not appearance. This needs the camera numbers in `FollowConfig.camera`
(`CameraGeometry`) to match the camera: 90° horizontal FOV, 20° pitch down, 1.8 m tall
person by default, the same as SceneBootstrap.

**Climbing for a better view.** While following, the core checks the 3-D map for an
obstacle on the line from the drone to the person's head, now and 1 s ahead. If there is
one, it climbs at 1 m/s until the line is clear and holds that height for 3 s. It never
climbs above 10 m (`los_max_altitude`), or so high that the person would drop out of the
bottom of the front camera. The sonar's overhead check still stops it climbing into
branches. The debug window shows `view: blocked: climbing` / `holding height`.

### Keeping its distance (not flying over the person)

With telemetry, the follow distance is measured on the ground, not from box size alone:
from the drone's altitude and how far down the camera looks at the bottom (or top) of
the box. Box size alone fails from above (a person seen from above looks shorter, which
reads as "too far", so the drone used to creep in until the person went under it). On
top of that:

* if the box is cut off by the top or bottom of the image, the drone never moves closer;
* closer than 60% of the follow distance, it brakes and backs off at 1.5 m/s and won't
  climb;
* if it loses the person right below it, it backs off to get them in view again;
* `--tracker color` in Unity knows it only sees the red shirt and arms (0.88 to 1.53 m
  above the ground), not a 1.8 m person. Before, that made it think the person was 3x
  further away than they were.

The climb for a better view (when an obstacle blocks the line of sight) still works
alongside this, and its height limit keeps the person inside the front camera's view.

### Four cameras and the surround map

The drone has four cameras: front, right, back and left, each 90° wide and pitched 20°
down, so together they see all the way round. I assumed 90° each; if your real cameras
differ, change `FOUR_CAMERA_RIG` in `domain/models.py` and the cameras in SceneBootstrap.

* **Tracking all the way round.** Each camera has its own tracker. The front camera
  drives the follow PIDs as before. If the target shows up only in the right, back or
  left camera, the state is **TURNING (right)** etc. and the drone yaws toward it until
  the front camera has it again. So a person who walks behind the drone is not lost.
* **Obstacle map.** Every frame also carries a small depth image per camera (32x24 in
  Unity, ray-cast). `SurroundMap` turns those into a top-down grid around the drone
  (60 m square, 25 cm cells) that remembers walls after they leave the view. The
  avoidance filter uses the closer of the map distance and the drone's rangefinders, so
  it also sees things to the side and behind. The followed person is left out of the map.
* **Debug window.** With more than one camera, the Python window shows three panels:
  1. a 2x2 mosaic of the cameras, each labelled (^ FRONT, > RIGHT, < LEFT, v BACK; the
     label turns green with a TARGET tag on the camera that sees the locked person);
  2. the **top-down map**: drone in the middle, nose up, grey = obstacles, rings every
     5 m, green dot = the target, orange ring = where it should be while reacquiring;
  3. the **3D map**: the sonar/depth voxel map seen from behind and above the drone,
     blocks coloured by height (blue at the ground, red 6 m and up), the drone and the
     target on stalks down to the ground. `[` and `]` turn the view 45°.
  The Unity camera previews (top right, P toggles) carry the same labels.
* `--cams front` tracks with the front camera only (map still built);
  `--no-map` turns the map off.

### Sonar array and the 3-D map

The drone carries 18 ultrasonic sensors (`SonarArray.cs`): 8 level every 45°, 4 tilted
35° up, 4 tilted 35° down, one straight up and one straight down. Each is a 30° cone,
0.2–7 m, 15 Hz. Like a real one it reports only the distance of the first echo
somewhere in the cone. An object must fill enough of the beam to echo, so 2–8 cm branches
are heard but only from close up: in the mock model an 8 cm branch is heard from about
5 m, 4 cm from 2 m and 2 cm from 1 m, and 8 mm twigs not at all. Range noise grows with
distance. So keep the follow speed low near trees (about 2 m/s): a 2–4 cm branch is
heard only 1–2 m before the drone reaches it.

* **3-D map.** `VoxelMap` (`usecases/voxel_map.py`) keeps a 30 x 30 x 16 m grid of
  20 cm voxels around the drone. Each echo marks the cone's cap at that distance as
  "maybe occupied" and everything nearer in the cone as free. As the drone moves, caps
  from different spots overlap only where the branch really is. Echoes that are just the
  ground under a down-tilted cone are recognised and ignored. The cameras' depth goes
  into the same grid. In the Python window, red dots are above the drone, yellow at its
  height and blue below.
* **Going over or under.** The map adds `front_up`, `front_down`, `up` and `below` to the
  ranges. When something blocks the way ahead but it's open above (a branch, a bar, a
  wire), the avoidance filter climbs over it instead of sidestepping along it. It drops
  under only if climbing isn't possible, because dropping brings it down toward the
  person. If it has climbed 3 m or tried for 4 s and is still blocked (a pole, a wall),
  it sidesteps instead. It never climbs with something within 1 m overhead, or
  descends onto something within 1 m below.
* **Scene.** SceneBootstrap now builds 10 trees with side branches at 2–5 m (2–8 cm
  thick, with 8 mm twigs on the ends) and a 6 cm branch across the path 10 m ahead of
  the target, 3 m up. The 32x24 camera depth mostly misses branches this thin, which is
  why the sonar matters. Turn them off with `branchyTreeCount = 0`,
  `branchAcrossPath` and `sonar`.
* **Real hardware notes.** Real ultrasonic sensors on a drone also suffer from propeller
  noise and downwash, cross-talk between transducers firing together, and soft
  targets such as leaves that absorb sound. 7 m of range gives about 2 s of warning at
  3.5 m/s. A 60 GHz mmWave radar (for example TI IWR6843) sees wires and branches better
  and further. The same map code works for it: send each detection as a narrow beam.

### Flight modes and failsafes

`FlightSupervisor` sits between the follow logic and the drone. Home is wherever
the drone was when telemetry first arrived.

* **Return home** (`h`, or automatic): climbs toward 6 m, flies home, then holds.
* **Low battery** (< 20%, `--low-battery`): forces return home and refuses `f` until it clears.
* **Geofence** (`--geofence 80` metres from home, 0 = off): forces return home.
* The obstacle filter still applies in every mode, so return-home also steers around things.

### Tuning with a flight log

```bash
python main.py --mode unity --tracker color --log run.csv
pip install matplotlib && python tools/plot_log.py run.csv
```

The plot shows the three errors, the commands, altitude/front range/battery and the
state timeline; the header of `plot_log.py` says how to read it for PID tuning.

### Useful flags

`--distance 5` sets the follow distance in metres (ground distance). Without it,
`--box-height 0.3` sets it (bigger box = closer; 0.3 ≈ 4.4 m for a 1.8 m person),
`--target-height 1.7` if the person you follow is not 1.8 m tall,
`--orbit 1.0` circles the target while following, `--target-class person`,
`--latency 0.08` is the camera-to-command delay the filter predicts across.
PID gains and loss timeouts live in `FollowConfig` (`usecases/follow_target.py`).

## 4. Making YOLO see the target

YOLO is trained on real people, so it won't reliably call the blocky
placeholder a "person". Swap in a humanoid:

1. Get a free character with a walk animation from **Mixamo** (download FBX for Unity,
   tick *In Place* on the walk clip) or Unity's **Starter Assets – Third Person**.
2. Make a prefab of the character with an Animator playing the walk loop.
3. Drag it into **SceneBootstrap → Target Visual Prefab**. It replaces the blocky body;
   movement still comes from TargetController.

SceneBootstrap also spawns **decoys** (`Decoy Count`, default 2) that wander near the
target in other shirt colours, to test that the lock stays on the right person.
With a prefab set, the decoys use the same character.

## 5. Going to the real drone

`main.py --mode real --camera 0,1,2,3 --mavlink udp://:14540` wires four
`OpenCVCamera`s (in the order front, right, back, left) + `MavsdkDrone` into the
*same* `FollowMission`. A real camera gives no depth, so the map is built only if you
add `--depth-model depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf`
(`pip install transformers torch`). That needs a GPU to keep up with four cameras, and I
couldn't download the model here, so it's untested. Do it in this order: props off →
PX4 SITL → tethered/field test with an RC kill switch. The MAVSDK adapter maps
the core's `up`/rad-s convention to MAVSDK's `down`/deg-s.

## Protocol (if you change either side, change both)

Every message: `uint32 BE length(type+payload) | uint8 type | payload`.

* Unity→Python `FRAME (1)`: `uint32 BE header_len | header JSON | blobs`.
  `cams: [{name, yaw_deg, len}]` lists the JPEGs in order; then `depth: [{name, w, h, len}]`
  lists float16 little-endian depth images in metres (row 0 = top, inf = nothing hit).
  A header without `cams` means one JPEG from the front camera.
  Header: `frame_id, t, pos [x,y,z] (Unity world), yaw_deg, alt, battery, vel_body [fwd,right,up], ranges {front,back,left,right,front_left,front_right,down}`,
  and optionally `sonar: {fov, min, max, beams: [[yaw_deg, pitch_deg, range], ...]}` (range < 0 = no echo).
* Python→Unity `COMMAND (2)`: JSON `{forward, right, up}` m/s, `yaw_rate` rad/s, + = turn right,
  optional `est: {has, x, y, h, r, state, modes: [{x, y, w}]}` for the ghost (`modes`
  = the top belief guesses while the target is hidden, v2).
* Python→Unity `RESET (5)`: empty.

Unity hovers if no command arrives for 0.5 s; Python hovers if frames stop for 0.5 s.

## What's tested and what isn't

* **v2 lost-target estimation (6 new tests, 55 + 1 skipped in all):** mock walls and
  shops hide people in 3-D (a 2.2 m wall can be seen over from 7 m, nobody can see into a
  shop), the map measures a known 2.2 m wall's height and says how high to climb to see
  over it, the belief keeps "stopped" alive next to "walked on", particles don't walk
  through walls, looking at an empty area moves the belief behind the hedge, and a
  closed-loop U-turn behind a wall is re-found in 3.4 s (v1: 14.9 s). Occlusion suite
  score 59.2 (v1) -> 82.2 (v2); the original 11 stay at 92.7. The Unity ghost change
  (`GhostTarget.cs`, `PythonBridge.cs`) wasn't compiled here: check the Console after
  opening Unity.

* **Sonar (6 tests):** an 8 cm branch is heard at 3 m and an 8 mm twig is not. Flying
  over open ground leaves no phantom obstacles in the 3-D map, and a branch ends up
  mapped at the right distance and height. A follow run with a 4 cm-radius branch across
  the path at flight height: without sonar the drone flew into it; with sonar it climbed
  over with about 1 m to spare and kept tracking. With sonar it still goes round a tall
  pillar instead of trying to climb it. `SonarArray.cs` compiles but hasn't been run in Unity.
* **Tested here (28 tests before the sonar):** all Python use cases, and closed-loop runs in the mock
  simulator: circling and running targets, wind, a degraded detector (5% box jitter,
  bursts of missed frames, id switches) with 100 ms camera lag, flying around a pillar
  that sits between drone and target, operator return-home, low-battery and geofence
  failsafes. Also the Unity TCP adapter end-to-end against `tools/fake_unity.py`
  (4 cameras + depth).
* **Four cameras:** a target that starts behind the drone and walks away is followed
  (TURNING, then TRACKING at ~4 m); with the front camera only it is never found.
  With all rangefinders switched off, so the map is the only obstacle information,
  the drone still went round a pillar in its path with 2 m clearance; without the map
  it hit it.
* **Occlusion:** in the mock sim, a target walking behind a 4–5 m pillar was re-found in
  5 of 5 layouts (hidden 5–7 s, back at ~4 m afterwards). The previous version (hover, then
  spin in place) re-found it in 0 of 5. The ground-position estimate averaged 0.25 m error.
* **Measured effect of the Kalman filter** in the degraded-detector run: distance held
  at 3.9 m ± 0.2 with the filter versus 6.3 m ± 0.4 on raw boxes, commands about 6x
  smoother, and 0% versus 19% of the time spent not following.
* **Compiled, not run:** the Unity C# scripts build cleanly against the Unity 2021.3
  engine API (legacy-input path), but I couldn't run Unity itself, so the scene,
  physics feel and camera stream are unverified. The Unity 6 `linearVelocity` and
  new-Input-System branches weren't compiled.
* **Not run:** the YOLO adapter with real weights, and the MAVSDK adapter.
  The mock sim draws pillars in the depth images but not in the colour frames, and the
  mock has no decoys. If Unity throws an error, paste it to me.
