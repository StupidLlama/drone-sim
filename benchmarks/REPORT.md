# Benchmark report

Mock-sim scenarios, same seeds every run (`python tools/benchmark.py --version vN`).
on = % of time locked on the real target, wrong = % locked on someone else, lost = % with no target, dist = mean / min metres, close = s under 2 m, relock = s from first loss back to the real target.

| version | score | notes | created |
|---|---|---|---|
| **v0** | 79.6 | baseline: same code, appearance memory off (position-only re-lock) | 2026-11-17T18:18 |
| **v1** | 92.7 | fly-over fix + appearance memory (colour) + id-swap check + ghost target | 2026-11-17T18:21 |

### circle_walk
_walks a 6 m circle at 1.2 m/s_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 4.3 / 4.08 | 0.0 |  |  | 20.73 / 25.26 |
| v1 | 100.0 | 0.0 | 0.0 | 4.3 / 4.08 | 0.0 |  |  | 22.59 / 29.22 |

### run_straight
_runs straight at 2.5 m/s_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 4.75 / 4.09 | 0.0 |  |  | 20.37 / 23.22 |
| v1 | 100.0 | 0.0 | 0.0 | 4.75 / 4.09 | 0.0 |  |  | 21.73 / 27.35 |

### walks_at_drone
_walks straight at the drone (drone 6 m up)_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 3.05 / 2.18 | 0.0 |  |  | 21.38 / 23.57 |
| v1 | 100.0 | 0.0 | 0.0 | 3.05 / 2.18 | 0.0 |  |  | 21.94 / 27.16 |

### high_start
_drone starts 7 m up, person stands still_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 3.99 / 3.82 | 0.0 |  |  | 20.79 / 23.64 |
| v1 | 100.0 | 0.0 | 0.0 | 3.99 / 3.82 | 0.0 |  |  | 22.32 / 27.57 |

### pillar_occlusion
_walks behind a 5 m wide pillar_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 81.6 | 0.0 | 18.4 | 6.46 / 3.95 | 0.0 | 5.9 | 1.66 | 21.3 / 27.42 |
| v1 | 81.6 | 0.0 | 18.4 | 6.46 / 3.95 | 0.0 | 5.9 | 1.66 | 21.82 / 27.43 |

### decoy_at_exit
_steps into a shop for 8 s; meanwhile someone in the SAME red shirt (green trousers) walks on exactly where the target was heading_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 9.7 | 87.7 | 2.6 | 4.7 / 1.06 | 2.7 | never |  | 21.62 / 26.38 |
| v1 | 68.6 | 0.0 | 31.4 | 4.49 / 1.02 | 3.0 | 11.6 |  | 21.11 / 26.69 |

### crossing_swap
_a look-alike (same shirt) crosses right in front; tracker swaps their ids_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 98.1 | 1.9 | 0.0 | 4.3 / 3.55 | 0.0 | 0.5 |  | 20.44 / 22.11 |
| v1 | 92.0 | 1.5 | 6.5 | 4.42 / 3.55 | 0.0 | 0.4 |  | 22.36 / 28.98 |

### crowd_walk
_circle walk with 3 other people wandering nearby (different clothes)_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 43.1 | 56.9 | 0.0 | 5.4 / 2.6 | 0.0 | 14.8 |  | 23.53 / 28.66 |
| v1 | 98.4 | 1.6 | 0.0 | 4.27 / 3.51 | 0.0 | 0.6 |  | 22.15 / 27.81 |

### noisy_detector
_missed frames, jitter, 100 ms lag, wind_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 4.41 / 3.93 | 0.0 |  |  | 21.58 / 27.15 |
| v1 | 100.0 | 0.0 | 0.0 | 4.41 / 3.93 | 0.0 |  |  | 22.48 / 28.44 |

### branch_climb
_4 cm branch across the path at flight height (sonar)_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 4.42 / 4.02 | 0.0 |  | 0.98 | 32.62 / 40.97 |
| v1 | 100.0 | 0.0 | 0.0 | 4.42 / 4.02 | 0.0 |  | 0.98 | 31.65 / 40.38 |

### shirt_only
_colour tracker that only sees the red shirt (like Unity)_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 4.28 / 4.03 | 0.0 |  |  | 22.22 / 25.75 |
| v1 | 100.0 | 0.0 | 0.0 | 4.28 / 4.03 | 0.0 |  |  | 21.47 / 26.73 |
