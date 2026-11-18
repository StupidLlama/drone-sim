# Benchmark report

Mock-sim scenarios, same seeds every run (`python tools/benchmark.py --version vN --suite all`).
on = % of time locked on the real target, wrong = % locked on someone else, lost = % with no target, dist = mean / min metres, close = s under 2 m, relock = s from first loss back to the real target.
Suite 2 (occlusion suite, v2+): ghost = mean metres from the ghost to the real person while lost, top3 = % of lost time the person is within 2 m of one of the 3 heaviest guesses, refind = s from the person being visible again to back on them.

| version | suite 1 score (11 original) | suite 2 score (occlusion) | notes | created |
|---|---|---|---|---|
| **v0** | 79.6 |  | baseline: same code, appearance memory off (position-only re-lock) | 2026-11-17T18:18 |
| **v1** | 92.7 |  | fly-over fix + appearance memory (colour) + id-swap check + ghost target | 2026-11-17T18:21 |
| **v1-rerun** | 92.7 | 59.2 | v1 code path (--legacy) re-run on both suites, as the v2 baseline | 2026-11-18T13:46 |
| **v2** | 92.7 | 82.2 | particle belief (walk/turn/stop/inside, map walls) + negative information + viewpoint search + obstacle-height query | 2026-11-18T13:47 |

## Suite 1

### circle_walk
_walks a 6 m circle at 1.2 m/s_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 4.3 / 4.08 | 0.0 |  |  | 20.73 / 25.26 |
| v1 | 100.0 | 0.0 | 0.0 | 4.3 / 4.08 | 0.0 |  |  | 22.59 / 29.22 |
| v1-rerun | 100.0 | 0.0 | 0.0 | 4.3 / 4.08 | 0.0 |  |  | 27.31 / 32.67 |
| v2 | 100.0 | 0.0 | 0.0 | 4.3 / 4.08 | 0.0 |  |  | 29.28 / 36.03 |

### run_straight
_runs straight at 2.5 m/s_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 4.75 / 4.09 | 0.0 |  |  | 20.37 / 23.22 |
| v1 | 100.0 | 0.0 | 0.0 | 4.75 / 4.09 | 0.0 |  |  | 21.73 / 27.35 |
| v1-rerun | 100.0 | 0.0 | 0.0 | 4.75 / 4.09 | 0.0 |  |  | 28.46 / 33.05 |
| v2 | 100.0 | 0.0 | 0.0 | 4.75 / 4.09 | 0.0 |  |  | 26.35 / 33.52 |

### walks_at_drone
_walks straight at the drone (drone 6 m up)_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 3.05 / 2.18 | 0.0 |  |  | 21.38 / 23.57 |
| v1 | 100.0 | 0.0 | 0.0 | 3.05 / 2.18 | 0.0 |  |  | 21.94 / 27.16 |
| v1-rerun | 100.0 | 0.0 | 0.0 | 3.05 / 2.18 | 0.0 |  |  | 26.24 / 32.02 |
| v2 | 100.0 | 0.0 | 0.0 | 3.05 / 2.18 | 0.0 |  |  | 24.89 / 31.63 |

### high_start
_drone starts 7 m up, person stands still_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 3.99 / 3.82 | 0.0 |  |  | 20.79 / 23.64 |
| v1 | 100.0 | 0.0 | 0.0 | 3.99 / 3.82 | 0.0 |  |  | 22.32 / 27.57 |
| v1-rerun | 100.0 | 0.0 | 0.0 | 3.99 / 3.82 | 0.0 |  |  | 25.74 / 31.37 |
| v2 | 100.0 | 0.0 | 0.0 | 3.99 / 3.82 | 0.0 |  |  | 27.7 / 32.72 |

### pillar_occlusion
_walks behind a 5 m wide pillar_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 81.6 | 0.0 | 18.4 | 6.46 / 3.95 | 0.0 | 5.9 | 1.66 | 21.3 / 27.42 |
| v1 | 81.6 | 0.0 | 18.4 | 6.46 / 3.95 | 0.0 | 5.9 | 1.66 | 21.82 / 27.43 |
| v1-rerun | 81.6 | 0.0 | 18.4 | 6.46 / 3.95 | 0.0 | 5.9 | 1.66 | 25.76 / 32.75 |
| v2 | 81.6 | 0.0 | 18.4 | 6.46 / 3.95 | 0.0 | 5.9 | 1.66 | 25.0 / 31.26 |

### decoy_at_exit
_steps into a shop for 8 s; meanwhile someone in the SAME red shirt (green trousers) walks on exactly where the target was heading_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 9.7 | 87.7 | 2.6 | 4.7 / 1.06 | 2.7 | never |  | 21.62 / 26.38 |
| v1 | 68.6 | 0.0 | 31.4 | 4.49 / 1.02 | 3.0 | 11.6 |  | 21.11 / 26.69 |
| v1-rerun | 68.6 | 0.0 | 31.4 | 4.49 / 1.02 | 3.0 | 11.6 |  | 26.92 / 32.61 |
| v2 | 68.6 | 0.0 | 31.4 | 4.49 / 1.02 | 3.0 | 11.6 |  | 28.47 / 32.31 |

### crossing_swap
_a look-alike (same shirt) crosses right in front; tracker swaps their ids_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 98.1 | 1.9 | 0.0 | 4.3 / 3.55 | 0.0 | 0.5 |  | 20.44 / 22.11 |
| v1 | 92.0 | 1.5 | 6.5 | 4.42 / 3.55 | 0.0 | 0.4 |  | 22.36 / 28.98 |
| v1-rerun | 92.0 | 1.5 | 6.5 | 4.42 / 3.55 | 0.0 | 0.4 |  | 26.15 / 31.99 |
| v2 | 92.0 | 1.5 | 6.5 | 4.42 / 3.55 | 0.0 | 0.4 |  | 25.11 / 30.42 |

### crowd_walk
_circle walk with 3 other people wandering nearby (different clothes)_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 43.1 | 56.9 | 0.0 | 5.4 / 2.6 | 0.0 | 14.8 |  | 23.53 / 28.66 |
| v1 | 98.4 | 1.6 | 0.0 | 4.27 / 3.51 | 0.0 | 0.6 |  | 22.15 / 27.81 |
| v1-rerun | 98.4 | 1.6 | 0.0 | 4.27 / 3.51 | 0.0 | 0.6 |  | 28.05 / 33.43 |
| v2 | 98.4 | 1.6 | 0.0 | 4.27 / 3.51 | 0.0 | 0.6 |  | 27.06 / 32.67 |

### noisy_detector
_missed frames, jitter, 100 ms lag, wind_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 4.41 / 3.93 | 0.0 |  |  | 21.58 / 27.15 |
| v1 | 100.0 | 0.0 | 0.0 | 4.41 / 3.93 | 0.0 |  |  | 22.48 / 28.44 |
| v1-rerun | 100.0 | 0.0 | 0.0 | 4.41 / 3.93 | 0.0 |  |  | 28.39 / 33.29 |
| v2 | 100.0 | 0.0 | 0.0 | 4.41 / 3.93 | 0.0 |  |  | 28.35 / 32.34 |

### branch_climb
_4 cm branch across the path at flight height (sonar)_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 4.42 / 4.02 | 0.0 |  | 0.98 | 32.62 / 40.97 |
| v1 | 100.0 | 0.0 | 0.0 | 4.42 / 4.02 | 0.0 |  | 0.98 | 31.65 / 40.38 |
| v1-rerun | 100.0 | 0.0 | 0.0 | 4.42 / 4.02 | 0.0 |  | 0.98 | 44.12 / 51.21 |
| v2 | 100.0 | 0.0 | 0.0 | 4.42 / 4.02 | 0.0 |  | 0.98 | 44.95 / 52.86 |

### shirt_only
_colour tracker that only sees the red shirt (like Unity)_

| version | on % | wrong % | lost % | dist mean/min | close s | relock s | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|
| v0 | 100.0 | 0.0 | 0.0 | 4.28 / 4.03 | 0.0 |  |  | 22.22 / 25.75 |
| v1 | 100.0 | 0.0 | 0.0 | 4.28 / 4.03 | 0.0 |  |  | 21.47 / 26.73 |
| v1-rerun | 100.0 | 0.0 | 0.0 | 4.28 / 4.03 | 0.0 |  |  | 27.88 / 34.01 |
| v2 | 100.0 | 0.0 | 0.0 | 4.28 / 4.03 | 0.0 |  |  | 26.96 / 34.05 |

## Suite 2 (occlusion)

### stop_behind_pillar
_walks behind a 5 m wide pillar and stops in its shadow (10 s / 5 s), then walks on_

| version | variant | on % | wrong % | close s | relock s | refind s | ghost m | @5s / @10s | top3 % | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v1-rerun | stop 10 s | 86.9 | 0.0 | 0.0 | 4.8 | 0.5 | 1.46 |  /  | 71.1 | 2.02 | 30.07 / 41.01 |
| v1-rerun | stop 5 s | 86.9 | 0.0 | 0.0 | 4.8 | 0.5 | 1.46 |  /  | 71.1 | 2.02 | 30.46 / 40.65 |
| v2 | stop 10 s | 89.5 | 0.0 | 0.0 | 3.9 | 0.5 | 1.04 |  /  | 96.2 | 2.31 | 31.79 / 42.97 |
| v2 | stop 5 s | 89.5 | 0.0 | 0.0 | 3.9 | 0.5 | 1.04 |  /  | 96.2 | 2.31 | 32.07 / 41.74 |

### turn_at_corner
_walks up the side of an 8x8 m building and turns round its far corner (keeps going / stops 8 s behind it)_

| version | variant | on % | wrong % | close s | relock s | refind s | ghost m | @5s / @10s | top3 % | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v1-rerun | keeps going | 87.9 | 0.0 | 0.0 | 5.1 | 0.5 | 0.97 | 1.58 /  | 100.0 | 1.05 | 32.15 / 41.04 |
| v1-rerun | stops behind it | 72.9 | 0.0 | 2.8 | 11.4 | 6.8 | 2.77 | 2.04 / 4.96 | 43.9 | 1.49 | 31.98 / 42.45 |
| v2 | keeps going | 93.9 | 0.0 | 0.0 | 2.6 | 0.5 | 0.92 |  /  | 100.0 | 1.04 | 32.76 / 41.87 |
| v2 | stops behind it | 93.9 | 0.0 | 0.0 | 2.6 | 0.5 | 0.92 |  /  | 100.0 | 0.69 | 32.96 / 42.42 |

### u_turn_hidden
_goes behind a 12 m wall, turns back after 4 / 6.5 m, comes out where it went in_

| version | variant | on % | wrong % | close s | relock s | refind s | ghost m | @5s / @10s | top3 % | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v1-rerun | turns at 4 m | 64.6 | 0.0 | 1.6 | 14.9 | 0.6 | 7.37 | 6.54 / 9.71 | 20.9 | 2.52 | 33.24 / 43.72 |
| v1-rerun | turns at 6.5 m | 66.5 | 0.0 | 1.6 | 14.1 | 10.1 | 5.68 | 2.49 / 8.6 | 22.8 | 2.52 | 33.18 / 44.02 |
| v2 | turns at 4 m | 92.0 | 0.0 | 0.0 | 3.4 | 0.5 | 0.91 |  /  | 100.0 | 2.86 | 35.01 / 43.36 |
| v2 | turns at 6.5 m | 92.0 | 0.0 | 0.8 | 3.4 | 0.5 | 1.35 |  /  | 98.5 | 1.66 | 33.13 / 44.26 |

### enters_building
_walks into a 10x6 m shop, stays 15 s, leaves by the same door / a second door_

| version | variant | on % | wrong % | close s | relock s | refind s | ghost m | @5s / @10s | top3 % | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v1-rerun | same door | 40.7 | 0.0 | 4.2 | 27.8 | 0.5 | 5.58 | 2.19 / 4.13 | 17.2 | 1.47 | 33.63 / 42.98 |
| v1-rerun | other door | 15.3 | 0.0 | 4.3 | never | never | 9.31 | 2.19 / 4.13 | 12.1 | 1.47 | 32.24 / 41.54 |
| v2 | same door | 59.7 | 0.0 | 0.0 | 19.0 | 0.5 | 2.05 | 2.26 / 2.26 | 57.3 | 3.02 | 37.53 / 71.43 |
| v2 | other door | 44.4 | 0.0 | 0.0 | 26.2 | 0.5 | 4.21 | 2.26 / 2.26 | 31.2 | 2.44 | 33.62 / 54.94 |

### decoy_at_exit_geo
_steps into a real shop for 8 / 12 s; a same-shirt decoy walks on where it was heading_

| version | variant | on % | wrong % | close s | relock s | refind s | ghost m | @5s / @10s | top3 % | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v1-rerun | 8 s inside | 66.5 | 0.0 | 4.2 | 14.1 | 0.5 | 4.98 | 3.82 / 7.52 | 21.4 | 2.49 | 31.49 / 41.86 |
| v1-rerun | 12 s inside | 43.3 | 0.0 | 4.2 | 23.8 | 0.6 | 6.65 | 3.82 / 5.87 | 12.6 | 2.49 | 31.78 / 41.91 |
| v2 | 8 s inside | 76.5 | 0.0 | 0.0 | 9.9 | 0.5 | 1.09 | 1.19 /  | 100.0 | 2.35 | 34.29 / 45.22 |
| v2 | 12 s inside | 67.0 | 0.0 | 0.0 | 13.9 | 0.5 | 1.12 | 1.19 / 1.19 | 100.0 | 2.35 | 35.63 / 47.64 |

### low_wall_see_over
_walks round the end of a 2.2 m wall and along behind it, stops 10 s / keeps walking; the drone can see over by climbing_

| version | variant | on % | wrong % | close s | relock s | refind s | ghost m | @5s / @10s | top3 % | clearance m | tick ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v1-rerun | stops 10 s | 93.6 | 0.0 | 0.0 | 2.4 | 0.5 | 0.89 |  /  | 100.0 | 1.79 | 33.23 / 42.58 |
| v1-rerun | keeps walking | 93.6 | 0.0 | 0.0 | 2.4 | 0.5 | 0.91 |  /  | 100.0 | 1.25 | 31.53 / 42.21 |
| v2 | stops 10 s | 95.7 | 0.0 | 0.0 | 1.6 | 0.5 | 0.84 |  /  | 100.0 | 0.83 | 32.29 / 43.28 |
| v2 | keeps walking | 95.7 | 0.0 | 0.0 | 1.6 | 0.5 | 0.84 |  /  | 100.0 | 0.99 | 33.5 / 42.77 |
