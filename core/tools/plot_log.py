"""Plot a flight log written by `main.py --log run.csv` (needs matplotlib).

    python tools/plot_log.py run.csv            # shows a window
    python tools/plot_log.py run.csv out.png    # saves a picture

Reading it for PID tuning:
  * error oscillates and grows       -> kp too high (or kd too low)
  * error settles but not at zero    -> raise ki
  * slow, lazy approach              -> raise kp
  * command is spiky                 -> lower kd, or raise TargetFilterConfig.r_*
"""
import csv
import sys

import matplotlib
if len(sys.argv) > 2:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


def col(rows, k):
    return [float(r[k]) if r.get(k) not in (None, "") else float("nan") for r in rows]


rows = list(csv.DictReader(open(sys.argv[1])))
t = col(rows, "t")
fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
for k, lab in (("ex", "ex (yaw)"), ("ey", "ey (vertical)"), ("ed", "ed (distance)")):
    ax[0].plot(t, col(rows, k), label=lab)
ax[0].axhline(0, color="k", lw=0.5); ax[0].set_ylabel("error"); ax[0].legend(loc="upper right")
for k in ("forward", "right", "up", "yaw_rate"):
    ax[1].plot(t, col(rows, k), label=k)
ax[1].set_ylabel("command"); ax[1].legend(loc="upper right")
ax[2].plot(t, col(rows, "alt"), label="altitude (m)")
ax[2].plot(t, col(rows, "front"), label="front range (m)")
ax[2].plot(t, [b * 10 for b in col(rows, "battery")], label="battery x10")
ax[2].legend(loc="upper right")
states = sorted({r["state"].split(" ")[0] for r in rows})
ax[3].plot(t, [states.index(r["state"].split(" ")[0]) for r in rows], drawstyle="steps-post")
ax[3].set_yticks(range(len(states))); ax[3].set_yticklabels(states); ax[3].set_xlabel("time (s)")
fig.tight_layout()
if len(sys.argv) > 2:
    fig.savefig(sys.argv[2], dpi=110)
else:
    plt.show()
