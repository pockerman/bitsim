"""Plot the speeds and the tow bar force, from the CSV distributed_towing writes.

Usage: python plot.py towing.csv distributed_towing.png
"""
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main(path, out):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    col = lambda name: [float(r[name]) for r in rows]
    t = col("t")
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)
    top.plot(t, col("target"), "k--", lw=1, label="target speed")
    top.plot(t, col("robot"), lw=1.5, label="robot")
    top.plot(t, col("trailer"), lw=1.5, label="trailer")
    top.set_ylabel("speed [m/s]")
    top.legend(fontsize=8)
    top.grid(alpha=0.3)
    bottom.plot(t, col("force loose"), lw=0.8, color="#d62728", label="loose coupling")
    bottom.plot(t, col("force whole system"), lw=3, color="black", alpha=0.3, label="whole system")
    bottom.plot(t, col("force strong"), lw=1.2, color="#1f77b4", label="strong coupling")
    bottom.set_ylabel("force of the bar on the robot [N]")
    bottom.set_xlabel("t [s]")
    bottom.set_ylim(-150, 150)
    bottom.legend(fontsize=8, loc="lower right")
    bottom.grid(alpha=0.3)
    top.set_title("A robot tows a trailer on a stiff bar, macro step 0.01 s")
    fig.tight_layout()
    fig.savefig(out, dpi=110)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
