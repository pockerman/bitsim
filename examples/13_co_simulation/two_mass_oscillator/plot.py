"""Plot x1 over time, from the CSV two_mass_oscillator writes.

Usage: python plot.py oscillator.csv two_mass_oscillator.png
"""
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main(path, out):
    with open(path) as f:
        rows = list(csv.reader(f))
    names, data = rows[0], [[float(v) for v in r] for r in rows[1:]]
    t = [r[0] for r in data]
    fig, ax = plt.subplots(figsize=(8, 4))
    styles = {"whole system": dict(color="black", lw=2.5, alpha=0.35)}
    for j, name in enumerate(names[1:], start=1):
        ax.plot(t, [r[j] for r in data], label=name, **styles.get(name, dict(lw=1.2)))
    ax.set_xlabel("t [s]")
    ax.set_ylabel("x1 [m]")
    ax.set_ylim(-2.5, 2.5)
    ax.set_title("Two masses coupled by a stiff spring, macro step 0.02 s")
    ax.legend(loc="lower left", fontsize=8, ncol=3)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=110)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
