"""Plot a previously saved viscoelastic fit."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    """Read a fit.csv and save its measured and simulated displacement plot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fit", type=Path, help="path to fit.csv")
    args = parser.parse_args()
    data = np.loadtxt(args.fit, delimiter=",", skiprows=1, ndmin=2)
    fig, axis = plt.subplots()
    axis.plot(data[:, 0], 1000 * data[:, 1], label="Measured")
    axis.plot(data[:, 0], 1000 * data[:, 2], label="Simulated")
    axis.set(xlabel="Cycle time (s)", ylabel="Tip displacement from first observation (mm)")
    axis.legend()
    fig.tight_layout()
    path = args.fit.with_suffix(".png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved {path}")


if __name__ == "__main__":
    main()
