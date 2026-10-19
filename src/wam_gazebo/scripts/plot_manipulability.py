#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({
    "font.size": 12,
    "axes.titlesize": 14,
    "axes.labelsize": 13,
    "axes.grid": True,
    "legend.fontsize": 10,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "lines.linewidth": 2.0,
    "grid.alpha": 0.35,
})


def main():
    parser = argparse.ArgumentParser(
        description="Plot data recorded by wam_manipulability_logger."
    )
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    data = np.genfromtxt(args.csv_file, delimiter=",", names=True)
    if data.size == 0:
        raise RuntimeError(f"No samples found in {args.csv_file}")
    data = np.atleast_1d(data)
    time = data["time"]

    figure, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    figure.patch.set_facecolor("white")

    for ax in axes.flat:
        ax.grid(True, linestyle="-", linewidth=0.8, alpha=0.4)
        ax.tick_params(axis="both", which="major", labelsize=11)

    axes[0, 0].plot(time, data["translational_manipulability"], label="translation")
    axes[0, 0].plot(time, data["rotational_manipulability"], label="rotation")
    axes[0, 0].plot(time, data["wrist_manipulability"], label="wrist")
    axes[0, 0].set_ylabel("Manipulability")
    axes[0, 0].legend(frameon=True, fancybox=True)

    axes[0, 1].plot(time, data["whole_manipulability_raw"], label="raw")
    axes[0, 1].plot(time, data["whole_manipulability_scaled"], label="scaled")
    axes[0, 1].set_ylabel("Whole manipulability")
    axes[0, 1].legend(frameon=True, fancybox=True)

    axes[1, 0].plot(time, data["sigma_min_translation"], label="translation")
    axes[1, 0].plot(time, data["sigma_min_rotation"], label="rotation")
    axes[1, 0].plot(time, data["sigma_min_full"], label="full")
    axes[1, 0].set_ylabel("Minimum singular value")
    axes[1, 0].legend(frameon=True, fancybox=True)

    axes[1, 1].plot(time, data["position_error_norm"], label="position [m]")
    axes[1, 1].plot(time, data["orientation_error_norm"], label="orientation [rad]")
    axes[1, 1].set_ylabel("Pose error norm")
    axes[1, 1].legend(frameon=True, fancybox=True)

    axes[2, 0].plot(time, data["phase"], label="phase")
    axes[2, 0].plot(time, data["phase_velocity"], label="phase velocity")
    axes[2, 0].set_xlabel("Time [s]")
    axes[2, 0].set_ylabel("Null-space path")
    axes[2, 0].legend(frameon=True, fancybox=True)

    for joint in range(1, 8):
        axes[2, 1].plot(time, data[f"q{joint}"], label=f"q{joint}")
    axes[2, 1].set_xlabel("Time [s]")
    axes[2, 1].set_ylabel("Joint position [rad]")
    axes[2, 1].legend(ncol=2, frameon=True, fancybox=True)

    figure.tight_layout()
    output = args.output or args.csv_file.with_suffix(".png")
    figure.savefig(output, dpi=220, bbox_inches="tight")
    print(f"Saved plot to {output.resolve()}")


if __name__ == "__main__":
    main()
