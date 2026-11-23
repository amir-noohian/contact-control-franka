#!/usr/bin/env python3
"""Plot a hybrid Cartesian x-motion and z-force run."""

import argparse
import csv
from pathlib import Path

from plot_cartesian_force_control import REQUIRED, latest_run, read_run


DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "cartesian_hybrid_control"
HYBRID_COLUMNS = (
    *REQUIRED, "x_m", "x_ref_m", "command_fx_N", "command_fz_N", "total_command_fz_N",
    "tau_command_Nm_2", "tau_command_Nm_4",
)


def plot(plt, path, data):
    fig, axes = plt.subplots(2, 2, sharex=True, figsize=(11, 7), layout="constrained")
    fig.suptitle(f"Hybrid Cartesian control · {path.name}")
    position, force, wrench, torque = axes.flat
    time = data["elapsed_s"]

    start_x = data["x_ref_m"][0]
    for name, style, label in (("x_ref_m", "--", "Desired"), ("x_m", "-", "Measured")):
        position.plot(time, [(x - start_x) * 1000 for x in data[name]], style, label=label)
    position.set(title="Base-frame x displacement", ylabel="mm")

    force.plot(time, data["force_ref_N"], "--", label="Desired (ramped reference)")
    force.plot(time, data["external_fz_N"], label="Measured (offset-corrected estimate)")
    force.set(title="Base-frame z reaction on robot", ylabel="N")

    wrench.plot(time, data["command_fx_N"], label="Total x command")
    wrench.plot(time, data["command_fz_N"], "--", label="z force-loop contribution")
    wrench.plot(time, data["total_command_fz_N"], label="Total z command")
    wrench.set(title="Commanded base-frame wrench components", ylabel="N")

    for joint in (2, 4):
        torque.plot(time, data[f"tau_command_Nm_{joint}"], label=f"Joint {joint}")
    torque.set(title="Limited joint torque commands", ylabel="N·m")

    for ax in axes[-1]:
        ax.set_xlabel("Elapsed time (s)")
    for ax in axes.flat:
        ax.legend()
        ax.grid(alpha=0.25)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, help="Run CSV (default: latest hybrid run)")
    parser.add_argument("--output", type=Path, help="Save the figure instead of displaying it")
    args = parser.parse_args()
    try:
        path = args.csv if args.csv is not None else latest_run(DATA_DIR)
        data = read_run(path, HYBRID_COLUMNS)
        from matplotlib import pyplot as plt

        if args.output:
            plt.switch_backend("Agg")
        fig = plot(plt, path, data)
        if args.output:
            fig.savefig(args.output, dpi=160)
            print(f"Saved {args.output}")
        else:
            plt.show()
    except ModuleNotFoundError:
        parser.exit(1, "Matplotlib is required. Activate the repository's .venv.\n")
    except (OSError, ValueError, csv.Error) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
