#!/usr/bin/env python3
"""Plot estimated external force and joint torques from a hybrid run."""

import argparse
import csv
from pathlib import Path

from plot_cartesian_force_control import latest_run, read_run


DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "cartesian_hybrid_external_control"
FORCES = tuple(f"external_f{axis}_N" for axis in "xyz")
JOINTS = range(1, 8)
TORQUES = tuple(f"tau_external_Nm_{joint}" for joint in JOINTS)
COLUMNS = ("elapsed_s", *FORCES, "force_ref_N", "x_ref_m", "x_m", *TORQUES)


def plot(plt, path, data):
    figure, (force, position, torque) = plt.subplots(
        3, 1, sharex=True, figsize=(10, 9), layout="constrained"
    )
    figure.suptitle(f"Hybrid external force and torque · {path.name}")
    time = data["elapsed_s"]
    for axis, column in zip("xyz", FORCES):
        force.plot(time, data[column], label=f"Measured F{axis}")
    force.plot(time, data["force_ref_N"], "--", color=force.lines[-1].get_color(),
               label="Desired Fz")
    force.set(title="External force on the tool in the robot base frame", ylabel="N")

    position.plot(time, data["x_ref_m"], "--", label="Desired")
    position.plot(time, data["x_m"], label="Measured")
    position.set(title="Base-frame x position", ylabel="m")

    for joint, column in zip(JOINTS, TORQUES):
        torque.plot(time, data[column], label=f"Joint {joint}")
    torque.set(title="Estimated external joint torques", ylabel="N·m", xlabel="Elapsed time (s)")

    force.legend(ncol=2)
    position.legend()
    torque.legend(ncol=4)
    for axis in (force, position, torque):
        axis.grid(alpha=0.25)
    return figure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, help="Run CSV (default: latest external run)")
    parser.add_argument("--output", type=Path, help="Save the figure instead of displaying it")
    args = parser.parse_args()
    try:
        path = args.csv if args.csv is not None else latest_run(DATA_DIR)
        data = read_run(path, COLUMNS, optional=())
        from matplotlib import pyplot as plt

        if args.output:
            plt.switch_backend("Agg")
        figure = plot(plt, path, data)
        if args.output:
            figure.savefig(args.output, dpi=160)
            print(f"Saved {args.output}")
        else:
            plt.show()
    except ModuleNotFoundError:
        parser.exit(1, "Matplotlib is required. Activate the repository's .venv.\n")
    except (OSError, ValueError, csv.Error) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
