#!/usr/bin/env python3
"""Plot force, position, and effective-normal adaptation from an IAC1 run."""

import argparse
import csv
from math import degrees, isfinite
from pathlib import Path

from plot_cartesian_force_control import latest_run, read_run


DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "cartesian_iac1_control"
TRACKING_DATA_DIR = DATA_DIR.with_name("cartesian_iac1_tracking_control")
IAC1_COLUMNS = (
    "elapsed_s", "contact_confirmed", "force_ref_N", "normal_force_N",
    "x_m", "z_m", "x_ref_m", "z_ref_m", "angle_estimate_rad", "effective_angle_rad",
    "command_fx_N", "total_command_fz_N", "tau_command_Nm_2", "tau_command_Nm_4",
)


def plot(plt, path, data, surface_angle=None):
    fig, axes = plt.subplots(3, 2, sharex=True, figsize=(12, 9), layout="constrained")
    variant = "IAC1 trajectory and force tracking" if "normal_command_N" in data else "IAC1 adaptive normal"
    fig.suptitle(f"{variant} · {path.name}")
    force, angle, x_position, z_position, wrench, torque = axes.flat
    time = data["elapsed_s"]

    force.plot(time, data["force_ref_N"], "--", label="Desired compression")
    force.plot(time, data["normal_force_N"], label="Measured x/z force magnitude")
    if "normal_command_N" in data:
        force.plot(time, data["normal_command_N"], ":", label="Commanded compression")
    force.set(title="Contact force", ylabel="N")

    contact_index = next((i for i, value in enumerate(data["contact_confirmed"]) if value), None)
    effective = [degrees(value) for value in data["effective_angle_rad"]]
    if surface_angle is None:
        angle.plot(time, [degrees(value) for value in data["angle_estimate_rad"]],
                   "--", label="Estimator state")
        angle.plot(time, effective, label="Effective normal")
        angle.set(title="Adaptive angle", ylabel="deg")
    elif contact_index is not None:
        error = [(value - surface_angle + 180) % 360 - 180 for value in effective[contact_index:]]
        angle.plot(time[contact_index:], error, label="Effective normal error")
        angle.axhline(0, color="0.4", linestyle="--", label="Surface normal")
        angle.set(title=f"Effective normal error · initial {error[0]:+.2f}°", ylabel="deg")
    else:
        angle.plot(time, effective, label="Nominal direction during approach")
        angle.set(title="Contact unconfirmed · adaptation did not start", ylabel="deg")
    if contact_index is not None:
        angle.plot(time[contact_index],
                   effective[contact_index] if surface_angle is None else error[0],
                   "o", label="Initial effective angle" if surface_angle is None else "Initial error")

    for component, axis in (("x", x_position), ("z", z_position)):
        measured = data[f"{component}_m"]
        desired = data[f"{component}_ref_m"]
        origin = measured[0]
        axis.plot(time, [(value - origin) * 1000 for value in desired], "--", label="Desired")
        axis.plot(time, [(value - origin) * 1000 for value in measured], label="Measured")
        axis.set(title=f"Base-frame {component} displacement", ylabel="mm")

    wrench.plot(time, data["command_fx_N"], label="x")
    wrench.plot(time, data["total_command_fz_N"], label="z")
    wrench.set(title="Commanded base-frame wrench components", ylabel="N")

    for joint in (2, 4):
        torque.plot(time, data[f"tau_command_Nm_{joint}"], label=f"Joint {joint}")
    torque.set(title="Limited joint torque commands", ylabel="N·m")

    contact = time[contact_index] if contact_index is not None else None
    for axis in axes.flat:
        if contact is not None:
            axis.axvline(contact, color="0.45", linestyle=":",
                         label="Contact confirmed" if axis is force else None)
        axis.legend()
        axis.grid(alpha=0.25)
    for axis in axes[-1]:
        axis.set_xlabel("Elapsed time (s)")
    return fig


def main(default_directory=DATA_DIR):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, help="Run CSV (default: latest IAC1 run)")
    if default_directory == DATA_DIR:
        parser.add_argument("--tracking", action="store_true",
                            help="Open the latest tracking-variant run when no CSV is given")
    else:
        parser.set_defaults(tracking=True)
    parser.add_argument("--output", type=Path, help="Save the figure instead of displaying it")
    parser.add_argument("--surface-angle-deg", type=float,
                        help="Known surface angle in degrees from +x toward +z, used for plotting")
    args = parser.parse_args()
    if args.surface_angle_deg is not None and not isfinite(args.surface_angle_deg):
        parser.error("--surface-angle-deg must be finite")
    try:
        directory = TRACKING_DATA_DIR if args.tracking else default_directory
        path = args.csv if args.csv is not None else latest_run(directory)
        data = read_run(path, IAC1_COLUMNS, optional=("normal_command_N",))
        from matplotlib import pyplot as plt

        if args.output:
            plt.switch_backend("Agg")
        fig = plot(plt, path, data, args.surface_angle_deg)
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
