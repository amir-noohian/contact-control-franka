#!/usr/bin/env python3
"""Plot force, position, and effective-normal adaptation from an IAC1 run."""

import argparse
import csv
from math import cos, degrees, isfinite, sin
from pathlib import Path

from plot_cartesian_force_control import latest_run, read_run


DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "cartesian_iac1_control"
TRACKING_DATA_DIR = DATA_DIR.with_name("cartesian_iac1_tracking_control")
HYBRID_DATA_DIR = DATA_DIR.with_name("cartesian_iac1_hybrid_control")
IAC1_COLUMNS = (
    "elapsed_s", "contact_confirmed", "force_ref_N", "normal_force_N",
    "x_m", "z_m", "x_ref_m", "z_ref_m", "angle_estimate_rad", "effective_angle_rad",
    "command_fx_N", "total_command_fz_N", "tau_command_Nm_2", "tau_command_Nm_4",
)


def plot(plt, path, data, surface_angle=None):
    sequential = "normal_frozen" in data
    if sequential:
        fig = plt.figure(figsize=(12, 9), layout="constrained")
        grid = fig.add_gridspec(3, 2)
        force = fig.add_subplot(grid[0, 0])
        angle = fig.add_subplot(grid[0, 1], sharex=force)
        motion = fig.add_subplot(grid[1, :], sharex=force)
        wrench = fig.add_subplot(grid[2, 0], sharex=force)
        torque = fig.add_subplot(grid[2, 1], sharex=force)
        for axis in (force, angle, motion):
            axis.tick_params(labelbottom=False)
        variant = "IAC1 estimation then hybrid control"
    else:
        fig, axes = plt.subplots(3, 2, sharex=True, figsize=(12, 9), layout="constrained")
        force, angle, x_position, z_position, wrench, torque = axes.flat
        variant = "IAC1 trajectory and force tracking" if "normal_command_N" in data else "IAC1 adaptive normal"
    fig.suptitle(f"{variant} · {path.name}")
    time = data["elapsed_s"]

    force.plot(time, data["force_ref_N"], "--", label="Desired force")
    force_label = ("Measured force (projected after switch)" if "normal_frozen" in data
                   else "Measured x/z force magnitude")
    force.plot(time, data["normal_force_N"], label=force_label)
    if "normal_command_N" in data and not sequential:
        force.plot(time, data["normal_command_N"], ":", label="Commanded force")
    if not sequential and "normal_command_N" not in data:
        for component, style in (("z", "-."), ("x", ":")):
            key = f"external_f{component}_N"
            if key in data:
                force.plot(time, data[key], style, label=f"Measured base-frame F{component}")
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

    switch_index = next((i for i, frozen in enumerate(data.get("normal_frozen", [])) if frozen), None)
    if sequential:
        travel = [0.0] * len(time)
        measured_travel = []
        if switch_index is not None:
            theta = data["effective_angle_rad"][switch_index]
            tx, tz = cos(theta), sin(theta)
            x, z = data["x_ref_m"], data["z_ref_m"]
            x0, z0 = x[switch_index], z[switch_index]
            travel[switch_index:] = [
                1000 * (tx * (xi - x0) + tz * (zi - z0))
                for xi, zi in zip(x[switch_index:], z[switch_index:])
            ]
            measured_travel = [
                1000 * (tx * (xi - x0) + tz * (zi - z0))
                for xi, zi in zip(data["x_m"][switch_index:], data["z_m"][switch_index:])
            ]
        motion.plot(time, travel, "--", label="Desired")
        if switch_index is not None:
            motion.plot(time[switch_index:], measured_travel, label="Measured")
        motion.set(title="Motion along the estimated surface tangent", ylabel="mm")
    else:
        for component, axis in (("x", x_position), ("z", z_position)):
            measured = data[f"{component}_m"]
            desired = data[f"{component}_ref_m"]
            origin = measured[0]
            if any(isfinite(value) for value in desired):
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
    switch = time[switch_index] if switch_index is not None else None
    for axis in fig.axes:
        if contact is not None:
            axis.axvline(contact, color="0.45", linestyle=":",
                         label="Contact confirmed" if axis is force else None)
        if switch is not None:
            axis.axvline(switch, color="tab:purple", linestyle="-.",
                         label="Normal frozen" if axis is angle else None)
        axis.legend()
        axis.grid(alpha=0.25)
    for axis in (wrench, torque):
        axis.set_xlabel("Elapsed time (s)")
    return fig


def main(default_directory=DATA_DIR, description=__doc__):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("csv", nargs="?", type=Path, help="Run CSV (default: latest run)")
    parser.set_defaults(tracking=False)
    if default_directory == DATA_DIR:
        parser.add_argument("--tracking", action="store_true",
                            help="Open the latest tracking-variant run when no CSV is given")
    parser.add_argument("--output", type=Path, help="Save the figure instead of displaying it")
    parser.add_argument("--surface-angle-deg", type=float,
                        help="Known surface angle in degrees from +x toward +z, used for plotting")
    args = parser.parse_args()
    if args.surface_angle_deg is not None and not isfinite(args.surface_angle_deg):
        parser.error("--surface-angle-deg must be finite")
    try:
        directory = TRACKING_DATA_DIR if args.tracking else default_directory
        path = args.csv if args.csv is not None else latest_run(directory)
        data = read_run(path, IAC1_COLUMNS,
                        optional=("normal_command_N", "normal_frozen",
                                  "external_fz_N", "external_fx_N"))
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
