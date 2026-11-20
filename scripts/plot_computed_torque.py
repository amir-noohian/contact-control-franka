#!/usr/bin/env python3
"""Plot a computed-torque experiment CSV."""

import argparse
import csv
from pathlib import Path


DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "computed_torque"
MOVING_JOINTS = (2, 4)
HOLDING_JOINTS = (1, 3, 5, 6, 7)
FORCE_COMPONENTS = ("external_fx_N", "external_fy_N", "external_fz_N")
FORCE_NORM = "external_force_norm_N"
FORCE_COLUMNS = (*FORCE_COMPONENTS, FORCE_NORM)
PANELS = (
    ("Position", "rad", "q_rad", "q_ref_rad", "Reference"),
    ("Velocity", "rad/s", "dq_rad_s", "dq_ref_rad_s", "Reference"),
    ("Torque", "N·m", "tau_measured_Nm", "tau_command_Nm", "Requested"),
)


def latest_run():
    logs = (path for path in DATA_DIR.glob("*.csv") if path.stem.isascii() and path.stem.isdecimal())
    for path in sorted(logs, key=lambda path: int(path.stem), reverse=True):
        with path.open(newline="") as stream:
            if next(csv.DictReader(stream), None) is not None:
                return path
    return None


def read_columns(path):
    names = ["elapsed_s"]
    for joint in MOVING_JOINTS:
        for _, _, measured, reference, _ in PANELS:
            names.extend((f"{measured}_{joint}", f"{reference}_{joint}"))
    for joint in HOLDING_JOINTS:
        names.extend((f"q_rad_{joint}", f"q_ref_rad_{joint}", f"tau_command_Nm_{joint}"))

    with path.open(newline="") as stream:
        rows = csv.DictReader(stream)
        missing = set(names).difference(rows.fieldnames or ())
        if missing:
            raise ValueError(f"Missing columns: {', '.join(sorted(missing))}")
        available_force = set(FORCE_COLUMNS).intersection(rows.fieldnames)
        if available_force:
            missing_force = set(FORCE_COLUMNS) - available_force
            if missing_force:
                raise ValueError(f"Missing force columns: {', '.join(sorted(missing_force))}")
            names.extend(FORCE_COLUMNS)
        columns = {name: [] for name in names}
        for row in rows:
            try:
                for name in names:
                    columns[name].append(float(row[name]))
            except (ValueError, TypeError) as error:
                raise ValueError(f"Invalid numeric value on CSV line {rows.line_num}") from error
    if not columns["elapsed_s"]:
        raise ValueError("The CSV contains no samples")
    return columns


def plot(plt, path, data):
    has_force = FORCE_NORM in data
    panel_rows = 5 if has_force else 4
    fig, axes = plt.subplots(panel_rows, 2, figsize=(12, 2.5 * panel_rows),
                             sharex=True, layout="constrained")
    fig.suptitle(path.name)
    time = data["elapsed_s"]

    for column, joint in enumerate(MOVING_JOINTS):
        for row, (name, unit, measured, reference, label) in enumerate(PANELS):
            ax = axes[row, column]
            ax.plot(time, data[f"{measured}_{joint}"], label="Measured")
            ax.plot(time, data[f"{reference}_{joint}"], "--", label=label)
            ax.set(title=f"Joint {joint} {name.lower()}", ylabel=unit)
            ax.legend()

    hold_error, hold_torque = axes[3]
    for joint in HOLDING_JOINTS:
        error = [measured - target for measured, target in
                 zip(data[f"q_rad_{joint}"], data[f"q_ref_rad_{joint}"])]
        hold_error.plot(time, error, label=f"J{joint}")
        hold_torque.plot(time, data[f"tau_command_Nm_{joint}"], label=f"J{joint}")
    hold_error.set(title="Holding joint position error", ylabel="rad")
    hold_torque.set(title="Holding joint torque request", ylabel="N·m")
    for ax in axes[3]:
        ax.legend(ncol=len(HOLDING_JOINTS))
    if has_force:
        force, magnitude = axes[4]
        for key, label in zip(FORCE_COMPONENTS, ("Fx", "Fy", "Fz")):
            force.plot(time, data[key], label=label)
        force.set(title="Estimated external force in robot base axes", ylabel="N")
        force.legend()
        magnitude.plot(time, data[FORCE_NORM])
        magnitude.set(title="Estimated external force magnitude", ylabel="N")

    for ax in axes[-1]:
        ax.set_xlabel("Elapsed time (s)")
    for ax in axes.flat:
        ax.grid(alpha=0.25)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, help="Run CSV (default: latest run)")
    parser.add_argument("--output", type=Path, help="Save the figure instead of displaying it")
    args = parser.parse_args()

    try:
        path = args.csv or latest_run()
        if path is None:
            raise ValueError(f"No run CSV with recorded samples found in {DATA_DIR}. Pass an existing CSV path.")
        data = read_columns(path)
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
        parser.exit(1, "Matplotlib is required. Install it in your Python environment.\n")
    except (OSError, ValueError, csv.Error) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
