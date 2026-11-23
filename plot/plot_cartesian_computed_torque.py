#!/usr/bin/env python3
"""Plot a Cartesian computed-torque run."""

import argparse
import csv
from pathlib import Path


DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "cartesian_computed_torque"
REQUIRED_COLUMNS = (
    "elapsed_s", "x_m", "z_m", "x_ref_m", "z_ref_m",
    "tau_command_Nm_2", "tau_command_Nm_4",
    "wrench_x_N", "wrench_z_N",
)
FORCE_COLUMNS = ("external_fx_N", "external_fy_N", "external_fz_N")
OPTIONAL_COLUMNS = (
    "tau_raw_Nm_2", "tau_raw_Nm_4", "tau_measured_Nm_2", "tau_measured_Nm_4",
    "tau_command_plus_gravity_Nm_2", "tau_command_plus_gravity_Nm_4", *FORCE_COLUMNS,
)


def latest_run():
    numbered = [
        path for path in DATA_DIR.glob("*.csv")
        if path.stem.isascii() and path.stem.isdecimal()
    ]
    legacy = sorted(DATA_DIR.glob("run-*.csv"), key=lambda path: path.stat().st_mtime, reverse=True)
    for path in sorted(numbered, key=lambda path: int(path.stem), reverse=True) + legacy:
        with path.open(newline="") as stream:
            if next(csv.DictReader(stream), None) is not None:
                return path
    raise ValueError(f"No run CSV with samples found in {DATA_DIR}")


def read_columns(path):
    with path.open(newline="") as stream:
        rows = csv.DictReader(stream)
        fields = set(rows.fieldnames or ())
        missing = set(REQUIRED_COLUMNS) - fields
        if missing:
            raise ValueError(f"Missing columns: {', '.join(sorted(missing))}")
        columns = {name: [] for name in (*REQUIRED_COLUMNS, *OPTIONAL_COLUMNS) if name in fields}
        for row in rows:
            try:
                for name, values in columns.items():
                    values.append(float(row[name]))
            except (ValueError, TypeError) as error:
                raise ValueError(f"Invalid numeric value on CSV line {rows.line_num}") from error
    if not columns["elapsed_s"]:
        raise ValueError("The CSV contains no samples")
    return columns


def plot(plt, path, data):
    fig, axes = plt.subplots(4, 2, figsize=(12, 10), sharex=True, layout="constrained")
    fig.suptitle(path.name)
    time = data["elapsed_s"]
    for column, (axis, joint) in enumerate(zip(("x", "z"), (2, 4))):
        measured = data[f"{axis}_m"]
        reference = data[f"{axis}_ref_m"]
        origin = reference[0]
        position, error, torque = axes[:3, column]
        position.plot(time, [(value - origin) * 1000 for value in measured], label="Measured")
        position.plot(time, [(value - origin) * 1000 for value in reference], "--", label="Reference")
        position.set(title=f"{axis.upper()} displacement from start", ylabel="mm")
        error.plot(time, [(m - r) * 1000 for m, r in zip(measured, reference)])
        error.set(title=f"{axis.upper()} tracking error (measured − reference)", ylabel="mm")

        for field, style, label in (
            ("command", "-", "Limited command"), ("raw", "--", "Raw command"),
            ("command_plus_gravity", "-.", "Command + modeled gravity"),
            ("measured", ":", "Measured"),
        ):
            key = f"tau_{field}_Nm_{joint}"
            if key in data:
                torque.plot(time, data[key], style, label=label)
        torque.set(title=f"Joint {joint} torque", ylabel="N·m")
        position.legend()
        torque.legend()

    external, force = axes[-1]
    for axis, key in zip("xyz", FORCE_COLUMNS):
        if key in data:
            external.plot(time, data[key], label=f"F{axis}")
    external.set(title="Estimated external force in robot base axes", ylabel="N")
    if external.lines:
        external.legend()
    else:
        external.text(0.5, 0.5, "No external force recorded", ha="center", va="center",
                      transform=external.transAxes)
    force.plot(time, data["wrench_x_N"], label="Requested x")
    force.plot(time, data["wrench_z_N"], label="Requested z")
    force.set(title="Requested Cartesian force", ylabel="N")
    force.legend()
    for ax in (external, force):
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
        path = args.csv if args.csv is not None else latest_run()
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
