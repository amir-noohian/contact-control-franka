#!/usr/bin/env python3
"""Plot a Cartesian z-force control run."""

import argparse
import csv
import math
from pathlib import Path


DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "cartesian_force_control"
REQUIRED = ("elapsed_s", "force_ref_N", "external_fz_N")


def latest_run(directory=DATA_DIR):
    numbered = [path for path in directory.glob("*.csv")
                if path.stem.isascii() and path.stem.isdecimal()]
    for path in sorted(numbered, key=lambda path: int(path.stem), reverse=True):
        with path.open(newline="") as stream:
            if next(csv.DictReader(stream), None) is not None:
                return path
    raise ValueError(f"No run with recorded samples found in {directory}")


def read_run(path, required=REQUIRED, optional=("force_target_N",)):
    with path.open(newline="") as stream:
        rows = csv.DictReader(stream)
        fields = set(rows.fieldnames or ())
        missing = set(required) - fields
        if missing:
            raise ValueError(f"Missing columns: {', '.join(sorted(missing))}")
        names = (*required, *(name for name in optional if name in fields))
        data = {key: [] for key in names}
        for row in rows:
            try:
                for key, values in data.items():
                    values.append(float(row[key]))
            except (TypeError, ValueError) as error:
                raise ValueError(f"Invalid numeric value on CSV line {rows.line_num}") from error
    if not data["elapsed_s"]:
        raise ValueError("The CSV contains no samples")
    return data


def plot(plt, path, data, target):
    fig, ax = plt.subplots(figsize=(8.5, 4.4), layout="constrained")
    time = data["elapsed_s"]
    if target is not None:
        ax.axhline(target, linestyle="--", label=f"Desired target ({target:g} N)")
    elif "force_target_N" in data:
        ax.plot(time, data["force_target_N"], "--", label="Desired target")
    else:
        ax.plot(time, data["force_ref_N"], "--", label="Desired (ramped reference)")
    ax.plot(time, data["external_fz_N"], label="Measured (estimated external force)")
    ax.set(title=f"Base-frame z force · {path.name}", xlabel="Elapsed time (s)",
           ylabel="Reaction force on robot (N)")
    ax.legend()
    ax.grid(alpha=0.25)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, help="Run CSV (default: latest run)")
    parser.add_argument("--output", type=Path, help="Save the figure instead of displaying it")
    parser.add_argument("--target", type=float,
                        help="Override the desired target in N, for example for older logs")
    args = parser.parse_args()
    if args.target is not None and not math.isfinite(args.target):
        parser.error("--target must be finite")
    try:
        path = args.csv if args.csv is not None else latest_run()
        data = read_run(path)
        from matplotlib import pyplot as plt

        if args.output:
            plt.switch_backend("Agg")
        fig = plot(plt, path, data, args.target)
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
