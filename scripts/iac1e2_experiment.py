#!/usr/bin/env python3
"""Prepare and score the IAC1E2 gain trials without editing the live config."""

import argparse
import csv
import json
import math
import os
import re
import shlex
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TRACKING = "cartesian_iac1_tracking_control"
CONFIG = ROOT / "config" / f"{TRACKING}.conf"
BLOCKS = ("ABCD", "DCBA", "BADC")


def settings(text):
    return dict(line.split() for raw in text.splitlines()
                if (line := raw.partition("#")[0].strip()))


def prepare(args):
    text = CONFIG.read_text()
    base = settings(text)
    kp, kv = args.gains or (float(base["motion_kp"]), float(base["motion_kv"]))
    if not all(math.isfinite(gain) and gain > 0 for gain in (kp, kv)):
        raise ValueError("Both baseline gains must be positive and finite.")
    if not float(base["min_target_N"]) <= args.force <= float(base["max_target_N"]):
        raise ValueError("Force target is outside the configured bounds.")
    angle = args.surface_angle
    if angle is not None and not math.isfinite(angle):
        raise ValueError("Surface angle must be finite.")
    gains = {"A": (kp, kv), "B": (0.75 * kp, kv), "C": (kp, 0.75 * kv),
             "D": (0.75 * kp, 0.75 * kv)}
    trials = [{"id": f"S{i}", "condition": "A", "dx": 0.0, "dz": 0.0}
              for i in range(1, 4)]
    if angle is not None:
        theta = math.radians(angle)
        dx, dz = 0.01 * math.cos(theta), 0.01 * math.sin(theta)
        trials += [{"id": "P", "condition": "A", "dx": dx, "dz": dz}]
        trials += [{"id": f"{block}{condition}", "condition": condition, "dx": dx, "dz": dz}
                   for block, order in enumerate(BLOCKS, 1) for condition in order]

    args.directory.mkdir(parents=True)
    for condition, (p, v) in gains.items():
        snapshot = text
        for key, value in (("motion_kp", p), ("motion_kv", v)):
            snapshot = re.sub(rf"(?m)^({key}[ \t]+)\S+",
                              lambda match: f"{match[1]}{value:.12g}", snapshot)
        (args.directory / f"{condition}.conf").write_text(snapshot)

    (args.directory / "plan.json").write_text(json.dumps({
        "force_N": args.force, "surface_angle_deg": angle, "gains": gains,
        "trajectory_duration_s": float(base["trajectory_duration_s"]),
        "run_duration_s": float(base["run_duration_s"]), "trials": trials,
    }, indent=2) + "\n")
    print(f"Prepared {len(trials)} trials in {args.directory}.")
    if angle is None:
        print("Stationary trials are ready. Moving trials require the measured surface angle.")


def trial_command(args):
    plan = json.loads((args.directory / "plan.json").read_text())
    trial = next((trial for trial in plan["trials"] if trial["id"] == args.trial), None)
    if trial is None:
        raise ValueError(f"Trial {args.trial} is not in this plan.")
    config = (args.directory / f"{trial['condition']}.conf").resolve()
    command = [str(ROOT / "build" / TRACKING), str(plan["force_N"]),
               f"{trial['dx']:.12g}", f"{trial['dz']:.12g}"]
    print(f"IAC1_TRACKING_CONFIG={shlex.quote(str(config))} {shlex.join(command)}", flush=True)
    if args.run:
        os.execve(command[0], command, {**os.environ, "IAC1_TRACKING_CONFIG": str(config)})


def summarize(args):
    with args.log.open(newline="") as stream:
        rows = [{name: float(row[name]) for name in (
            "elapsed_s", "force_target_N", "normal_force_N", "effective_angle_rad",
            "x_m", "x_ref_m", "z_m", "z_ref_m",
        )} for row in csv.DictReader(stream) if int(row["contact_confirmed"])]
    if not rows:
        print(json.dumps({"contact_confirmed": False, "stop_reason": args.stop_reason}, indent=2))
        return

    duration = rows[-1]["elapsed_s"] - rows[0]["elapsed_s"]
    contact_complete = duration >= args.duration - 0.002 and args.stop_reason is None
    angles = [math.degrees(row["effective_angle_rad"]) for row in rows]
    angle_reference = args.surface_angle if args.surface_angle is not None else angles[0]
    errors = [(value - angle_reference + 180) % 360 - 180 for value in angles]
    tail_force = [row["normal_force_N"] for row in rows
                  if row["elapsed_s"] >= rows[-1]["elapsed_s"] - 1]
    summary = {
        "confirmed_contact_s": duration, "completed": contact_complete,
        "stop_reason": args.stop_reason, "force_target_N": rows[0]["force_target_N"],
        "peak_force_N": max(row["normal_force_N"] for row in rows),
        "final_second_mean_force_N": statistics.mean(tail_force) if contact_complete else None,
        "angle_reference": "measured surface" if args.surface_angle is not None else "first sample",
        "peak_absolute_angle_difference_deg": max(map(abs, errors)),
        "final_angle_difference_deg": errors[-1],
    }
    for axis in ("x", "z"):
        errors_mm = [1000 * abs(row[f"{axis}_m"] - row[f"{axis}_ref_m"]) for row in rows
                     if math.isfinite(row[f"{axis}_ref_m"])]
        summary[f"peak_{axis}_tracking_error_mm"] = max(errors_mm, default=None)
    print(json.dumps(summary, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prep = commands.add_parser("prepare")
    prep.add_argument("directory", type=Path)
    prep.add_argument("--force", type=float, required=True)
    prep.add_argument("--surface-angle", type=float, help="Measured tangent angle in degrees")
    prep.add_argument("--gains", type=float, nargs=2, metavar=("KP", "KV"))
    prep.set_defaults(action=prepare)
    trial = commands.add_parser("trial")
    trial.add_argument("directory", type=Path)
    trial.add_argument("trial")
    trial.add_argument("--run", action="store_true", help="Open the interactive Franka controller")
    trial.set_defaults(action=trial_command)
    summary = commands.add_parser("summarize")
    summary.add_argument("log", type=Path)
    summary.add_argument("--surface-angle", type=float)
    summary.add_argument("--duration", type=float, default=12)
    summary.add_argument("--stop-reason")
    summary.set_defaults(action=summarize)
    args = parser.parse_args()
    try:
        args.action(args)
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
