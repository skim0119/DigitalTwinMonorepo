"""Collect repeated quasi-static single-tendon force staircases on Spirob."""

import argparse
import csv
import json
import math
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path

if __package__:
    from .quasi_static import TENDONS, make_pair_table, make_table
else:
    from quasi_static import TENDONS, make_pair_table, make_table

BODY_PREFIX = "arm_"
ROOT = Path(__file__).resolve().parents[1]
FIELDS = ("cycle", "elapsed_s", "ros_stamp_ns", "frame", "body",
          "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
          "tendon", "target_kgf", "load_kgf", "run_status", "simulated")


def export(path, run, poses, controls, tendons, cycle_duration, cycles):
    loads = iter(row for row in controls if row[1] in tendons)
    upcoming = next(loads, None)
    latest = {}
    recorded = set()
    with path.open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        for elapsed, stamp, frame, body, *pose in poses:
            if not body.startswith(BODY_PREFIX):
                continue
            cycle = min(int(elapsed / cycle_duration), cycles - 1)
            while upcoming is not None and upcoming[0] <= elapsed:
                latest[upcoming[1]] = (upcoming[2], upcoming[4])
                upcoming = next(loads, None)
            for tendon in tendons:
                target, load = latest.get(tendon, (0, ""))
                writer.writerow((cycle + 1, elapsed - cycle * cycle_duration, stamp, frame,
                                 body, *pose, tendon, target, load, run["status"], run["simulated"]))
            recorded.add(cycle)
    return len(recorded) == cycles


def collect_local(raw, path, tendons, cycle_duration, cycles, args, *, number):
    import fcntl

    sys.path.insert(0, str(ROOT / "experiment-api"))
    from control_core import open_controller, safety_limits
    from experiment_api.hardware import Hardware
    from experiment_api.runner import Experiments
    from experiment_api.table import parse_table

    table = parse_table(raw, "force", "kgf")
    settings = json.loads(args.config.read_text())
    tightening_limit = args.max_tightening_steps
    if tightening_limit is None:
        tightening_limit = settings.get("max_tightening_steps")
    if type(tightening_limit) is not int or not 0 < tightening_limit <= sys.maxsize:
        raise ValueError("Set a positive integer max_tightening_steps in the pin config or pass --max-tightening-steps.")
    force_limit, rate_limit = safety_limits(settings)
    max_force = force_limit if args.max_tension_kg is None else args.max_tension_kg
    if not 0 < max_force <= force_limit:
        raise ValueError(f"Tension limit must be positive and at most {force_limit:g} kgf.")
    active_indices = tuple(TENDONS.index(tendon) for tendon in tendons)

    class ForcePID(Hardware):
        def validate(self, table):
            if table.mode != "force" or table.unit != "kgf":
                raise ValueError("Bending collection requires force commands in kgf.")
            if max(map(max, table.values)) >= max_force:
                raise ValueError("Every commanded force must be below the tension limit.")

        def open(self, _table, cancel):
            self.controller = open_controller(
                settings, [(-sys.maxsize, sys.maxsize)] * len(TENDONS),
                port=int(os.environ.get("SPIROB_MONITOR_PORT", "5050")),
                max_force=max_force, max_rate=rate_limit,
                max_tightening_steps=tightening_limit,
                motor=args.tendon if args.single_mode else None,
            )
            self.wait_ready(cancel)

    with (Path.home() / ".spirob-experiment-hardware.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("The Experiment API or another collector owns the hardware.") from None
        hardware = ForcePID(None)
        hardware.validate(table)
        experiment = Experiments(path.with_suffix(""), hardware)
        stopped = False
        run = None
        try:
            print(f"Waiting for mocap: {BODY_PREFIX}*", flush=True)
            experiment.start_mocap()
            deadline = time.monotonic() + 3
            while not experiment.mocap_error and time.monotonic() < deadline:
                try:
                    experiment.check_required_bodies((), time.monotonic(), BODY_PREFIX)
                    break
                except RuntimeError:
                    time.sleep(0.05)
            label = f"Exp {number} bending collection"
            if args.single_mode:
                label += " (single mode)"
            run, = experiment.submit(table, raw, label, 1, body_prefix=BODY_PREFIX)
            print(f"{label} | Recording run {run['id']}", flush=True)
            next_report = 0.0
            while experiment.worker.is_alive():
                with experiment.lock:
                    sample = next(
                        (rows for _, _, kind, rows in reversed(experiment.preview)
                         if kind == "controls"),
                        None,
                    )
                if sample is not None and sample[0][0] >= next_report:
                    elapsed = sample[0][0]
                    cycle = min(int(elapsed / cycle_duration) + 1, cycles)
                    readings = []
                    for index in active_indices:
                        _, _, target, _, load, *_ = sample[index]
                        readings.append(f"T{index} target {target:.2f} kgf, load {load:.2f} kgf")
                    print(f"Cycle {cycle}/{cycles} | {elapsed:6.1f} s | " + " | ".join(readings), flush=True)
                    next_report = elapsed + 1.0
                time.sleep(0.1)
        except KeyboardInterrupt:
            stopped = True
        finally:
            experiment.close()

    if run is None:
        return 130
    run = experiment.get(run["id"])
    folder = experiment.root / run["id"]

    def read_rows(name):
        source = folder / name
        if source.is_file():
            with source.open(newline="") as stream:
                rows = csv.reader(stream)
                next(rows, None)
                for elapsed, *values in rows:
                    yield (float(elapsed), *values)

    complete = export(path, run, read_rows("poses.csv"), read_rows("controls.csv"),
                      tendons, cycle_duration, cycles)
    return report_result(path, run, complete, stopped)


def main(*, pair=False):
    number = 2 if pair else 1
    parser = argparse.ArgumentParser(description=f"Collect quasi-static SpiRob bending data for Exp {number}.")
    parser.set_defaults(single_mode=False)
    if not pair:
        parser.add_argument("-N", "--tendon", type=int, choices=range(6), default=0)
        parser.add_argument("--single-mode", action="store_true",
                            help="apply force PID only to the selected tendon")
    parser.add_argument("-F", "--force-kgf", type=float, default=2.5)
    parser.add_argument("-C", "--cycles", type=int, choices=range(1, 31), default=1)
    increments = parser.add_mutually_exclusive_group()
    increments.add_argument("--step-kgf", type=float, default=0.2,
                            help="force increment in kgf (default: 0.2)")
    increments.add_argument("--steps", type=int, choices=range(1, 101),
                            help="use this many equal increments per phase")
    parser.add_argument("--dwell-s", type=float, default=1.0,
                        help="seconds to hold each setpoint (default: 1)")
    parser.add_argument("--name", help="Append a name to the run folder and CSV")
    parser.add_argument("--config", type=Path,
                        default=Path(os.environ.get("SPIROB_PINS_CONFIG", ROOT / "pins.json")))
    parser.add_argument("--max-tension-kg", type=float, help="Lower the global tension limit (kgf)")
    parser.add_argument("--max-tightening-steps", type=int, help="Override the configured tightening pulse limit")
    args = parser.parse_args()
    if args.name is not None and not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", args.name):
        parser.error("--name must contain 1–80 letters, digits, hyphens or underscores")
    if args.max_tightening_steps is not None and not 0 < args.max_tightening_steps <= sys.maxsize:
        parser.error("Use a positive tightening pulse limit.")
    if not all(math.isfinite(value) and value > 0 for value in (
        args.force_kgf, args.step_kgf, args.dwell_s
    )):
        parser.error("Use finite positive force, increment, and dwell values.")

    try:
        if pair:
            raw, cycle_duration = make_pair_table(
                args.force_kgf, args.cycles, args.steps, args.dwell_s, step_kgf=args.step_kgf
            )
            tendons = TENDONS[:2]
        else:
            raw, cycle_duration = make_table(
                args.tendon, args.force_kgf, args.cycles, args.steps, args.dwell_s,
                step_kgf=args.step_kgf, single_mode=args.single_mode,
            )
            tendons = (TENDONS[args.tendon],)
    except ValueError as error:
        parser.error(str(error))

    folder = ROOT / "output" / "experiment"
    folder.mkdir(parents=True, exist_ok=True)
    name = "exp1_single_mode" if args.single_mode else f"exp{number}"
    stem = f'{name}_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}'
    if args.name:
        stem += f"_{args.name}"
    path = folder / f"{stem}.csv"
    from control_core.slack import load_settings, post_message

    settings = load_settings(ROOT / ".env")
    if settings is None:
        return collect_local(raw, path, tendons, cycle_duration, args.cycles, args, number=number)
    status = 1
    try:
        status = collect_local(raw, path, tendons, cycle_duration, args.cycles, args, number=number)
        return status
    except KeyboardInterrupt:
        status = 130
        raise
    finally:
        message = f"spirob-finished: exit={status}"
        if path.is_file():
            message += f" csv={path.relative_to(ROOT)}"
        print(message, flush=True)
        try:
            post_message(settings, message)
        except RuntimeError as error:
            print(error, file=sys.stderr)
        else:
            print("Slack accepted the completion message.", flush=True)


def report_result(path, run, complete, stopped):
    print(path)
    if stopped:
        return 130
    if run["status"] != "complete" or not complete:
        print(run.get("error") or "Recording ended without an arm pose in each cycle.", file=sys.stderr)
        return 1
    return 0


def run_cli(*, pair=False):
    try:
        sys.exit(main(pair=pair))
    except (RuntimeError, OSError, ValueError) as error:
        sys.exit(str(error))


if __name__ == "__main__":
    run_cli()
