"""Shared hardware collection, CSV export, and completion reporting."""

import csv
import json
import os
import re
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

if __package__:
    from .quasi_static import TENDONS
else:
    from quasi_static import TENDONS

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


def collect_local(raw, path, tendons, cycle_duration, cycles, args, *, label, required_bodies=()):
    import fcntl

    from control_core import open_controller, safety_limits
    from experiment_runner.hardware import Hardware
    from experiment_runner.runner import Experiments
    from experiment_runner.table import parse_table

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
            raise RuntimeError("Another collector owns the hardware.") from None
        hardware = ForcePID(None)
        hardware.validate(table)
        experiment = Experiments(path.with_suffix(""), hardware, record_poses=not args.no_poses)
        stopped = False
        run = None
        try:
            if not args.no_poses:
                print(f"Waiting for mocap: {BODY_PREFIX}*", flush=True)
                experiment.start_mocap()
                deadline = time.monotonic() + 3
                while not experiment.mocap_error and time.monotonic() < deadline:
                    try:
                        experiment.check_required_bodies(required_bodies, time.monotonic(), BODY_PREFIX)
                        break
                    except RuntimeError:
                        time.sleep(0.05)
            run, = experiment.submit(table, raw, label, 1, required_bodies, body_prefix=BODY_PREFIX)
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

    if args.no_poses:
        controls = folder / "controls.csv"
        if controls.is_file():
            with controls.open("rb") as source, path.open("xb") as destination:
                shutil.copyfileobj(source, destination)
        return report_result(path, run, controls.is_file(), stopped)

    complete = export(path, run, read_rows("poses.csv"), read_rows("controls.csv"),
                      tendons, cycle_duration, cycles)
    return report_result(path, run, complete, stopped)


def add_collection_arguments(parser):
    parser.add_argument("--no-poses", action="store_true",
                        help="record forces without subscribing to mocap")
    parser.add_argument("--name", help="Append a name to the run folder and CSV")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "output" / "experiment",
                        help="Parent folder for the run directory and CSV")
    parser.add_argument("--config", type=Path,
                        default=Path(os.environ.get("SPIROB_PINS_CONFIG", ROOT / "pins.json")))
    parser.add_argument("--max-tension-kg", type=float, help="Lower the global tension limit (kgf)")
    parser.add_argument("--max-tightening-steps", type=int, help="Override the configured tightening pulse limit")


def validate_collection_arguments(parser, args):
    if args.name is not None and not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", args.name):
        parser.error("--name must contain 1–80 letters, digits, hyphens or underscores")
    if args.max_tightening_steps is not None and not 0 < args.max_tightening_steps <= sys.maxsize:
        parser.error("Use a positive tightening pulse limit.")


def collect_trial(raw, tendons, cycle_duration, cycles, args, *, number, label, required_bodies=()):
    folder = args.output_dir.expanduser().resolve()
    folder.mkdir(parents=True, exist_ok=True)
    name = f"exp{number}"
    if args.single_mode:
        name += "_single_mode"
    stem = f'{name}_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}'
    if args.name:
        stem += f"_{args.name}"
    path = folder / f"{stem}.csv"
    from control_core.slack import load_settings, post_message

    settings = load_settings(ROOT / ".env")
    status = 1
    try:
        status = collect_local(
            raw, path, tendons, cycle_duration, cycles, args,
            label=label, required_bodies=required_bodies,
        )
        return status
    except KeyboardInterrupt:
        status = 130
        raise
    finally:
        if settings is not None:
            message = f"spirob-finished: exit={status}"
            if path.is_file():
                csv_path = path.relative_to(ROOT) if path.is_relative_to(ROOT) else path
                message += f" csv={csv_path}"
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
